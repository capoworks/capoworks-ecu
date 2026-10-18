#include "pch.h"
#include "configuration_write_state.h"
#include "configuration_storage.h"
#include "crc_accelerator.h"
#include "eficonsole.h"
#include "flash_int.h"
#include "flash_main.h"
#include "runtime_state.h"
#include "tunerstudio.h"

#include <functional>
#include <vector>

// Exercise the production entry points with only platform and storage-boundary
// hooks replaced. firmwareError returns in production, unlike the general host mock.
namespace flash_lifecycle_test {
static persistent_config_container_s persistentState;
static bool concurrent;
static unsigned errors;
static unsigned reads;
static std::vector<char> events;
static std::function<void()> duringWrite;
static std::function<void()> duringRead;
static std::function<void()> duringError;
static ConfigurationWriteResult nextResult;

bool allowFlashWhileRunning() {
	return concurrent;
}

flashaddr_t getFlashAddrFirstCopy() {
	return 0x100000;
}

flashaddr_t getFlashAddrSecondCopy() {
	return 0x200000;
}

void firmwareError(const char*, ...) {
	++errors;
	if (duringError) {
		duringError();
	}
}

auto readConfigurationCopies = [](flashaddr_t, flashaddr_t, persistent_config_container_s&) {
	events.push_back('R');
	++reads;
	if (duringRead) {
		duringRead();
	}
	return ConfigurationFlashState::Ok;
};

auto writeConfigurationCopies = [](flashaddr_t, flashaddr_t, const persistent_config_container_s& data) {
	events.push_back('W');
	EXPECT_EQ(data.version, FLASH_DATA_VERSION);
	EXPECT_EQ(data.size, sizeof(data));
	EXPECT_EQ(data.value, singleCrc(&data.persistentConfiguration, sizeof(data.persistentConfiguration)));
	if (duringWrite) {
		duringWrite();
	}
	return nextResult;
};

#undef EFI_INTERNAL_FLASH
#define EFI_INTERNAL_FLASH 1
#undef EFI_STORAGE_INT_FLASH
#define EFI_STORAGE_INT_FLASH 1
#undef EFI_FLASH_WRITE_THREAD
#define EFI_FLASH_WRITE_THREAD 0
#include "../../../firmware/controllers/flash_main.cpp"
} // namespace flash_lifecycle_test

namespace {
class FlashWriteLifecycle : public testing::Test {
protected:
	void SetUp() override {
		using namespace flash_lifecycle_test;
		concurrent = false;
		errors = 0;
		reads = 0;
		events.clear();
		duringWrite = {};
		duringRead = {};
		duringError = {};
		nextResult = {ConfigurationWritePhase::Complete, 0, FLASH_RETURN_SUCCESS, 2};
		configurationWriteState = {};
		lastWriteResult = {ConfigurationWritePhase::Complete, 0, FLASH_RETURN_SUCCESS, 0};
		burnWithoutFlash = false;

		persistentState = {};
		persistentState.size = sizeof(persistentState);
		persistentState.version = FLASH_DATA_VERSION;
		persistentState.value =
				singleCrc(&persistentState.persistentConfiguration, sizeof(persistentState.persistentConfiguration));
	}

	void TearDown() override {
		Sensor::inhibitTimeouts(false);
	}
};

TEST_F(FlashWriteLifecycle, CoalescesRequestsBeforeAutomaticWrite) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::setNeedToWriteConfiguration();
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, RequestDuringWriteSurvivesUntilNextAttempt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::setNeedToWriteConfiguration();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::duringWrite = {};
	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, RequestDuringReadSurvivesWithoutOverlappingWrite) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringRead = [] {
		flash_lifecycle_test::setNeedToWriteConfiguration();
		flash_lifecycle_test::writeToFlashIfPending();
		EXPECT_EQ(flash_lifecycle_test::events.size(), 1u);
	};
	flash_lifecycle_test::readFromFlash();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'R'}));
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'R', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, ReentrantDirectWritesDeferAndCoalesce) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::writeToFlashNow();
		flash_lifecycle_test::writeToFlashNow();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::duringWrite = {};
	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, ReadConfigIsRejectedWhileWriting) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::readFromFlash();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_EQ(flash_lifecycle_test::reads, 0u);
}

TEST_F(FlashWriteLifecycle, FailedAttemptRemainsPendingWithoutContinuousRetry) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::nextResult = {
			ConfigurationWritePhase::Verify, 0x100000, FLASH_RETURN_BAD_FLASH, 1};
	flash_lifecycle_test::duringError = [] {
		using namespace flash_lifecycle_test;
		EXPECT_FALSE(configurationWriteState.writing());
		EXPECT_TRUE(configurationWriteState.pending());
		EXPECT_FALSE(configurationWriteState.shouldWrite());
		EXPECT_EQ(lastWriteResult.phase, ConfigurationWritePhase::Verify);
		EXPECT_EQ(lastWriteResult.address, 0x100000);
		EXPECT_EQ(lastWriteResult.verifiedCopies, 1u);
	};
	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_EQ(flash_lifecycle_test::errors, 1u);

	const auto eventCount = flash_lifecycle_test::events.size();
	for (int i = 0; i < 5; ++i) {
		flash_lifecycle_test::writeToFlashIfPending();
	}
	EXPECT_EQ(flash_lifecycle_test::events.size(), eventCount);

	flash_lifecycle_test::nextResult = {ConfigurationWritePhase::Complete, 0, FLASH_RETURN_SUCCESS, 2};
	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
	EXPECT_EQ(flash_lifecycle_test::events.size(), eventCount + 1);
}

TEST_F(FlashWriteLifecycle, ConsumedRequestDoesNotTriggerDuplicateAttempt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::setNeedToWriteConfiguration();
	// A console write claims the queued request before the automatic path runs.
	flash_lifecycle_test::writeToFlashNow();
	flash_lifecycle_test::writeToFlashIfPending();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, RequestDuringFailedWriteIsEligibleForNextAttempt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::nextResult = {
			ConfigurationWritePhase::Program, 0x100000, FLASH_RETURN_PSEQERROR, 0};
	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::setNeedToWriteConfiguration();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());
	EXPECT_TRUE(flash_lifecycle_test::shouldWriteConfiguration());

	flash_lifecycle_test::duringWrite = {};
	flash_lifecycle_test::nextResult = {ConfigurationWritePhase::Complete, 0, FLASH_RETURN_SUCCESS, 2};
	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'W', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}
} // namespace
