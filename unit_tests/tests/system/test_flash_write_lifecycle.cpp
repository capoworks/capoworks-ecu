#include "pch.h"
#include "configuration_write_state.h"
#include "crc_accelerator.h"
#include "eficonsole.h"
#include "flash_int.h"
#include "flash_main.h"
#include "runtime_state.h"
#include "tunerstudio.h"

#include <functional>
#include <vector>

// Exercise the production entry points with only platform and flash I/O hooks
// replaced. firmwareError returns in production, unlike the general host mock.
namespace flash_lifecycle_test {
static persistent_config_container_s persistentState;
static bool concurrent;
static unsigned errors;
static std::vector<char> events;
static std::function<void()> duringWrite;
static std::function<void()> duringRead;
static int eraseResult;

bool allowFlashWhileRunning() {
	return concurrent;
}

flashaddr_t getFlashAddrFirstCopy() {
	return 0x100000;
}

flashaddr_t getFlashAddrSecondCopy() {
	return 0x200000;
}

int intFlashErase(flashaddr_t, size_t) {
	events.push_back('E');
	if (duringWrite) {
		duringWrite();
	}
	return eraseResult;
}

int intFlashWrite(flashaddr_t, const char* buffer, size_t size) {
	events.push_back('W');
	EXPECT_EQ(size, sizeof(persistentState));
	EXPECT_EQ(buffer, reinterpret_cast<const char*>(&persistentState));
	if (duringWrite) {
		duringWrite();
	}
	return FLASH_RETURN_SUCCESS;
}

int intFlashRead(flashaddr_t, char* buffer, size_t size) {
	events.push_back('R');
	if (duringRead) {
		duringRead();
	}
	EXPECT_EQ(size, sizeof(persistentState));
	if (buffer != reinterpret_cast<char*>(&persistentState)) {
		memcpy(buffer, &persistentState, sizeof(persistentState));
	}
	return FLASH_RETURN_SUCCESS;
}

void firmwareError(const char*, ...) {
	++errors;
}

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
		events.clear();
		duringWrite = {};
		duringRead = {};
		eraseResult = FLASH_RETURN_SUCCESS;
		configurationWriteState = {};
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

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'W', 'E', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, RequestDuringWriteSurvivesUntilNextAttempt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::setNeedToWriteConfiguration();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'W', 'E', 'W'}));
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::duringWrite = {};
	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events.size(), 8u);
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
	EXPECT_EQ(flash_lifecycle_test::events.size(), 5u);
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, ReentrantDirectWritesDeferAndCoalesce) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::writeToFlashNow();
		flash_lifecycle_test::writeToFlashNow();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'W', 'E', 'W'}));
	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());

	flash_lifecycle_test::duringWrite = {};
	flash_lifecycle_test::writeToFlashIfPending();
	EXPECT_EQ(flash_lifecycle_test::events.size(), 8u);
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}

TEST_F(FlashWriteLifecycle, ReadConfigIsRejectedWhileWriting) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::duringWrite = [] {
		flash_lifecycle_test::readFromFlash();
	};
	flash_lifecycle_test::writeToFlashNow();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'W', 'E', 'W'}));
}

TEST_F(FlashWriteLifecycle, FailedAttemptRemainsPendingWithoutContinuousRetry) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::eraseResult = FLASH_RETURN_BAD_FLASH;
	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_TRUE(flash_lifecycle_test::getNeedToWriteConfiguration());
	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'E'}));
	EXPECT_EQ(flash_lifecycle_test::errors, 2u);

	const auto eventCount = flash_lifecycle_test::events.size();
	for (int i = 0; i < 5; ++i) {
		flash_lifecycle_test::writeToFlashIfPending();
	}
	EXPECT_EQ(flash_lifecycle_test::events.size(), eventCount);

	flash_lifecycle_test::eraseResult = FLASH_RETURN_SUCCESS;
	flash_lifecycle_test::setNeedToWriteConfiguration();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
	EXPECT_EQ(flash_lifecycle_test::events.size(), eventCount + 4);
}

TEST_F(FlashWriteLifecycle, ConsumedRequestDoesNotTriggerDuplicateAttempt) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	flash_lifecycle_test::setNeedToWriteConfiguration();
	// A console write claims the queued request before the automatic path runs.
	flash_lifecycle_test::writeToFlashNow();
	flash_lifecycle_test::writeToFlashIfPending();
	flash_lifecycle_test::writeToFlashIfPending();

	EXPECT_EQ(flash_lifecycle_test::events, (std::vector<char>{'E', 'W', 'E', 'W'}));
	EXPECT_FALSE(flash_lifecycle_test::getNeedToWriteConfiguration());
}
} // namespace
