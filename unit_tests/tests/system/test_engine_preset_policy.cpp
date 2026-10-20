#include "pch.h"

#include "engine_test_helper.h"

#include <cstring>
#include <stdexcept>

TEST(EnginePresetPolicy, unavailablePresetDoesNotMutateMinimalConfiguration) {
	if (FOME_ENGINE_PRESET_ENABLED(FORD_ESCORT_GT)) {
		GTEST_SKIP() << "This policy test requires an excluded preset profile";
	}

	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);
	engine_configuration_s before = *engineConfiguration;

	EXPECT_FALSE(isEnginePresetAvailable(engine_type_e::FORD_ESCORT_GT));

	EXPECT_THROW(setEngineType(engine_type_e::FORD_ESCORT_GT), std::logic_error);
	EXPECT_EQ(engineConfiguration->engineType, engine_type_e::MINIMAL_PINS);
	EXPECT_EQ(std::memcmp(&before, engineConfiguration, sizeof(before)), 0);

	EXPECT_THROW(resetConfigurationExt(engine_type_e::FORD_ESCORT_GT), std::logic_error);
	EXPECT_EQ(engineConfiguration->engineType, engine_type_e::MINIMAL_PINS);
	EXPECT_EQ(std::memcmp(&before, engineConfiguration, sizeof(before)), 0);
}

TEST(EnginePresetPolicy, defaultAndMinimalPresetsAreAvailable) {
	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);

	EXPECT_TRUE(isEnginePresetAvailable(engine_type_e::MINIMAL_PINS));

	constexpr engine_type_e configuredDefault = engine_type_e::DEFAULT_ENGINE_TYPE;
	const engine_type_e expectedDefault =
			isEnginePresetAvailable(configuredDefault) ? configuredDefault : engine_type_e::MINIMAL_PINS;
	EXPECT_EQ(getDefaultEngineType(), expectedDefault);

	setEngineType(expectedDefault);
	EXPECT_EQ(engineConfiguration->engineType, expectedDefault);
}

#if !HW_POLYGONUS_PRESETS
TEST(EnginePresetPolicy, missingBoardPresetIsRejectedBeforeMutation) {
	if (!FOME_ENGINE_PRESET_ENABLED(POLYGONUS_MIATA_NA6)) {
		GTEST_SKIP() << "This policy test requires the preset to be selected";
	}

	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);
	engine_configuration_s before = *engineConfiguration;

	EXPECT_FALSE(isEnginePresetAvailable(engine_type_e::POLYGONUS_MIATA_NA6));

	EXPECT_THROW(setEngineType(engine_type_e::POLYGONUS_MIATA_NA6), std::logic_error);
	EXPECT_EQ(engineConfiguration->engineType, engine_type_e::MINIMAL_PINS);
	EXPECT_EQ(std::memcmp(&before, engineConfiguration, sizeof(before)), 0);
}
#endif
