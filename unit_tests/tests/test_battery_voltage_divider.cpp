/**
 * @file	test_battery_voltage_divider.cpp
 * @author	Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "engine_configuration.h"

#include <limits>

TEST(BatteryVoltageDivider, disabledEnforcesBoardNominal) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	ASSERT_FALSE(engineConfiguration->enableBatteryDividerOverride);
	engineConfiguration->vbattDividerCoeff = 42.5f;

	for (float nominal : {7.34f, 9.2f, 5.835f}) {
		setBoardBatteryVoltageDivider(nominal);
		EXPECT_FLOAT_EQ(nominal, engineConfiguration->vbattDividerCoeff);
	}
}

TEST(BatteryVoltageDivider, enabledRetainsValidCalibrationAcrossCalls) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->enableBatteryDividerOverride = true;
	engineConfiguration->vbattDividerCoeff = 8.125f;

	setBoardBatteryVoltageDivider(7.34f);
	setBoardBatteryVoltageDivider(9.2f);
	setBoardBatteryVoltageDivider(5.835f);
	EXPECT_FLOAT_EQ(8.125f, engineConfiguration->vbattDividerCoeff);

	for (float boundary : {0.01f, 99.0f}) {
		engineConfiguration->vbattDividerCoeff = boundary;
		setBoardBatteryVoltageDivider(7.34f);
		EXPECT_FLOAT_EQ(boundary, engineConfiguration->vbattDividerCoeff);
	}
}

TEST(BatteryVoltageDivider, clearingOverrideRestoresBoardNominal) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->enableBatteryDividerOverride = true;
	engineConfiguration->vbattDividerCoeff = 8.125f;
	setBoardBatteryVoltageDivider(7.34f);
	EXPECT_FLOAT_EQ(8.125f, engineConfiguration->vbattDividerCoeff);

	engineConfiguration->enableBatteryDividerOverride = false;
	setBoardBatteryVoltageDivider(5.835f);
	EXPECT_FLOAT_EQ(5.835f, engineConfiguration->vbattDividerCoeff);
}

TEST(BatteryVoltageDivider, invalidCalibrationFallsBackToBoardNominal) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->enableBatteryDividerOverride = true;

	const float invalidCoefficients[] = {
			std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::infinity(),
			-std::numeric_limits<float>::infinity(),
			0.0f,
			-1.0f,
			0.009f,
			99.001f,
	};

	for (float invalid : invalidCoefficients) {
		SCOPED_TRACE(invalid);
		engineConfiguration->vbattDividerCoeff = invalid;
		setBoardBatteryVoltageDivider(7.34f);
		EXPECT_FLOAT_EQ(7.34f, engineConfiguration->vbattDividerCoeff);
	}
}

TEST(BatteryVoltageDivider, helperDoesNotChangeUnrelatedConfiguration) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->enableBatteryDividerOverride = true;
	engineConfiguration->vbattDividerCoeff = 8.125f;
	engineConfiguration->injectorFlowAsMassFlow = true;
	engineConfiguration->vbattAdcChannel = EFI_ADC_1;

	setBoardBatteryVoltageDivider(7.34f);

	EXPECT_TRUE(engineConfiguration->enableBatteryDividerOverride);
	EXPECT_FLOAT_EQ(8.125f, engineConfiguration->vbattDividerCoeff);
	EXPECT_TRUE(engineConfiguration->injectorFlowAsMassFlow);
	EXPECT_EQ(EFI_ADC_1, engineConfiguration->vbattAdcChannel);
}

TEST(BatteryVoltageDivider, resetReappliesBoardNominalAfterPreset) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	if (!isEnginePresetAvailable(engine_type_e::DODGE_NEON_2003_CRANK)) {
		GTEST_SKIP() << "DODGE_NEON_2003_CRANK preset is unavailable";
	}

	resetConfigurationExt(
			[](engine_configuration_s*) { setBoardBatteryVoltageDivider(7.34f); },
			engine_type_e::DODGE_NEON_2003_CRANK);

	ASSERT_FALSE(engineConfiguration->enableBatteryDividerOverride);
	EXPECT_FLOAT_EQ(7.34f, engineConfiguration->vbattDividerCoeff);

	resetConfigurationExt(nullptr, engine_type_e::DODGE_NEON_2003_CRANK);

	ASSERT_FALSE(engineConfiguration->enableBatteryDividerOverride);
	EXPECT_FLOAT_EQ(9.75f, engineConfiguration->vbattDividerCoeff);
}
