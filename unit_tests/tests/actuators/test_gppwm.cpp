#include "pch.h"

#include "airmass_loads.h"
#include "engine_math.h"
#include "gppwm_channel.h"
#include "gppwm.h"

#include "mocks.h"

using ::testing::InSequence;
using ::testing::StrictMock;

namespace {
bool publishPhysicalLoads(
		engine_load_mode_e strategy, expected<float> measuredMap = 92.375f, expected<float> tps = 23.5f) {
	engineConfiguration->fuelAlgorithm = strategy;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);

	const auto token =
			engine->airmassInjectionState.beginCalculation(strategy, 2400, engine->getGlobalConfigurationVersion());
	AirmassInputs inputs;
	inputs.HasPublicationContext = true;
	inputs.PublicationEpoch = token;
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
	inputs.ActiveStrategy = strategy;
	inputs.Displacement = engineConfiguration->displacement;
	inputs.CylinderCount = engine->engineState.cylinderCount;
	inputs.Rpm = 2400;
	inputs.MeasuredMap = measuredMap;
	inputs.NativeMap.Map = 71.25f;
	inputs.NativeMap.Valid = true;
	inputs.NativeLoad = strategy == LM_REAL_MAF ? expected<float>(138.75f) : expected<float>(unexpected);
	inputs.Tps = tps;
	inputs.Pedal = 41.5f;
	inputs.TpsToleranceMin = -10;
	inputs.TpsToleranceMax = 110;

	const auto candidate = prepareAirmassConsumerLoads(inputs, 0.3f);
	chibios_rt::CriticalSectionLocker csl;
	auto admission = engine->airmassInjectionState.locked(csl);
	return publishAirmassConsumerLoads(candidate, admission);
}
} // namespace

TEST(GpPwm, OutputWithPwm) {
	GppwmChannel ch;

	gppwm_channel cfg;

	StrictMock<MockPwm> pwm;

	// Shouldn't throw with no config
	EXPECT_NO_THROW(ch.setOutput(10));

	{
		InSequence i;
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.25f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.75f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.0f));
		EXPECT_CALL(pwm, setSimplePwmDutyCycle(1.0f));
	}

	ch.init(true, &pwm, nullptr, nullptr, &cfg);

	// Set the output - should set directly to PWM
	ch.setOutput(25.0f);
	ch.setOutput(75.0f);

	// Test clamping behavior - should clamp to [0, 100]
	ch.setOutput(-10.0f);
	ch.setOutput(110.0f);
}

TEST(GpPwm, OutputOnOff) {
	GppwmChannel ch;

	gppwm_channel cfg;
	cfg.onAboveDuty = 50;
	cfg.offBelowDuty = 40;

	MockOutputPin pin;

	{
		InSequence i;

		// Rising edge test
		EXPECT_CALL(pin, setValue(0));
		EXPECT_CALL(pin, setValue(1));
		EXPECT_CALL(pin, setValue(1));

		// Falling edge test
		EXPECT_CALL(pin, setValue(1));
		EXPECT_CALL(pin, setValue(0));
		EXPECT_CALL(pin, setValue(0));
	}

	ch.init(false, nullptr, &pin, nullptr, &cfg);

	// Test rising edge - these should output 0, 1, 1
	ch.setOutput(49.0f);
	ch.setOutput(51.0f);
	ch.setOutput(49.0f);

	// Test falling edge - these should output 1, 0, 0
	ch.setOutput(41.0f);
	ch.setOutput(39.0f);
	ch.setOutput(41.0f);
}

TEST(GpPwm, TestGetOutput) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	GppwmChannel ch;

	gppwm_channel cfg;
	cfg.loadAxis = GPPWM_Tps;
	cfg.rpmAxis = GPPWM_Rpm;
	cfg.dutyIfError = 21.0f;

	MockVp3d table;

	EXPECT_CALL(table, getValue(1200, 35.0f)).WillRepeatedly([](float x, float tps) { return tps; });

	ch.init(false, nullptr, nullptr, &table, &cfg);

	Sensor::resetAllMocks();

	// Should return dutyIfError
	EXPECT_FLOAT_EQ(21.0f, ch.getOutput().Result);

	// Set TPS, should return tps value
	Sensor::setMockValue(SensorType::Tps1, 35.0f);
	Sensor::setMockValue(SensorType::Rpm, 1200);
	EXPECT_FLOAT_EQ(35.0f, ch.getOutput().Result);
}

TEST(GpPwm, FuelAndIgnitionLoadsUseFullPrecisionPublishedCoordinates) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.fuelingLoad = 12.0f;
	engine->engineState.ignitionLoad = 13.0f;
	engineConfiguration->ignOverrideMode = AFR_MAP;

	for (const auto& [strategy, expectedNative] :
		 {std::pair{LM_SPEED_DENSITY, 71.25f}, std::pair{LM_ALPHA_N, 23.5f}, std::pair{LM_REAL_MAF, 138.75f}}) {
		SCOPED_TRACE(static_cast<int>(strategy));
		ASSERT_TRUE(publishPhysicalLoads(strategy));
		EXPECT_FLOAT_EQ(getFuelingLoad(), expectedNative);
		EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_FuelLoad).value_or(NAN), expectedNative);
		EXPECT_FLOAT_EQ(getIgnitionLoad(), 92.375f);
		EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_IgnLoad).value_or(NAN), 92.375f);

		// Measured MAP remains the direct sensor channel, not a snapshot substitute.
		Sensor::setMockValue(SensorType::Map, 44.5f);
		EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_Map).value_or(NAN), 44.5f);
		Sensor::setInvalidMockValue(SensorType::Map);
		EXPECT_FALSE(readGppwmChannel(GPPWM_Map).Valid);

		invalidateAirmassConsumerLoads();
		EXPECT_FALSE(std::isfinite(getFuelingLoad()));
		EXPECT_FALSE(std::isfinite(getIgnitionLoad()));
		EXPECT_FALSE(readGppwmChannel(GPPWM_FuelLoad).Valid);
		EXPECT_FALSE(readGppwmChannel(GPPWM_IgnLoad).Valid);
	}
}

TEST(GpPwm, LegacyModesKeepLiveLoadFallback) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->fuelAlgorithm = LM_MOCK;
	engine->engineState.fuelingLoad = 55.5f;
	engine->engineState.ignitionLoad = 44.5f;
	engineConfiguration->ignOverrideMode = AFR_None;

	EXPECT_FLOAT_EQ(getFuelingLoad(), 55.5f);
	EXPECT_FLOAT_EQ(getIgnitionLoad(), 44.5f);
	EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_FuelLoad).value_or(NAN), 55.5f);
	EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_IgnLoad).value_or(NAN), 44.5f);

	engineConfiguration->ignOverrideMode = AFR_MAP;
	Sensor::setMockValue(SensorType::Map, 130);
	EXPECT_FLOAT_EQ(getIgnitionLoad(), 130);
	EXPECT_FLOAT_EQ(readGppwmChannel(GPPWM_IgnLoad).value_or(NAN), 130);
}

TEST(GpPwm, OptionalMapAndTpsFallbacksDoNotInvalidateFuelLoad) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	ASSERT_TRUE(publishPhysicalLoads(LM_SPEED_DENSITY, unexpected, unexpected));

	engineConfiguration->ignOverrideMode = AFR_MAP;
	EXPECT_FLOAT_EQ(getIgnitionLoad(), 200);
	engineConfiguration->ignOverrideMode = AFR_Tps;
	EXPECT_FLOAT_EQ(getIgnitionLoad(), 100);

	// The optional actuator coordinates use their historical fallback without
	// rejecting the otherwise valid native fuel load publication.
	EXPECT_FLOAT_EQ(getFuelingLoad(), 71.25f);
}
