#include "pch.h"

#ifdef MODULE_BOOST

#include "airmass_loads.h"
#include "boost_control.h"

using ::testing::_;
using ::testing::StrictMock;

namespace {
bool publishPhysicalFuelLoad() {
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);

	const auto token = engine->airmassInjectionState.beginCalculation(
			LM_SPEED_DENSITY, 2000, engine->getGlobalConfigurationVersion());
	AirmassInputs inputs;
	inputs.HasPublicationContext = true;
	inputs.PublicationEpoch = token;
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
	inputs.ActiveStrategy = LM_SPEED_DENSITY;
	inputs.Displacement = engineConfiguration->displacement;
	inputs.CylinderCount = engine->engineState.cylinderCount;
	inputs.Rpm = 2000;
	inputs.MeasuredMap = 92.375f;
	inputs.NativeMap.Map = 71.25f;
	inputs.NativeMap.Valid = true;
	inputs.Tps = 23.5f;
	inputs.Pedal = 41.5f;
	inputs.TpsToleranceMin = -10;
	inputs.TpsToleranceMax = 110;

	const auto candidate = prepareAirmassConsumerLoads(inputs, 0.3f);
	chibios_rt::CriticalSectionLocker csl;
	auto admission = engine->airmassInjectionState.locked(csl);
	return publishAirmassConsumerLoads(candidate, admission);
}
} // namespace

TEST(BoostControl, LoadAxisUsesPhysicalSnapshotAndStalePublicationFallsBack) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setMockValue(SensorType::Rpm, 2000);
	engine->engineState.fuelingLoad = 12.0f;
	engineConfiguration->boostOpenLoopXAxis = GPPWM_Rpm;
	engineConfiguration->boostOpenLoopYAxis = GPPWM_FuelLoad;
	StrictMock<MockVp3d> openMap;
	EXPECT_CALL(openMap, getValue(2000, 71.25f)).WillOnce([](float, float load) { return 40.0f; });
	BoostController bc;
	bc.init(nullptr, &openMap, nullptr, nullptr);

	ASSERT_TRUE(publishPhysicalFuelLoad());
	EXPECT_FLOAT_EQ(bc.getOpenLoop(0).value_or(0), 40.0f);

	invalidateAirmassConsumerLoads();
	EXPECT_FALSE(bc.getOpenLoop(0).Valid);
}

TEST(BoostControl, Setpoint) {
	MockVp3d targetMap;

	// Just pass TPS input to output
	EXPECT_CALL(targetMap, getValue(_, _)).WillRepeatedly([](float xRpm, float tps) { return tps; });

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->boostType = CLOSED_LOOP;

	BoostController bc;

	// Should return unexpected without a pedal map cfg'd
	EXPECT_EQ(bc.getSetpoint(), unexpected);

	// Now init with mock target map
	bc.init(nullptr, nullptr, &targetMap, nullptr);

	// Should still return unxepected since TPS is invalid
	EXPECT_EQ(bc.getSetpoint(), unexpected);

	// Configure TPS, should get passthru of tps value
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 35.0f);
	EXPECT_FLOAT_EQ(bc.getSetpoint().value_or(-1), 35.0f);

	// Back in open loop mode, setpoint should be 0
	engineConfiguration->boostType = OPEN_LOOP;
	EXPECT_FLOAT_EQ(bc.getSetpoint().value_or(-1), 0);
}

TEST(BoostControl, ObservePlant) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	BoostController bc;

	Sensor::resetMockValue(SensorType::Map);
	// Check that invalid MAP returns unexpected
	EXPECT_EQ(bc.observePlant(), unexpected);

	// Test valid MAP value
	Sensor::setMockValue(SensorType::Map, 150);

	EXPECT_FLOAT_EQ(bc.observePlant().value_or(0), 150.0f);
}

TEST(BoostControl, OpenLoop) {
	MockVp3d openMap;

	// Just pass MAP input to output
	EXPECT_CALL(openMap, getValue(_, _)).WillRepeatedly([](float xRpm, float tps) { return tps; });

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	BoostController bc;

	// Without table set, should return unexpected
	EXPECT_EQ(bc.getOpenLoop(0), unexpected);

	bc.init(nullptr, &openMap, nullptr, nullptr);

	// Should pass TPS value thru
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 47.0f);
	EXPECT_FLOAT_EQ(bc.getOpenLoop(0).value_or(-1), 47.0f);
}

TEST(BoostControl, TestClosedLoop) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	BoostController bc;

	pid_s pidCfg = {
			1,
			0,
			0, // P controller, easier to test
			0, // no offset
			-100,
			100, // min/max output
			0	 // alignment pad
	};

	bc.init(nullptr, nullptr, nullptr, &pidCfg);

	// Enable closed loop
	engineConfiguration->boostType = CLOSED_LOOP;
	// Minimum 75kpa
	engineConfiguration->minimumBoostClosedLoopMap = 75;

	// At 0 RPM, closed loop is disabled
	Sensor::setMockValue(SensorType::Rpm, 0);
	EXPECT_EQ(0, bc.getClosedLoop(150, 100).value_or(-1000));

	// Stopped engine, disable closed loop
	Sensor::setMockValue(SensorType::Rpm, 0);
	EXPECT_EQ(0, bc.getClosedLoop(150, 50).value_or(-1000));

	// With RPM, we should get an output
	Sensor::setMockValue(SensorType::Rpm, 1000);
	// Actual is below target -> positive output
	EXPECT_FLOAT_EQ(50, bc.getClosedLoop(150, 100).value_or(-1000));
	// MAP below target -> returns 0, closed loop disabled
	EXPECT_FLOAT_EQ(0, bc.getClosedLoop(150, 50).value_or(-1000));
	// Actual is above target -> negative output
	EXPECT_FLOAT_EQ(-25.0f, bc.getClosedLoop(150, 175).value_or(-1000));

	// Disabling closed loop should return 0
	engineConfiguration->boostType = OPEN_LOOP;
	EXPECT_FLOAT_EQ(0, bc.getClosedLoop(150, 175).value_or(-1000));
}

TEST(BoostControl, SetOutput) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isBoostControlEnabled = true;

	StrictMock<MockPwm> pwm;
	StrictMock<MockEtb> etb;
	BoostController bc;

	// ETB wastegate position & PWM should both be set
	EXPECT_CALL(etb, setWastegatePosition(25.0f));
	EXPECT_CALL(pwm, setSimplePwmDutyCycle(0.25f));

	// Don't crash if not init'd (don't deref null ptr m_pwm)
	EXPECT_NO_THROW(bc.setOutput(25.0f));

	// Init with mock PWM device and ETB
	bc.init(&pwm, nullptr, nullptr, nullptr);
	engine->etbControllers[0] = &etb;

	bc.setOutput(25.0f);
}

#endif // MODULE_BOOST
