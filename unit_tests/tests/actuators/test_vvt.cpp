#include "pch.h"

#include "airmass_loads.h"
#include "vvt.h"

using ::testing::Return;
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

TEST(VVT, IntakeAndExhaustSourcesStayIndependent) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setMockValue(SensorType::Rpm, 2000);
	Sensor::setMockValue(SensorType::Map, 80);
	Sensor::setMockValue(SensorType::Tps1, 20);
	engineConfiguration->vvtIntakeYAxisOverride = GPPWM_Tps;
	engineConfiguration->vvtExhaustYAxisOverride = GPPWM_Map;
	StrictMock<MockVp3d> intakeMap;
	StrictMock<MockVp3d> exhaustMap;
	EXPECT_CALL(intakeMap, getValue(2000, 20)).WillOnce(Return(10));
	EXPECT_CALL(exhaustMap, getValue(2000, 80)).WillRepeatedly(Return(30));
	VvtController intake(0, 0, 0);
	VvtController exhaust(1, 0, 1);
	intake.init(&intakeMap, nullptr);
	exhaust.init(&exhaustMap, nullptr);
	EXPECT_FLOAT_EQ(intake.getSetpoint().value_or(0), 10);
	EXPECT_FLOAT_EQ(exhaust.getSetpoint().value_or(0), 30);

	Sensor::setInvalidMockValue(SensorType::Tps1);
	EXPECT_FALSE(intake.getSetpoint().Valid);
	EXPECT_FLOAT_EQ(exhaust.getSetpoint().value_or(0), 30);
}

TEST(VVT, DefaultAxisUsesPhysicalSnapshotAndStalePublicationDisables) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setMockValue(SensorType::Rpm, 2000);
	engine->engineState.fuelingLoad = 11.25f;
	engineConfiguration->vvtIntakeYAxisOverride = GPPWM_Zero;
	StrictMock<MockVp3d> targetMap;
	EXPECT_CALL(targetMap, getValue(2000, 71.25f)).WillOnce(Return(10));
	VvtController dut(0, 0, 0);
	dut.init(&targetMap, nullptr);

	ASSERT_TRUE(publishPhysicalFuelLoad());
	EXPECT_FLOAT_EQ(dut.getSetpoint().value_or(0), 10);
	EXPECT_FLOAT_EQ(dut.targetYAxis, 71.3f);

	invalidateAirmassConsumerLoads();
	EXPECT_FALSE(dut.getSetpoint().Valid);
	EXPECT_FLOAT_EQ(dut.targetYAxis, 0);
}

TEST(VVT, Setpoint) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	// Set up a mock target map
	StrictMock<MockVp3d> targetMap;
	EXPECT_CALL(targetMap, getValue(4321, 55)).WillRepeatedly(Return(20));

	// Mock necessary inputs
	engine->engineState.fuelingLoad = 55;
	Sensor::setMockValue(SensorType::Rpm, 4321);

	VvtController dut(0, 0, 0);
	dut.init(&targetMap, nullptr);

	setTimeNowUs(0);

	// Test dut
	EXPECT_EQ(20, dut.getSetpoint().value_or(0));

	// Apply position bump
	dut.setTargetOffset(10);
	EXPECT_EQ(20 + 10, dut.getSetpoint().value_or(0));

	// 1.9 seconds: still bumped
	setTimeNowUs(1.9e6);
	EXPECT_EQ(20 + 10, dut.getSetpoint().value_or(0));

	// 2.1 seconds: things go back to normal
	setTimeNowUs(2.1e6);
	EXPECT_EQ(20, dut.getSetpoint().value_or(0));
}

struct FakeMap : public ValueProvider3D {
	float setpoint;

	float getValue(float xColumn, float yRow) const override {
		return setpoint;
	}
};

TEST(VVT, SetpointHysteresisAdvancingCam) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	FakeMap targetMap;

	VvtController dut(0, 0, 0);
	dut.init(&targetMap, nullptr);

	// 0 position returns unexpected
	targetMap.setpoint = 0;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));

	// Between hysteresis still return unexpected
	targetMap.setpoint = 2;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));

	// Above hysteresis returns real value
	targetMap.setpoint = 5;
	EXPECT_EQ(5, dut.getSetpoint().value_or(-1000));

	// Between hysteresis still returns valid
	targetMap.setpoint = 2;
	EXPECT_EQ(2, dut.getSetpoint().value_or(-1000));

	// Back under hysteresis retuns unexpected again
	targetMap.setpoint = 0.5;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));
}

TEST(VVT, SetpointHysteresisRetardingCam) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->invertVvtControlIntake = true;

	FakeMap targetMap;

	VvtController dut(0, 0, 0);
	dut.init(&targetMap, nullptr);

	// 0 position returns unexpected
	targetMap.setpoint = 0;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));

	// Between hysteresis still return unexpected
	targetMap.setpoint = -2;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));

	// Above hysteresis returns real value
	targetMap.setpoint = -5;
	EXPECT_EQ(-5, dut.getSetpoint().value_or(-1000));

	// Between hysteresis still returns valid
	targetMap.setpoint = -2;
	EXPECT_EQ(-2, dut.getSetpoint().value_or(-1000));

	// Back under hysteresis retuns unexpected again
	targetMap.setpoint = -0.5;
	EXPECT_EQ(-1000, dut.getSetpoint().value_or(-1000));
}

TEST(VVT, ObservePlant) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engine->triggerCentral.vvtPosition[0][0].t.reset();
	engine->triggerCentral.vvtPosition[0][0].angle = 23;

	VvtController dut(0, 0, 0);
	dut.init(nullptr, nullptr);

	EXPECT_EQ(23, dut.observePlant().value_or(0));
}

TEST(VVT, OpenLoop) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	copyArray(config->vvtOpenLoop[0].bins, {-20, 0, 20, 40, 60, 80, 100, 120});
	copyArray(config->vvtOpenLoop[0].values, {0, 10, 20, 30, 40, 50, 60, 70});

	VvtController dut(0, 0, 0);

	// Check first with oil temperature
	Sensor::resetMockValue(SensorType::Clt);
	Sensor::setMockValue(SensorType::OilTemperature, 20);
	EXPECT_EQ(dut.getOpenLoop(0), 20);
	Sensor::setMockValue(SensorType::OilTemperature, 100);
	EXPECT_EQ(dut.getOpenLoop(0), 60);

	// Both sensors dead, expect 80C value
	Sensor::resetMockValue(SensorType::Clt);
	Sensor::resetMockValue(SensorType::OilTemperature);

	// Check coolant temp
	Sensor::setMockValue(SensorType::Clt, 20);
	EXPECT_EQ(dut.getOpenLoop(0), 20);
	Sensor::setMockValue(SensorType::Clt, 100);
	EXPECT_EQ(dut.getOpenLoop(0), 60);

	// In case you have both, it should us oil T
	Sensor::setMockValue(SensorType::OilTemperature, 40);
	Sensor::setMockValue(SensorType::Clt, 60);
	EXPECT_EQ(dut.getOpenLoop(0), 30);
}

TEST(VVT, ClosedLoopNotInverted) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	VvtController dut(0, 0, 0);
	dut.init(nullptr, nullptr);

	engineConfiguration->auxPid[0].pFactor = 1.5f;
	engineConfiguration->auxPid[0].iFactor = 0;
	engineConfiguration->auxPid[0].dFactor = 0;
	engineConfiguration->auxPid[0].offset = 0;

	// Target of 30 with position 20 should yield positive duty, P=1.5 means 15% duty for 10% error
	EXPECT_EQ(dut.getClosedLoop(30, 20).value_or(0), 15);
}

TEST(VVT, ClosedLoopInverted) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	VvtController dut(0, 0, 0);
	dut.init(nullptr, nullptr);

	engineConfiguration->invertVvtControlIntake = true;
	engineConfiguration->auxPid[0].pFactor = 1.5f;
	engineConfiguration->auxPid[0].iFactor = 0;
	engineConfiguration->auxPid[0].dFactor = 0;
	engineConfiguration->auxPid[0].offset = 0;

	// Target of -30 with position -20 should yield positive duty, P=1.5 means 15% duty for 10% error
	EXPECT_EQ(dut.getClosedLoop(-30, -20).value_or(0), 15);
}
