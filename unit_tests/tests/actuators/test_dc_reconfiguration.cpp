#include "pch.h"

#include "dc_motor.h"
#include "electronic_throttle_impl.h"

namespace {

struct RecordingPwm : IPwm {
	float duty = 0;
	unsigned immediateWrites = 0;

	void setSimplePwmDutyCycle(float value) override {
		duty = value;
	}

	void setDutyImmediate(float value) override {
		++immediateWrites;
		duty = value;
	}
};

struct MotorHardware {
	OutputPin disable;
	TwoPinDcMotor motor{disable};
	RecordingPwm enable, dir1, dir2;

	MotorHardware() {
		motor.configure(enable, dir1, dir2, false);
		motor.setType(TwoPinDcMotor::ControlType::PwmEnablePin);
	}

	void expectStopped() const {
		EXPECT_TRUE(disable.getLogicValue());
		EXPECT_EQ(motor.get(), 0);
		EXPECT_EQ(enable.duty, 0);
		EXPECT_EQ(dir1.duty, 0);
		EXPECT_EQ(dir2.duty, 0);
	}
};

pid_s makePid() {
	pid_s pid = {};
	pid.pFactor = 1;
	pid.minValue = -90;
	pid.maxValue = 90;
	return pid;
}

} // namespace

TEST(DcReconfiguration, StopImmediatelyDoesNotWaitForPwmTimers) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (auto type : {TwoPinDcMotor::ControlType::PwmDirectionPins, TwoPinDcMotor::ControlType::PwmEnablePin}) {
		for (bool inverted : {false, true}) {
			for (float batteryVoltage : {14.0f, 0.0f}) {
				MotorHardware hw;
				hw.motor.setType(type);
				hw.motor.configure(hw.enable, hw.dir1, hw.dir2, inverted);
				hw.motor.set(0.6f);
				hw.motor.enable();
				ASSERT_FALSE(hw.disable.getLogicValue());

				// Advance no timer: the terminal outputs must already be safe.
				Sensor::setMockValue(SensorType::BatteryVoltage, batteryVoltage);
				hw.motor.stopImmediately();

				EXPECT_EQ(hw.enable.duty, 0);
				EXPECT_EQ(hw.dir1.duty, inverted ? 1 : 0);
				EXPECT_EQ(hw.dir2.duty, inverted ? 1 : 0);
				EXPECT_EQ(hw.enable.immediateWrites, 1u);
				EXPECT_EQ(hw.dir1.immediateWrites, 1u);
				EXPECT_EQ(hw.dir2.immediateWrites, 1u);
				EXPECT_EQ(hw.motor.get(), 0);
				EXPECT_TRUE(hw.disable.getLogicValue());

				// Releasing the disable GPIO cannot restore the old command.
				hw.motor.enable();
				EXPECT_EQ(hw.enable.duty, 0);
				Sensor::setMockValue(SensorType::BatteryVoltage, 14);
			}
		}
	}
}

TEST(DcReconfiguration, DirectionIsProgrammedBeforeGateOnResume) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	testing::StrictMock<MockOutputPin> disable;
	EXPECT_CALL(disable, setValue(true)).Times(2);
	TwoPinDcMotor motor(disable);
	testing::StrictMock<MockPwm> enable, dir1, dir2;
	motor.configure(enable, dir1, dir2, false);

	::testing::InSequence sequence;
	EXPECT_CALL(disable, setValue(true));
	EXPECT_CALL(enable, setSimplePwmDutyCycle(0));
	EXPECT_CALL(dir1, setSimplePwmDutyCycle(0));
	EXPECT_CALL(dir2, setSimplePwmDutyCycle(0));
	motor.stopImmediately();

	EXPECT_CALL(dir1, setSimplePwmDutyCycle(0));
	EXPECT_CALL(dir2, setSimplePwmDutyCycle(0.4f));
	EXPECT_CALL(enable, setSimplePwmDutyCycle(1));
	EXPECT_CALL(disable, setValue(false));
	motor.set(-0.4f);
	motor.enable();
}

TEST(DcReconfiguration, DisabledSlotStopsAndDetachesController) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MotorHardware hw;
	auto pid = makePid();
	EtbController controller;
	ASSERT_TRUE(controller.init(DC_Wastegate, &hw.motor, &pid, nullptr, false));
	controller.setOutput(50);
	ASSERT_GT(hw.enable.duty, 0);

	engine->etbControllers[0] = &controller;
	engine->etbControllers[1] = nullptr;
	engineConfiguration->etbFunctions[0] = DC_None;
	engineConfiguration->etbFunctions[1] = DC_None;
	doInitElectronicThrottle();

	hw.expectStopped();
	controller.setOutput(50);
	controller.update();
	hw.expectStopped();
}

TEST(DcReconfiguration, DirectNoneInitializationStopsPreviousMotor) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MotorHardware hw;
	auto pid = makePid();
	EtbController controller;
	ASSERT_TRUE(controller.init(DC_Wastegate, &hw.motor, &pid, nullptr, false));
	controller.setOutput(50);

	EXPECT_FALSE(controller.init(DC_None, &hw.motor, &pid, nullptr, false));
	controller.setOutput(50);
	hw.expectStopped();
}

enum class InitFailure {
	MissingPedal,
	MissingPrimaryTps,
	NonredundantTps,
	NonredundantPedal
};

class FailedDcReconfiguration : public testing::TestWithParam<InitFailure> {};

TEST_P(FailedDcReconfiguration, StopsOldMotorAndDetachesNewBinding) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MotorHardware oldHardware, newHardware;
	auto pid = makePid();
	EtbController controller;
	ASSERT_TRUE(controller.init(DC_Wastegate, &oldHardware.motor, &pid, nullptr, false));
	controller.setOutput(50);

	Sensor::setMockValue(SensorType::Tps1Primary, 10);
	Sensor::setMockValue(SensorType::Tps1, 10, GetParam() != InitFailure::NonredundantTps);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 30, GetParam() != InitFailure::NonredundantPedal);
	if (GetParam() == InitFailure::MissingPrimaryTps) {
		Sensor::resetMockValue(SensorType::Tps1Primary);
	}

	if (GetParam() == InitFailure::NonredundantTps || GetParam() == InitFailure::NonredundantPedal) {
		EXPECT_FATAL_ERROR(controller.init(DC_Throttle1, &newHardware.motor, &pid, nullptr, true));
	} else {
		ASSERT_FALSE(controller.init(
				DC_Throttle1, &newHardware.motor, &pid, nullptr, GetParam() != InitFailure::MissingPedal));
	}

	oldHardware.expectStopped();
	controller.setOutput(50);
	controller.update();
	oldHardware.expectStopped();
	EXPECT_EQ(newHardware.enable.immediateWrites, 0u);
	EXPECT_EQ(newHardware.enable.duty, 0);
	EXPECT_FLOAT_EQ(controller.m_outputDuty, 0);
}

INSTANTIATE_TEST_SUITE_P(
		ValidationFailures,
		FailedDcReconfiguration,
		testing::Values(
				InitFailure::MissingPedal,
				InitFailure::MissingPrimaryTps,
				InitFailure::NonredundantTps,
				InitFailure::NonredundantPedal));

TEST(DcReconfiguration, ValidReinitializationBindsOnlyNewMotor) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MotorHardware oldHardware, newHardware, unaffectedHardware;
	auto pid = makePid();
	EtbController controller, unaffected;
	ASSERT_TRUE(controller.init(DC_Wastegate, &oldHardware.motor, &pid, nullptr, false));
	ASSERT_TRUE(unaffected.init(DC_Wastegate, &unaffectedHardware.motor, &pid, nullptr, false));
	controller.setOutput(50);
	unaffected.setOutput(30);

	ASSERT_TRUE(controller.init(DC_Wastegate, &newHardware.motor, &pid, nullptr, false));
	oldHardware.expectStopped();
	EXPECT_FLOAT_EQ(controller.m_outputDuty, 0);
	EXPECT_FLOAT_EQ(unaffectedHardware.motor.get(), 0.3f);
	EXPECT_EQ(unaffectedHardware.enable.immediateWrites, 0u);
	EXPECT_FALSE(unaffectedHardware.disable.getLogicValue());

	controller.setOutput(-25);
	EXPECT_FLOAT_EQ(newHardware.motor.get(), -0.25f);
	EXPECT_FLOAT_EQ(newHardware.enable.duty, 0.25f);
	EXPECT_FALSE(newHardware.disable.getLogicValue());
	oldHardware.expectStopped();
}

TEST(DcReconfiguration, ReinitializationAbortsAutocalibration) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MotorHardware oldHardware, newHardware;
	auto pid = makePid();
	struct PedalMap : ValueProvider3D {
		float getValue(float, float pedal) const override {
			return pedal;
		}
	} map;
	Sensor::setMockValue(SensorType::Tps1Primary, 10);
	Sensor::setMockValue(SensorType::Tps1, 10, true);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 30, true);
	initElectronicThrottle();
	auto controller = engine->etbControllers[0];
	ASSERT_NE(controller, nullptr);
	ASSERT_TRUE(controller->init(DC_Throttle1, &oldHardware.motor, &pid, &map, true));
	controller->autoCalibrateTps();
	controller->update();
	ASSERT_FLOAT_EQ(oldHardware.motor.get(), 0.5f);

	// A reconfiguration must not continue a sequence started on the old motor.
	ASSERT_TRUE(controller->init(DC_Throttle1, &newHardware.motor, &pid, &map, true));
	oldHardware.expectStopped();
	controller->update();
	EXPECT_NE(newHardware.motor.get(), 0.5f);
	EXPECT_GT(newHardware.enable.duty, 0);
	controller->deinit();
}
