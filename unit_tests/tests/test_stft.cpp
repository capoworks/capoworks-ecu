#include "pch.h"

#include "closed_loop_fuel_cell.h"
#include "closed_loop_fuel.h"

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class MockClCell : public ClosedLoopFuelCellBase {
public:
	MOCK_METHOD(float, getLambdaError, (), (const));
	MOCK_METHOD(float, getMaxAdjustment, (), (const));
	MOCK_METHOD(float, getMinAdjustment, (), (const));
	MOCK_METHOD(float, getIntegratorGain, (), (const));
};

TEST(ClosedLoopFuelCell, getLambdaError) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	ClosedLoopFuelCellImpl dut;
	stft_cell_cfg_s cellCfg;
	dut.configure(cellCfg, SensorType::Lambda1);

	// We are slightly lean of target
	Sensor::setMockValue(SensorType::Lambda1, 0.92f);
	engine->fuelComputer.targetLambda = 0.9f;

	// lean -> need to add fuel -> positive error
	EXPECT_NEAR(dut.getLambdaError(), 0.02222, 1e-4);

	// Dead lambda sensor -> zero lambda error
	Sensor::resetMockValue(SensorType::Lambda1);
	ASSERT_FALSE(Sensor::get(SensorType::Lambda1).Valid);
	EXPECT_EQ(dut.getLambdaError(), 0);
}

TEST(ClosedLoopFuelCell, AdjustRate) {
	StrictMock<MockClCell> cl;

	EXPECT_CALL(cl, getLambdaError()).WillOnce(Return(0.1f));
	EXPECT_CALL(cl, getMinAdjustment()).WillOnce(Return(-0.2f));
	EXPECT_CALL(cl, getMaxAdjustment()).WillOnce(Return(0.2f));
	EXPECT_CALL(cl, getIntegratorGain()).WillOnce(Return(2.0f));

	cl.update(false);

	// Should have integrated 0.2 * dt
	// dt = 1000.0f / FAST_CALLBACK_PERIOD_MS
	EXPECT_FLOAT_EQ(cl.getAdjustment(), 1 + (0.2f / (1000.0f / FAST_CALLBACK_PERIOD_MS)));
}

TEST(ClosedLoopFuel, CellSelection) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	struct : public MockIdleController {
		bool isIdling = false;

		bool isIdlingOrTaper() const override {
			return isIdling;
		}
	} idleController;

	engine->engineModules.get<IdleController>().set(&idleController);

	stft_s cfg;

	// Sensible region config
	cfg.minPowerLoad = 80;
	cfg.maxOverrunLoad = 30;

	// Test idle
	idleController.isIdling = true;
	EXPECT_EQ(0, computeStftBin(10, cfg));
	EXPECT_EQ(0, computeStftBin(50, cfg));
	EXPECT_EQ(0, computeStftBin(90, cfg));
	idleController.isIdling = false;

	// Test overrun
	EXPECT_EQ(1, computeStftBin(10, cfg));

	// Test load
	EXPECT_EQ(2, computeStftBin(90, cfg));

	// Main cell
	EXPECT_EQ(3, computeStftBin(50, cfg));
}

TEST(ClosedLoopFuel, ExplicitTargetAndLoadDoNotUsePublishedState) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	struct : public MockIdleController {
		bool isIdlingOrTaper() const override {
			return false;
		}
	} idleController;
	engine->engineModules.get<IdleController>().set(&idleController);

	// Transition to running while correction is still disabled.
	engine->rpmCalculator.setRpmValue(2000);

	engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	engineConfiguration->stft.startupDelay = 0;
	engineConfiguration->stft.minClt = 60;
	engineConfiguration->stft.minLambda = 0.7f;
	engineConfiguration->stft.maxLambda = 1.2f;
	engineConfiguration->noFuelTrimAfterDfcoTime = 0;
	engineConfiguration->stftIgnoreErrorMagnitude = false;

	for (auto& cellCfg : engineConfiguration->stft.cellCfgs) {
		cellCfg.timeConstant = 1;
		cellCfg.maxAdd = 0;
		cellCfg.maxRemove = 0;
	}
	// Only the power cell may remove fuel, making the selected load observable.
	engineConfiguration->stft.cellCfgs[2].maxRemove = -25;

	Sensor::setMockValue(SensorType::Lambda1, 0.9f);
	eth.moveTimeForwardSec(3);
	engine->fuelComputer.running.timeSinceCrankingInSecs = 10;

	// These are the last published values. The current calculation is 90 load at lambda 1.0.
	constexpr float publishedLoad = 10;
	constexpr float publishedTarget = 0.8f;
	engine->engineState.fuelingLoad = publishedLoad;
	engine->fuelComputer.targetLambda = publishedTarget;

	auto result = fuelClosedLoopCorrection(90, 1.0f);

	// Lambda 0.9 is rich against the current target 1.0, so the power cell removes fuel.
	EXPECT_LT(result.banks[0], 1.0f);
	EXPECT_FLOAT_EQ(publishedLoad, engine->engineState.fuelingLoad);
	EXPECT_FLOAT_EQ(publishedTarget, engine->fuelComputer.targetLambda);
}

TEST(ClosedLoopFuel, lambdaLimits) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->stft.minLambda = 0.7f;
	engineConfiguration->stft.maxLambda = 1.2f;

	// Lower bound
	Sensor::setMockValue(SensorType::Lambda1, 0.65f);
	EXPECT_FALSE(shouldUpdateCorrection(SensorType::Lambda1));
	Sensor::setMockValue(SensorType::Lambda1, 0.75f);
	EXPECT_TRUE(shouldUpdateCorrection(SensorType::Lambda1));

	Sensor::setMockValue(SensorType::Lambda1, 1.15f);
	EXPECT_TRUE(shouldUpdateCorrection(SensorType::Lambda1));
	Sensor::setMockValue(SensorType::Lambda1, 1.25f);
	EXPECT_FALSE(shouldUpdateCorrection(SensorType::Lambda1));
}

TEST(ClosedLoopFuel, deadLambdaSensorWithZeroMinLambda) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	// With min lambda 0, a dead sensor reading as 0 would pass the range check
	engineConfiguration->stft.minLambda = 0;
	engineConfiguration->stft.maxLambda = 1.2f;

	// Working sensor -> correct
	Sensor::setMockValue(SensorType::Lambda1, 1.0f);
	EXPECT_TRUE(shouldUpdateCorrection(SensorType::Lambda1));

	// Dead sensor -> don't correct
	Sensor::setInvalidMockValue(SensorType::Lambda1);
	ASSERT_FALSE(Sensor::get(SensorType::Lambda1).Valid);
	EXPECT_FALSE(shouldUpdateCorrection(SensorType::Lambda1));

	// Missing sensor -> don't correct
	Sensor::resetMockValue(SensorType::Lambda1);
	ASSERT_FALSE(Sensor::get(SensorType::Lambda1).Valid);
	EXPECT_FALSE(shouldUpdateCorrection(SensorType::Lambda1));
}
