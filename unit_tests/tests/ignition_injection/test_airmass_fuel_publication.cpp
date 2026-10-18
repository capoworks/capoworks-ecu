#include "pch.h"

#include "airmass_calibration.h"
#include "fuel_math.h"

namespace {
using Fault = AirmassInjectionFault;
using Status = AirmassInjectionStatus;

void configureSpeedDensityFuel(EngineTestHelper& eth) {
	setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth);
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);

	engine->engineState.sd.tChargeK = 293.15f;
	Sensor::setMockValue(SensorType::Rpm, 1800);
	Sensor::setMockValue(SensorType::Map, 55);
	Sensor::setMockValue(SensorType::Tps1, 20);

	setLinearCurve(config->veLoadBins, 0, 140, 1);
	setLinearCurve(config->veRpmBins, 500, 8000, 1);
	setTable(config->veTable, 60);
}

void runFuelCalculation(EngineTestHelper& eth) {
	eth.engine.periodicFastCallback();
}

void expectAllActiveCylinderMasses(float expected) {
	for (size_t i = 0; i < engine->engineState.cylinderCount; i++) {
		EXPECT_FLOAT_EQ(expected, engine->cylinders[i].getInjectionMass()) << i;
	}
}

void expectFuelOutputsZero() {
	EXPECT_FALSE(engine->engineState.airmassCalculationValid);
	EXPECT_FLOAT_EQ(0, engine->engineState.baseFuel);
	EXPECT_FLOAT_EQ(0, engine->engineState.tpsAccelEnrich);
	EXPECT_FLOAT_EQ(0, engine->engineState.injectionStage2Fraction);
	EXPECT_FLOAT_EQ(0, engine->engineState.injectionDuration);
	EXPECT_FLOAT_EQ(0, engine->engineState.injectionDurationStage2);
	EXPECT_FLOAT_EQ(0, engine->engineState.injectionOffset);
	EXPECT_FLOAT_EQ(0, static_cast<float>(engine->fuelComputer.running.baseFuel));
	EXPECT_FLOAT_EQ(0, static_cast<float>(engine->fuelComputer.running.fuel));
	expectAllActiveCylinderMasses(0);
}
} // namespace

TEST(AirmassFuelPublication, CompletedPublicationSurvivesPositiveRpmReplacementStart) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);

	runFuelCalculation(eth);
	ASSERT_TRUE(engine->engineState.airmassCalculationValid);
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
	const float publishedMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(publishedMass, 0);

	invalidateAirmassCalibration();
	bool admittedDuringEvaluation = false;
	float massDuringEvaluation = 0;
	int scanCallbacks = 0;
	engine->onAirmassCalibrationScanned = [&] {
		if (scanCallbacks++ != 0) {
			return;
		}
		engine->airmassInjectionState.beginCalculation(LM_SPEED_DENSITY, 1800, engine->getGlobalConfigurationVersion());
		admittedDuringEvaluation = engine->airmassInjectionState.allowInjection();
		massDuringEvaluation = engine->cylinders[0].getInjectionMass();
	};

	runFuelCalculation(eth);

	EXPECT_GE(scanCallbacks, 1);
	EXPECT_TRUE(admittedDuringEvaluation) << "replacement work must not close the completed publication";
	EXPECT_FLOAT_EQ(publishedMass, massDuringEvaluation);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_FALSE(engine->engineState.airmassCalculationValid) << "in-progress work is not a completed publication";
	EXPECT_FLOAT_EQ(publishedMass, engine->cylinders[0].getInjectionMass())
			<< "the stale evaluation must not alter the newer publication";

	engine->onAirmassCalibrationScanned = nullptr;
	setTable(config->veTable, 70);
	runFuelCalculation(eth);

	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_NE(publishedMass, engine->cylinders[0].getInjectionMass());
}

TEST(AirmassFuelPublication, TuneGenerationChangeCannotReopenStaleResult) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);

	runFuelCalculation(eth);
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
	const float publishedMass = engine->cylinders[0].getInjectionMass();

	invalidateAirmassCalibration();
	bool tuneInvalidated = false;
	engine->onAirmassCalibrationScanned = [&] {
		if (tuneInvalidated) {
			return;
		}
		tuneInvalidated = true;
		++engine->globalConfigurationVersion;
		engine->airmassInjectionState.onConfigurationWrite(LM_SPEED_DENSITY, false);
	};

	runFuelCalculation(eth);

	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	EXPECT_FLOAT_EQ(publishedMass, engine->cylinders[0].getInjectionMass())
			<< "a stale result must not replace the last completed publication";

	engine->onAirmassCalibrationScanned = nullptr;
	runFuelCalculation(eth);

	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_EQ(Status::Legacy, engine->airmassInjectionState.status());
	EXPECT_EQ(Fault::None, engine->airmassInjectionState.fault());
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
}

TEST(AirmassFuelPublication, CurrentCorrectionFailureClearsEverythingAndRecovers) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);

	runFuelCalculation(eth);
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
	ASSERT_GT(engine->cylinders[0].getInjectionMass(), 0);

	engineConfiguration->globalFuelCorrection = NAN;
	runFuelCalculation(eth);

	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	EXPECT_EQ(Status::Faulted, engine->airmassInjectionState.status());
	EXPECT_EQ(Fault::Result, engine->airmassInjectionState.fault());
	expectFuelOutputsZero();

	engineConfiguration->globalFuelCorrection = 1;
	runFuelCalculation(eth);

	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
}

TEST(AirmassFuelPublication, InvalidCylinderBankClearsEverythingAndRecovers) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);

	runFuelCalculation(eth);
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());

	const uint8_t originalBank = engineConfiguration->cylinderBankSelect[2];
	engineConfiguration->cylinderBankSelect[2] = STFT_BANK_COUNT;
	runFuelCalculation(eth);

	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	expectFuelOutputsZero();

	engineConfiguration->cylinderBankSelect[2] = originalBank;
	runFuelCalculation(eth);

	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_GT(engine->cylinders[2].getInjectionMass(), 0);
}

TEST(AirmassFuelPublication, LegacyMockAndLuaAdjustmentsRemainNumericallyCompatible) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth);
	engineConfiguration->fuelAlgorithm = LM_MOCK;
	engineConfiguration->postCrankingFactor = 1.0f;
	engine->engineState.lua.fuelMult = 1.2f;
	engine->engineState.lua.fuelAdd = 0.03f;

	EXPECT_CALL(*eth.mockAirmass, getAirmass(testing::_, testing::_))
			.WillRepeatedly(testing::Return(AirmassResult{1.3440001f, 50.0f}));

	Sensor::setMockValue(SensorType::Rpm, 2000);
	engine->fuelComputer.running.intakeTemperatureCoefficient = 1.0f;
	engine->fuelComputer.running.coolantTemperatureCoefficient = 1.0f;
	engine->fuelComputer.running.postCrankingFuelCorrection = 1.0f;
	engine->engineState.baroCorrection = 1.0f;
	const auto epochBeforeDirectMass = engine->airmassInjectionState.publicationEpoch();
	const bool injectionReadyBeforeDirectMass = engine->airmassInjectionState.allowInjection();
	const float rawCycleMass = getCycleInjectionMass(2000, false);
	ASSERT_GT(rawCycleMass, 0);
	const float expectedCycleMass = rawCycleMass * engine->engineState.lua.fuelMult + engine->engineState.lua.fuelAdd;
	EXPECT_TRUE(injectionReadyBeforeDirectMass);
	EXPECT_EQ(epochBeforeDirectMass, engine->airmassInjectionState.publicationEpoch());
	EXPECT_EQ(injectionReadyBeforeDirectMass, engine->airmassInjectionState.allowInjection());

	runFuelCalculation(eth);

	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_EQ(Status::Legacy, engine->airmassInjectionState.status());
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_NEAR(expectedCycleMass, engine->cylinders[0].getInjectionMass(), EPS4D);
	EXPECT_GE(engine->engineState.injectionDuration, 0);
}
