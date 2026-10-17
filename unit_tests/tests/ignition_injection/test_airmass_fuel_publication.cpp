#include "pch.h"

#include "airmass_calibration.h"
#include "fuel_math.h"
#include "lua_airmass.h"
#include "lua_hooks.h"

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

void configureLegacyConsumerTables() {
	setLinearCurve(config->injPhaseLoadBins, 0, 100, 1);
	setLinearCurve(config->injPhaseRpmBins, 0, 8000, 1);
	setLinearCurve(config->injectorStagingLoadBins, 0, 100, 1);
	setLinearCurve(config->injectorStagingRpmBins, 0, 8000, 1);
	setLinearCurve(config->trailingIgnitionLoadBins, 0, 100, 1);
	setLinearCurve(config->trailingIgnitionRpmBins, 0, 8000, 1);
	setLinearCurve(config->ignitionIatCorrLoadBins, 0, 100, 1);
	setLinearCurve(config->ignitionIatCorrTempBins, 0, 100, 1);
	setLinearCurve(config->ignTrimLoadBins, 0, 100, 1);
	setLinearCurve(config->ignTrimRpmBins, 0, 8000, 1);

	setTable(config->ignitionTable, 10);
	setArrayValues(config->cltTimingExtra, 0);
	for (size_t row = 0; row < efi::size(config->injectionPhase); row++) {
		setArrayValues(config->injectionPhase[row], config->injPhaseLoadBins[row]);
	}
	for (size_t row = 0; row < efi::size(config->injectorStagingTable); row++) {
		setArrayValues(config->injectorStagingTable[row], config->injectorStagingLoadBins[row]);
	}
	for (size_t row = 0; row < efi::size(config->trailingIgnitionTable); row++) {
		setArrayValues(config->trailingIgnitionTable[row], config->trailingIgnitionLoadBins[row] / 10.0f);
	}
	for (size_t row = 0; row < efi::size(config->ignitionIatCorrTable); row++) {
		setArrayValues(config->ignitionIatCorrTable[row], config->ignitionIatCorrLoadBins[row] / 10.0f);
	}
}

struct NeverIdleController : public MockIdleController {
	bool isIdlingOrTaper() const override {
		return false;
	}
};

void configureLegacyConsumerRun(EngineTestHelper& eth, engine_load_mode_e mode, MockIdleController& idleController) {
	setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth);
	setCylinderCount(4);
	engineConfiguration->fuelAlgorithm = mode;
	engineConfiguration->postCrankingFactor = 1.0f;
	engineConfiguration->isInjectionEnabled = true;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->timingMode = TM_DYNAMIC;
	engineConfiguration->enableStagedInjection = true;
	engineConfiguration->enableTrailingSparks = true;
	engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	engineConfiguration->stft.startupDelay = 0;
	engineConfiguration->stft.minClt = 60;
	engineConfiguration->stft.minLambda = 0.7f;
	engineConfiguration->stft.maxLambda = 1.2f;
	engineConfiguration->stft.maxOverrunLoad = 40;
	engineConfiguration->stft.minPowerLoad = 70;
	engineConfiguration->noFuelTrimAfterDfcoTime = 0;
	engineConfiguration->lambdaProtectionEnable = false;
	configureLegacyConsumerTables();

	Sensor::setMockValue(SensorType::Rpm, 2000);
	Sensor::setMockValue(SensorType::Map, 80);
	Sensor::setMockValue(SensorType::Tps1, 20);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 30);
	Sensor::setMockValue(SensorType::Iat, 20);
	Sensor::setMockValue(SensorType::Clt, 80);
	Sensor::setMockValue(SensorType::Lambda1, 0.9f);
	engine->rpmCalculator.setRpmValue(1800);
	engine->fuelComputer.running.timeSinceCrankingInSecs = 10;
	engine->fuelComputer.running.intakeTemperatureCoefficient = 1.0f;
	engine->fuelComputer.running.coolantTemperatureCoefficient = 1.0f;
	engine->fuelComputer.running.postCrankingFuelCorrection = 1.0f;
	engine->engineState.baroCorrection = 1.0f;
	engine->engineModules.get<IdleController>().set(&idleController);

	const AirmassResult airmass{1.3440001f, 50.0f};
	if (mode == LM_LUA) {
		static_cast<LuaAirmass&>(getLuaAirmassModel()).setAirmass(airmass);
	} else {
		EXPECT_CALL(*eth.mockAirmass, getAirmass(testing::_, testing::_)).WillRepeatedly(testing::Return(airmass));
	}
}

void expectLegacyCustomConsumerAxes() {
	EXPECT_NEAR(20, engine->engineState.injectionOffset, 0.1f);
	EXPECT_NEAR(0.8f, engine->engineState.injectionStage2Fraction, 0.01f);
	EXPECT_NEAR(2, engine->engineState.trailingSparkAngle, 0.1f);
	EXPECT_NEAR(8, engine->ignitionState.timingIatCorrection, 0.1f);
	EXPECT_EQ(1, engine->outputChannels.fuelClosedLoopBinIdx);
	EXPECT_NEAR(15, engine->cylinders[1].getIgnitionTimingBtdc() - engine->cylinders[0].getIgnitionTimingBtdc(), 0.1f);
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

TEST(AirmassFuelPublication, LegacyMockSelectorsUseLiveConsumerAxes) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	NeverIdleController idleController;
	configureLegacyConsumerRun(eth, LM_MOCK, idleController);
	config->injectionPhaseLoadSource = AFR_Tps;
	config->stagingLoadSource = AFR_MAP;
	config->trailingSparkLoadSource = AFR_Tps;
	config->ignitionIatLoadSource = AFR_MAP;
	config->stftLoadSource = AFR_AccPedal;
	config->ignitionTrimLoadSource[0] = AFR_Tps;
	config->ignitionTrimLoadSource[1] = AFR_MAP;
	for (size_t row = 0; row < efi::size(config->ignTrims[0].table); row++) {
		setArrayValues(config->ignTrims[0].table[row], config->ignTrimLoadBins[row] / 4.0f);
		setArrayValues(config->ignTrims[1].table[row], config->ignTrimLoadBins[row] / 4.0f);
	}

	runFuelCalculation(eth);

	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	expectLegacyCustomConsumerAxes();
}

TEST(AirmassFuelPublication, LegacyLuaSelectorsUseLiveConsumerAxes) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	NeverIdleController idleController;
	configureLegacyConsumerRun(eth, LM_LUA, idleController);
	config->injectionPhaseLoadSource = AFR_Tps;
	config->stagingLoadSource = AFR_MAP;
	config->trailingSparkLoadSource = AFR_Tps;
	config->ignitionIatLoadSource = AFR_MAP;
	config->stftLoadSource = AFR_AccPedal;
	config->ignitionTrimLoadSource[0] = AFR_Tps;
	config->ignitionTrimLoadSource[1] = AFR_MAP;
	for (size_t row = 0; row < efi::size(config->ignTrims[0].table); row++) {
		setArrayValues(config->ignTrims[0].table[row], config->ignTrimLoadBins[row] / 4.0f);
		setArrayValues(config->ignTrims[1].table[row], config->ignTrimLoadBins[row] / 4.0f);
	}

	runFuelCalculation(eth);

	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	expectLegacyCustomConsumerAxes();
}

TEST(AirmassFuelPublication, LegacyDefaultSelectorsKeepHistoricalConsumerCoordinates) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	NeverIdleController idleController;
	configureLegacyConsumerRun(eth, LM_MOCK, idleController);
	engineConfiguration->afrOverrideMode = AFR_Tps;
	engineConfiguration->ignOverrideMode = AFR_MAP;
	config->injectionPhaseLoadSource = AFR_None;
	config->stagingLoadSource = AFR_None;
	config->trailingSparkLoadSource = AFR_None;
	config->ignitionIatLoadSource = AFR_None;
	config->stftLoadSource = AFR_None;
	for (auto& source : config->ignitionTrimLoadSource) {
		source = AFR_None;
	}
	config->ignitionTrimLoadSource[3] = AFR_Tps;
	for (size_t row = 0; row < efi::size(config->ignTrims[0].table); row++) {
		for (auto& trim : config->ignTrims) {
			setArrayValues(trim.table[row], config->ignTrimLoadBins[row] / 10.0f);
		}
	}

	runFuelCalculation(eth);

	ASSERT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_FLOAT_EQ(50, engine->engineState.fuelingLoad);
	EXPECT_FLOAT_EQ(80, engine->engineState.ignitionLoad);
	EXPECT_FLOAT_EQ(20, engine->fuelComputer.afrTableYAxis);
	EXPECT_NEAR(50, engine->engineState.injectionOffset, 0.1f);
	EXPECT_NEAR(0.2f, engine->engineState.injectionStage2Fraction, 0.01f);
	EXPECT_NEAR(5, engine->engineState.trailingSparkAngle, 0.1f);
	EXPECT_NEAR(8, engine->ignitionState.timingIatCorrection, 0.1f);
	EXPECT_EQ(3, engine->outputChannels.fuelClosedLoopBinIdx);
	EXPECT_NEAR(6, engine->cylinders[0].getIgnitionTimingBtdc() - engine->cylinders[3].getIgnitionTimingBtdc(), 0.1f);
}

TEST(AirmassFuelPublication, LegacyLambdaSelectorsUseMonitorAndDeviationAxes) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	NeverIdleController idleController;
	configureLegacyConsumerRun(eth, LM_MOCK, idleController);
	engineConfiguration->lambdaProtectionEnable = true;
	engineConfiguration->lambdaProtectionTimeout = 0;
	engineConfiguration->lambdaProtectionMinRpm = 1000;
	engineConfiguration->lambdaProtectionMinLoad = 20;
	engineConfiguration->lambdaProtectionMinTps = 0;
	engine->lambdaMonitor.lambdaMonitorCut = false;
	setTable(config->lambdaTable, 1);
	setLinearCurve(config->lambdaMaxDeviationLoadBins, 0, 100, 1);
	setLinearCurve(config->lambdaMaxDeviationRpmBins, 0, 8000, 1);
	for (size_t row = 0; row < efi::size(config->lambdaMaxDeviationTable); row++) {
		setArrayValues(config->lambdaMaxDeviationTable[row], 0.7f - 0.012f * config->lambdaMaxDeviationLoadBins[row]);
	}

	Sensor::setMockValue(SensorType::Tps1, 90);
	Sensor::setMockValue(SensorType::Map, 10);
	Sensor::setMockValue(SensorType::Lambda1, 1.5f);
	setTimeNowUs(10e6);

	// A custom monitor load below the minimum bypasses protection even though
	// the fuel load and deviation load would both be high.
	config->lambdaMonitorLoadSource = AFR_MAP;
	config->lambdaDeviationLoadSource = AFR_Tps;
	runFuelCalculation(eth);
	EXPECT_FALSE(engine->lambdaMonitor.isCut());

	// With the monitor enabled, a custom deviation load of 10 permits lambda 1.5;
	// the legacy fuel load of 50 would exceed the allowed deviation.
	engine->lambdaMonitor.lambdaMonitorCut = false;
	config->lambdaMonitorLoadSource = AFR_Tps;
	config->lambdaDeviationLoadSource = AFR_MAP;
	runFuelCalculation(eth);
	EXPECT_FALSE(engine->lambdaMonitor.isCut());
}

TEST(AirmassFuelPublication, PhysicalAfrtargetUsesCapturedCoordinateWithoutLiveRereads) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);
	engineConfiguration->afrOverrideMode = AFR_Tps;
	Sensor::setMockValue(SensorType::Tps1, 20);

	runFuelCalculation(eth);
	ASSERT_TRUE(engine->engineState.airmassCalculationValid);

	const auto token = engine->airmassInjectionState.beginCalculation(
			LM_SPEED_DENSITY, 1800, engine->getGlobalConfigurationVersion());
	FuelAirmassCapture capture;
	ASSERT_TRUE(captureFuelAirmassInputs(1800, token, capture));

	// Neither of these changes may influence the candidate already captured.
	Sensor::setMockValue(SensorType::Tps1, 90);
	Sensor::setMockValue(SensorType::Map, 120);
	engineConfiguration->afrOverrideMode = AFR_MAP;

	const auto candidate = calculateFuelPublication(1800, false, capture);
	ASSERT_TRUE(candidate.Valid);
	EXPECT_FLOAT_EQ(20, candidate.ResolvedLambdaLoad);
	EXPECT_FLOAT_EQ(20, candidate.AfrTableYAxis);
}

TEST(AirmassFuelPublication, FastLoopUsesCurrentCandidateConsumerCoordinates) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);

	engineConfiguration->isInjectionEnabled = true;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->timingMode = TM_DYNAMIC;
	engineConfiguration->enableStagedInjection = true;
	engineConfiguration->enableTrailingSparks = true;
	engineConfiguration->afrOverrideMode = AFR_Tps;
	Sensor::setMockValue(SensorType::Iat, 20);
	config->injectionPhaseLoadSource = AFR_Tps;
	config->stagingLoadSource = AFR_Tps;
	config->trailingSparkLoadSource = AFR_Tps;
	config->ignitionIatLoadSource = AFR_Tps;
	config->ignitionTrimLoadSource[0] = AFR_Tps;
	config->ignitionTrimLoadSource[1] = AFR_MAP;

	setLinearCurve(config->injPhaseLoadBins, 0, 100, 1);
	setLinearCurve(config->injectorStagingLoadBins, 0, 100, 1);
	setLinearCurve(config->trailingIgnitionLoadBins, 0, 100, 1);
	setLinearCurve(config->ignitionIatCorrLoadBins, 0, 100, 1);
	setLinearCurve(config->ignTrimLoadBins, 0, 100, 1);
	for (size_t row = 0; row < efi::size(config->injectionPhase); row++) {
		setArrayValues(config->injectionPhase[row], config->injPhaseLoadBins[row]);
	}
	for (size_t row = 0; row < efi::size(config->injectorStagingTable); row++) {
		setArrayValues(config->injectorStagingTable[row], config->injectorStagingLoadBins[row]);
	}
	for (size_t row = 0; row < efi::size(config->trailingIgnitionTable); row++) {
		setArrayValues(config->trailingIgnitionTable[row], config->trailingIgnitionLoadBins[row] / 10.0f);
	}
	for (size_t row = 0; row < efi::size(config->ignitionIatCorrTable); row++) {
		setArrayValues(config->ignitionIatCorrTable[row], config->ignitionIatCorrLoadBins[row] / 10.0f);
	}
	for (size_t row = 0; row < efi::size(config->ignTrims[0].table); row++) {
		setArrayValues(config->ignTrims[0].table[row], config->ignTrimLoadBins[row] / 4.0f);
		setArrayValues(config->ignTrims[1].table[row], config->ignTrimLoadBins[row] / 4.0f);
	}

	Sensor::setMockValue(SensorType::Map, 80);
	Sensor::setMockValue(SensorType::Tps1, 20);
	runFuelCalculation(eth);
	ASSERT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_NEAR(20, engine->engineState.injectionOffset, 0.1f);
	EXPECT_NEAR(0.20f, engine->engineState.injectionStage2Fraction, 0.01f);
	EXPECT_NEAR(2, engine->engineState.trailingSparkAngle, 0.1f);
	EXPECT_NEAR(2, engine->ignitionState.timingIatCorrection, 0.1f);
	const float firstTimingDelta =
			engine->cylinders[1].getIgnitionTimingBtdc() - engine->cylinders[0].getIgnitionTimingBtdc();
	const float firstTiming = engine->cylinders[0].getIgnitionTimingBtdc();
	EXPECT_NEAR(15, firstTimingDelta, 0.1f);

	// A selector-context bug would keep using the previous published load (20).
	Sensor::setMockValue(SensorType::Tps1, 40);
	runFuelCalculation(eth);
	ASSERT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_NEAR(40, engine->engineState.injectionOffset, 0.1f);
	EXPECT_NEAR(0.40f, engine->engineState.injectionStage2Fraction, 0.01f);
	EXPECT_NEAR(4, engine->engineState.trailingSparkAngle, 0.1f);
	EXPECT_NEAR(4, engine->ignitionState.timingIatCorrection, 0.1f);
	EXPECT_FLOAT_EQ(40, engine->fuelComputer.afrTableYAxis);
	EXPECT_NEAR(10, engine->cylinders[1].getIgnitionTimingBtdc() - engine->cylinders[0].getIgnitionTimingBtdc(), 0.1f);
	EXPECT_NE(firstTiming, engine->cylinders[0].getIgnitionTimingBtdc());
}

TEST(AirmassFuelPublication, MissingOptionalCoordinateDoesNotCutFuel) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);
	config->injectionPhaseLoadSource = AFR_AccPedal;
	Sensor::setInvalidMockValue(SensorType::AcceleratorPedal);

	runFuelCalculation(eth);

	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
}

TEST(AirmassFuelPublication, StftUsesCurrentCandidateSelector) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);
	engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	engineConfiguration->stft.startupDelay = 0;
	engineConfiguration->stft.minClt = 60;
	engineConfiguration->stft.minLambda = 0.7f;
	engineConfiguration->stft.maxLambda = 1.2f;
	engineConfiguration->stft.maxOverrunLoad = 30;
	engineConfiguration->stft.minPowerLoad = 50;
	engineConfiguration->noFuelTrimAfterDfcoTime = 0;
	config->stftLoadSource = AFR_Tps;
	Sensor::setMockValue(SensorType::Clt, 80);
	Sensor::setMockValue(SensorType::Lambda1, 0.9f);

	struct : public MockIdleController {
		bool isIdlingOrTaper() const override {
			return false;
		}
	} idleController;
	engine->engineModules.get<IdleController>().set(&idleController);

	engine->rpmCalculator.setRpmValue(1800);
	engine->fuelComputer.running.timeSinceCrankingInSecs = 10;

	Sensor::setMockValue(SensorType::Tps1, 20);
	runFuelCalculation(eth);
	EXPECT_EQ(1, engine->outputChannels.fuelClosedLoopBinIdx);

	// The previous publication had load 20, which is in the overrun cell.
	Sensor::setMockValue(SensorType::Tps1, 60);
	runFuelCalculation(eth);
	EXPECT_EQ(2, engine->outputChannels.fuelClosedLoopBinIdx);
}

TEST(AirmassFuelPublication, DisabledLatchedMonitorKeepsRestoreCoordinateWithoutTarget) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityFuel(eth);
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->lambdaProtectionEnable = false;
	engineConfiguration->lambdaProtectionRestoreRpm = 2500;
	engineConfiguration->lambdaProtectionRestoreLoad = 50;
	engineConfiguration->lambdaProtectionRestoreTps = 100;
	engineConfiguration->afrOverrideMode = AFR_AccPedal;
	config->lambdaMonitorLoadSource = AFR_Tps;
	Sensor::setInvalidMockValue(SensorType::AcceleratorPedal);
	Sensor::setMockValue(SensorType::Tps1, 40);
	engine->lambdaMonitor.lambdaMonitorCut = true;
	engine->fuelComputer.targetLambda = 0.8f;

	runFuelCalculation(eth);

	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_FALSE(engine->lambdaMonitor.isCut());
	EXPECT_FLOAT_EQ(0, engine->cylinders[0].getInjectionMass());
	EXPECT_FLOAT_EQ(0, engine->fuelComputer.targetLambda);
	EXPECT_FLOAT_EQ(40, engine->outputChannels.lambdaMonitorLoad);
}
