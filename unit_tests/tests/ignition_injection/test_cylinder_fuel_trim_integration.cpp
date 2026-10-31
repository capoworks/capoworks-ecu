#include "pch.h"

#include "accel_enrichment.h"
#include "airmass_calibration.h"
#include "closed_loop_fuel.h"
#include "cylinder_fuel_trim.h"
#include "fuel_math.h"
#include "tunerstudio.h"

#ifdef MODULE_CYLINDER_FUEL_TRIM

#if CYLINDER_FUEL_TRIM_COUNT >= 4

namespace {

constexpr size_t TrimCylinders = 4;

static_assert(CYLINDER_FUEL_TRIM_COUNT >= TrimCylinders);

#if CYLINDER_FUEL_TRIM_COUNT < MAX_CYLINDER_COUNT
constexpr size_t oversizedCylinderCount() {
	switch (CYLINDER_FUEL_TRIM_COUNT) {
		case 6:
			return 8;
		case 10:
			return 12;
		default:
			return CYLINDER_FUEL_TRIM_COUNT + 1;
	}
}
#endif

void configureTrimAxes() {
	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 300, 20);
	setLinearCurve(config->cylinderFuelTrimRpmBins, 0, 7500, 500);
}

void configureTrimDefaults() {
	configureTrimAxes();
	for (auto& source : config->cylinderFuelTrimLoadSource) {
		source = AFR_None;
	}
	for (auto& trim : config->cylinderFuelTrims) {
		setTable(trim.table, 0);
	}
}

void setAllTrims(float percent) {
	for (auto& trim : config->cylinderFuelTrims) {
		setTable(trim.table, percent);
	}
}

void setTrim(size_t cylinder, float percent) {
	ASSERT_LT(cylinder, CYLINDER_FUEL_TRIM_COUNT);
	setTable(config->cylinderFuelTrims[cylinder].table, percent);
}

struct StftCellAdjustmentReset {
	~StftCellAdjustmentReset() {
		auto& cellConfig = engineConfiguration->stft.cellCfgs[3];
		cellConfig.timeConstant = 0.1f;
		cellConfig.maxAdd = 0;
		cellConfig.maxRemove = 0;
		fuelClosedLoopCorrection(50, 1);
	}
};

void expectNeutralTrimTelemetry() {
	for (size_t cylinder = 0; cylinder < CYLINDER_FUEL_TRIM_COUNT; cylinder++) {
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrim[cylinder], 0) << cylinder;
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrimLoad[cylinder], 0) << cylinder;
	}
}

template <typename T>
void liveCalibrationWrite(T& destination, float value) {
	TunerStudio tunerStudio;
	::testing::NiceMock<MockTsChannel> channel;
	const auto offset = reinterpret_cast<uint8_t*>(&destination) - reinterpret_cast<uint8_t*>(config);
	T payload = value;
	tunerStudio.handleWriteChunkCommand(&channel, offset, sizeof(T), &payload);
}

class CylinderFuelTrimIntegration : public ::testing::Test {
protected:
	CylinderFuelTrimIntegration()
		: eth(engine_type_e::TEST_ENGINE) {
		engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
		engineConfiguration->useSeparateVeForIdle = false;
		engineConfiguration->displacement = 4;
		engineConfiguration->isInjectionEnabled = true;
		engineConfiguration->isIgnitionEnabled = false;
		setCylinderCount(TrimCylinders);

		engine->engineState.sd.tChargeK = 300;
		engine->rpmCalculator.setRpmValue(2500);
		setLinearCurve(config->veLoadBins, 0, 200, 1);
		setLinearCurve(config->veRpmBins, 0, 8000, 1);
		setTable(config->veTable, 60);
		setLinearCurve(config->alphaNTpsBins, 0, 100, 1);
		setLinearCurve(config->alphaNRpmBins, 0, 8000, 1);
		setTable(config->alphaNTable, 40);
		setLinearCurve(config->mafTrimLoadBins, 0, 140, 1);
		setLinearCurve(config->mafTrimRpmBins, 0, 8000, 1);
		setTable(config->mafTrimTable, 100);
		setLinearCurve(config->airmassBlendLoadBins, 0, 100, 1);
		setLinearCurve(config->airmassBlendRpmBins, 0, 8000, 1);
		config->airmassBlendLoadSource = AirmassBlendLoadSource::Tps;
		setTable(config->airmassBlendTable, 50);
		setTable(config->lambdaTable, 1);

		Sensor::setMockValue(SensorType::Rpm, 2500);
		Sensor::setMockValue(SensorType::Map, 50);
		Sensor::setMockValue(SensorType::Tps1, 20);
		Sensor::setMockValue(SensorType::DriverThrottleIntent, 20);
		Sensor::setMockValue(SensorType::AcceleratorPedal, 30);
		Sensor::setMockValue(SensorType::Iat, 20);
		Sensor::setMockValue(SensorType::Clt, 80);
		Sensor::setMockValue(SensorType::Maf, 72);

		configureTrimDefaults();
		invalidateAirmassCalibration();
	}

	void selectModel(engine_load_mode_e mode) {
		engineConfiguration->fuelAlgorithm = mode;
		invalidateAirmassCalibration();
	}

	void runFuelCalculation() {
		engine->periodicFastCallback();
	}

	EngineTestHelper eth;
};

} // namespace

TEST_F(CylinderFuelTrimIntegration, CapturedModelFamiliesApplyOneSingleFactor) {
	for (const auto mode : {LM_SPEED_DENSITY, LM_ALPHA_N, LM_REAL_MAF, LM_SD_ALPHA_N}) {
		selectModel(mode);
		setAllTrims(0);
		invalidateAirmassCalibration();
		runFuelCalculation();

		ASSERT_TRUE(engine->airmassInjectionState.allowInjection()) << static_cast<int>(mode);
		ASSERT_TRUE(engine->engineState.airmassCalculationValid) << static_cast<int>(mode);
		float baseline[TrimCylinders];
		for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
			baseline[cylinder] = engine->cylinders[cylinder].getInjectionMass();
			ASSERT_GT(baseline[cylinder], 0) << "mode " << static_cast<int>(mode) << " cylinder " << cylinder;
		}

		setTrim(0, 10);
		setTrim(1, -5);
		invalidateAirmassCalibration();
		runFuelCalculation();

		ASSERT_TRUE(engine->airmassInjectionState.allowInjection()) << static_cast<int>(mode);
		EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), baseline[0] * 1.10f, EPS4D) << static_cast<int>(mode);
		EXPECT_NEAR(engine->cylinders[1].getInjectionMass(), baseline[1] * 0.95f, EPS4D) << static_cast<int>(mode);
		for (size_t cylinder = 2; cylinder < TrimCylinders; cylinder++) {
			EXPECT_NEAR(engine->cylinders[cylinder].getInjectionMass(), baseline[cylinder], EPS4D)
					<< "mode " << static_cast<int>(mode) << " cylinder " << cylinder;
		}
		EXPECT_NEAR(engine->outputChannels.cylinderFuelTrim[0], 10, EPS4D);
		EXPECT_NEAR(engine->outputChannels.cylinderFuelTrim[1], -5, EPS4D);
		EXPECT_TRUE(std::isfinite(engine->outputChannels.cylinderFuelTrimLoad[0]));
	}
}

TEST_F(CylinderFuelTrimIntegration, AdditiveAndLuaFuelStayOutsideModelTrim) {
	engineConfiguration->tpsAccelEnrichmentThreshold = 5;
	engineConfiguration->tpsAccelLookback = 2;
	engineConfiguration->tpsAccelFractionPeriod = 1;
	engineConfiguration->tpsAccelFractionDivisor = 1;
	setTable(config->tpsTpsAccelTable, 5);
	initAccelEnrichment();

	engine->module<TpsAccelEnrichment>()->setLength(2);
	engine->module<TpsAccelEnrichment>()->onNewValue(0);
	engine->module<TpsAccelEnrichment>()->onNewValue(10);
	setAllTrims(0);
	invalidateAirmassCalibration();
	runFuelCalculation();

	FuelAirmassCapture capture;
	ASSERT_TRUE(captureFuelAirmassInputs(2500, engine->airmassInjectionState.publicationEpoch(), capture, false));
	const auto candidate = calculateFuelPublication(2500, false, capture, false);
	ASSERT_TRUE(candidate.Valid);
	const float modeled = candidate.ModeledCycleFuelMass;
	const float additive = candidate.AdditiveCycleFuelMass;
	ASSERT_GT(modeled, 0);
	ASSERT_GT(additive, 0);

	setTrim(0, 10);
	engine->engineState.lua.fuelMult = 2;
	engine->engineState.lua.fuelAdd = 0.003f;
	invalidateAirmassCalibration();
	runFuelCalculation();

	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), (modeled * 1.10f + additive) * 2 + 0.003f, EPS4D);
	for (size_t cylinder = 1; cylinder < TrimCylinders; cylinder++) {
		EXPECT_NEAR(engine->cylinders[cylinder].getInjectionMass(), (modeled + additive) * 2 + 0.003f, EPS4D)
				<< cylinder;
	}

	const auto trimmedProvenance = engine->cylinders[0].getFuelProvenance();
	const auto neutralTrimProvenance = engine->cylinders[1].getFuelProvenance();
	ASSERT_TRUE(trimmedProvenance.Valid);
	ASSERT_TRUE(neutralTrimProvenance.Valid);
	EXPECT_EQ(engine->airmassCalibration.Generation, trimmedProvenance.CalibrationGeneration);
	EXPECT_NEAR(trimmedProvenance.PreAeBaseFuelMass, modeled * 1.10f * 2 + 0.003f, EPS4D);
	EXPECT_NEAR(trimmedProvenance.AppliedAeFuelMass, additive * 2, EPS4D);
	EXPECT_NEAR(neutralTrimProvenance.PreAeBaseFuelMass, modeled * 2 + 0.003f, EPS4D);
	EXPECT_NEAR(neutralTrimProvenance.AppliedAeFuelMass, additive * 2, EPS4D);
	EXPECT_NEAR(
			engine->cylinders[0].getInjectionMass(),
			trimmedProvenance.PreAeBaseFuelMass + trimmedProvenance.AppliedAeFuelMass,
			EPS4D);
}

TEST_F(CylinderFuelTrimIntegration, FuelProvenanceAppliesBankTrimToBaseAndAe) {
	struct : public MockIdleController {
		bool isIdlingOrTaper() const override {
			return false;
		}
	} idleController;
	engine->engineModules.get<IdleController>().set(&idleController);

	Sensor::setMockValue(SensorType::Iat, 27);
	Sensor::setMockValue(SensorType::Clt, 27);
	engine->engineState.sd.tCharge = 27;
	engine->engineState.sd.tChargeK = 300.15f;

	engineConfiguration->tpsAccelEnrichmentThreshold = 5;
	engineConfiguration->tpsAccelLookback = 2;
	engineConfiguration->tpsAccelFractionPeriod = 1;
	engineConfiguration->tpsAccelFractionDivisor = 1;
	setTable(config->tpsTpsAccelTable, 5);
	initAccelEnrichment();

	engine->module<TpsAccelEnrichment>()->setLength(2);
	engine->module<TpsAccelEnrichment>()->onNewValue(0);
	engine->module<TpsAccelEnrichment>()->onNewValue(10);
	setAllTrims(0);
	invalidateAirmassCalibration();
	runFuelCalculation();

	const float modeledTChargeC = engine->engineState.sd.tCharge;
	const float modeledTChargeK = engine->engineState.sd.tChargeK;
	EXPECT_FLOAT_EQ(27, modeledTChargeC);
	EXPECT_FLOAT_EQ(300.15f, modeledTChargeK);

	FuelAirmassCapture capture;
	ASSERT_TRUE(captureFuelAirmassInputs(2500, engine->airmassInjectionState.publicationEpoch(), capture, false));
	const auto candidate = calculateFuelPublication(2500, false, capture, false);
	ASSERT_TRUE(candidate.Valid);
	const float modeled = candidate.ModeledCycleFuelMass;
	const float additive = candidate.AdditiveCycleFuelMass;
	ASSERT_GT(modeled, 0);
	ASSERT_GT(additive, 0);

	setTrim(0, 10);
	engine->engineState.lua.fuelMult = 2;
	engine->engineState.lua.fuelAdd = 0.003f;
	engineConfiguration->fuelClosedLoopCorrectionEnabled = true;
	engineConfiguration->stft.startupDelay = 0;
	engineConfiguration->stft.minClt = 0;
	engineConfiguration->stft.maxOverrunLoad = 30;
	engineConfiguration->stft.minPowerLoad = 70;
	for (auto& cellConfig : engineConfiguration->stft.cellCfgs) {
		cellConfig.timeConstant = 0.1f;
		cellConfig.maxAdd = 0;
		cellConfig.maxRemove = 0;
	}
	engineConfiguration->stft.cellCfgs[3].timeConstant = 0.1f;
	engineConfiguration->stft.cellCfgs[3].maxAdd = 20;
	engineConfiguration->stft.cellCfgs[3].maxRemove = -20;
	engineConfiguration->noFuelTrimAfterDfcoTime = 0;
	engineConfiguration->cylinderBankSelect[0] = 0;
	engine->fuelComputer.running.timeSinceCrankingInSecs = 10;
	Sensor::setMockValue(SensorType::Lambda1, 0.8f);
	setTimeNowUs(3e6);
	StftCellAdjustmentReset stftCellAdjustmentReset;

	// Saturate bank 0 at its configured 20% removal before publishing fuel.
	for (size_t i = 0; i < 100; i++) {
		fuelClosedLoopCorrection(50, 1);
	}

	invalidateAirmassCalibration();
	runFuelCalculation();

	EXPECT_FLOAT_EQ(modeledTChargeC, engine->engineState.sd.tCharge);
	EXPECT_FLOAT_EQ(modeledTChargeK, engine->engineState.sd.tChargeK);

	const float bankTrim = engine->stftCorrection[0];
	const auto provenance = engine->cylinders[0].getFuelProvenance();
	ASSERT_TRUE(provenance.Valid);
	EXPECT_NEAR(0.8f, bankTrim, EPS4D);
	EXPECT_NEAR(provenance.PreAeBaseFuelMass, (modeled * 1.10f * 2 + 0.003f) * bankTrim, EPS4D);
	EXPECT_NEAR(provenance.AppliedAeFuelMass, additive * 2 * bankTrim, EPS4D);
	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), ((modeled * 1.10f + additive) * 2 + 0.003f) * bankTrim, EPS4D);
}

TEST_F(CylinderFuelTrimIntegration, DisabledInjectionBypassesInvalidTrimCalibration) {
	setTrim(0, 25);
	config->cylinderFuelTrimLoadBins[1] = config->cylinderFuelTrimLoadBins[0];
	engineConfiguration->isInjectionEnabled = false;
	invalidateAirmassCalibration();
	runFuelCalculation();

	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_FLOAT_EQ(engine->cylinders[cylinder].getInjectionMass(), 0) << cylinder;
	}
	expectNeutralTrimTelemetry();

	engineConfiguration->isInjectionEnabled = true;
	configureTrimAxes();
	invalidateAirmassCalibration();
	runFuelCalculation();
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_NEAR(engine->outputChannels.cylinderFuelTrim[0], 25, EPS4D);
}

TEST_F(CylinderFuelTrimIntegration, DfcoRemovesFuelInsteadOfLeavingTrimmedResidue) {
	setTrim(0, 10);
	engineConfiguration->coastingFuelCutEnabled = true;
	engineConfiguration->coastingFuelCutRpmLow = 1200;
	engineConfiguration->coastingFuelCutRpmHigh = 2000;
	engineConfiguration->coastingFuelCutTps = 5;
	engineConfiguration->coastingFuelCutMap = 60;
	engineConfiguration->coastingFuelCutClt = 60;
	engineConfiguration->coastingFuelCutVssHigh = 0;
	engineConfiguration->coastingFuelCutVssLow = 0;
	engineConfiguration->dfcoDelay = 0;
	engineConfiguration->disableFuelCutOnClutch = true;
	Sensor::setMockValue(SensorType::Map, 20);
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 0);
	engine->rpmCalculator.setRpmValue(2500);
	invalidateAirmassCalibration();

	auto& dfco = engine->module<DfcoController>().unmock();
	runFuelCalculation();

	EXPECT_TRUE(dfco.cutFuel());
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_FLOAT_EQ(engine->cylinders[cylinder].getInjectionMass(), 0) << cylinder;
	}
}

TEST_F(CylinderFuelTrimIntegration, FixedCrankingFuelIsNeutralButRunningCrankingMathIsTrimmed) {
	engineConfiguration->useRunningMathForCranking = false;
	engineConfiguration->cranking.baseFuel = 100;
	engineConfiguration->cranking.rpm = 1000;
	setArrayValues(config->crankingCycleCoef, 1);
	setArrayValues(config->crankingFuelCoef, 1);
	setArrayValues(config->crankingTpsCoef, 1);
	engine->OnTriggerSynchronizationLost();
	Sensor::setMockValue(SensorType::Rpm, 500);
	engine->rpmCalculator.setRpmValue(500);
	ASSERT_TRUE(engine->rpmCalculator.isCranking());
	runFuelCalculation();
	const float fixedCrankingMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(fixedCrankingMass, 0);

	setTrim(0, 25);
	invalidateAirmassCalibration();
	runFuelCalculation();
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_NEAR(engine->cylinders[cylinder].getInjectionMass(), fixedCrankingMass, EPS4D) << cylinder;
	}
	EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrim[0], 0);

	engineConfiguration->useRunningMathForCranking = true;
	setTrim(0, 0);
	invalidateAirmassCalibration();
	runFuelCalculation();
	const float modeledCrankingMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(modeledCrankingMass, 0);

	setTrim(0, 25);
	invalidateAirmassCalibration();
	runFuelCalculation();
	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), modeledCrankingMass * 1.25f, EPS4D);
	for (size_t cylinder = 1; cylinder < TrimCylinders; cylinder++) {
		EXPECT_NEAR(engine->cylinders[cylinder].getInjectionMass(), modeledCrankingMass, EPS4D) << cylinder;
	}
}

TEST_F(CylinderFuelTrimIntegration, LiveWriteDuringPreparationCannotPublishStaleTrim) {
	setTrim(0, 10);
	invalidateAirmassCalibration();
	runFuelCalculation();
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());

	float previousMasses[TrimCylinders];
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		previousMasses[cylinder] = engine->cylinders[cylinder].getInjectionMass();
		ASSERT_GT(previousMasses[cylinder], 0) << cylinder;
	}
	ASSERT_GT(engine->outputChannels.cylinderFuelTrim[0], 0);

	invalidateAirmassCalibration();
	bool wrote = false;
	engine->onAirmassCalibrationScanned = [&] {
		if (wrote) {
			return;
		}
		wrote = true;
		liveCalibrationWrite(config->cylinderFuelTrims[1].table[2][5], 20.0f);
	};
	runFuelCalculation();
	engine->onAirmassCalibrationScanned = nullptr;
	ASSERT_TRUE(wrote);

	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_FLOAT_EQ(engine->cylinders[cylinder].getInjectionMass(), previousMasses[cylinder]) << cylinder;
	}
	expectNeutralTrimTelemetry();

	runFuelCalculation();
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(engine->outputChannels.cylinderFuelTrim[1], 0);
	EXPECT_GT(engine->cylinders[1].getInjectionMass(), 0);
}

TEST_F(CylinderFuelTrimIntegration, OversizedEngineIsConfigurationFaultBeforeLoadPreparation) {
#if CYLINDER_FUEL_TRIM_COUNT < MAX_CYLINDER_COUNT
	setCylinderCount(oversizedCylinderCount());
	invalidateAirmassCalibration();

	FuelAirmassCapture capture;
	ASSERT_TRUE(captureFuelAirmassInputs(2500, engine->airmassInjectionState.publicationEpoch(), capture, false));
	const auto candidate = calculateFuelPublication(2500, false, capture, false);

	EXPECT_FALSE(candidate.Valid);
	EXPECT_EQ(candidate.Fault, AirmassInjectionFault::Configuration);
	EXPECT_FALSE(candidate.ConsumerLoads.Prepared);
#else
	GTEST_SKIP() << "A 12-cylinder trim capacity has no larger legal MAX_CYLINDER_COUNT victim";
#endif
}

#endif // CYLINDER_FUEL_TRIM_COUNT >= 4

#endif // MODULE_CYLINDER_FUEL_TRIM
