#include "pch.h"

#include "accel_enrichment.h"
#include "airmass_calibration.h"
#include "fuel_math.h"
#include "tunerstudio.h"

namespace {

constexpr float TrimGasConstant = 0.28705f;

mass_t trimIdealGasMass(float displacement, int cylinders, float vePercent, float pressure, float temperatureK) {
	return (vePercent * 0.01f) * displacement * pressure / (TrimGasConstant * temperatureK) / cylinders;
}

class CylinderFuelTrimIntegrationTest : public ::testing::Test {
protected:
	CylinderFuelTrimIntegrationTest()
		: eth(engine_type_e::TEST_ENGINE) {
		engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
		engineConfiguration->useSeparateVeForIdle = false;
		engineConfiguration->displacement = 4;
		setCylinderCount(4);
		engine->engineState.sd.tChargeK = 300;
		setTable(config->veTable, 60);
		setTable(config->alphaNTable, 40);
		setTable(config->lambdaTable, 1);
		Sensor::setMockValue(SensorType::Rpm, 2500);
		Sensor::setMockValue(SensorType::Map, 50);
		Sensor::setMockValue(SensorType::Tps1, 20);
		Sensor::setMockValue(SensorType::DriverThrottleIntent, 20);
		Sensor::setMockValue(SensorType::AcceleratorPedal, 30);
		Sensor::setMockValue(SensorType::Iat, 20);
		Sensor::setMockValue(SensorType::Clt, 80);

		setLinearCurve(config->sdCylinderFuelTrimLoadBins, 0, 300);
		setLinearCurve(config->sdCylinderFuelTrimRpmBins, 0, 7000);
		setLinearCurve(config->alphaNCylinderFuelTrimTpsBins, 0, 100);
		setLinearCurve(config->alphaNCylinderFuelTrimRpmBins, 0, 7000);
		for (size_t cylinder = 0; cylinder < MAX_CYLINDER_COUNT; cylinder++) {
			setTable(config->sdCylinderFuelTrims[cylinder].table, 0);
			setTable(config->alphaNCylinderFuelTrims[cylinder].table, 0);
		}
		invalidateAirmassCalibration();
	}

	EngineTestHelper eth;
};

template <typename T>
void liveCalibrationWrite(T& destination, float value) {
	TunerStudio tunerStudio;
	::testing::NiceMock<MockTsChannel> channel;
	const auto offset = reinterpret_cast<uint8_t*>(&destination) - reinterpret_cast<uint8_t*>(config);
	T payload;
	payload = value;
	tunerStudio.handleWriteChunkCommand(&channel, offset, sizeof(T), &payload);
}

} // namespace

TEST_F(CylinderFuelTrimIntegrationTest, UnequalBlendMassesTrimOnlyModeledPerCylinderFuel) {
	engineConfiguration->fuelAlgorithm = LM_SD_ALPHA_N;
	setTable(config->airmassBlendTable, 50);
	invalidateAirmassCalibration();

	engine->periodicFastCallback();
	const float baseline = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(baseline, 0);
	for (size_t i = 1; i < 4; i++) {
		EXPECT_NEAR(engine->cylinders[i].getInjectionMass(), baseline, EPS4D);
	}

	const float sdTrim[4] = {10, -10, 5, 0};
	const float alphaNTrim[4] = {0, 0, 0, 20};
	for (size_t i = 0; i < 4; i++) {
		setTable(config->sdCylinderFuelTrims[i].table, sdTrim[i]);
		setTable(config->alphaNCylinderFuelTrims[i].table, alphaNTrim[i]);
	}
	invalidateAirmassCalibration();

	const float sdMass = trimIdealGasMass(4, 4, 60, 50, 300);
	const float alphaNMass = trimIdealGasMass(4, 4, 40, 101.325f, 300);
	ASSERT_NE(sdMass, alphaNMass);
	const float sdWeighted = 0.5f * sdMass;
	const float alphaNWeighted = 0.5f * alphaNMass;
	const float total = sdWeighted + alphaNWeighted;

	engine->periodicFastCallback();
	for (size_t i = 0; i < 4; i++) {
		const float factor = 1 + (sdWeighted * sdTrim[i] + alphaNWeighted * alphaNTrim[i]) / total * 0.01f;
		EXPECT_NEAR(engine->cylinders[i].getInjectionMass(), baseline * factor, EPS4D) << i;
		EXPECT_NEAR(engine->outputChannels.cylinderModelFuelTrim[i], (factor - 1) * 100, EPS4D) << i;
	}

	EXPECT_EQ(
			engine->outputChannels.cylinderModelFuelTrimBranches,
			CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD | CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN);
	EXPECT_FLOAT_EQ(engine->outputChannels.sdCylinderFuelTrimLoad, 50);
	EXPECT_FLOAT_EQ(engine->outputChannels.alphaNCylinderFuelTrimLoad, 20);
}

TEST_F(CylinderFuelTrimIntegrationTest, ActiveEndpointIgnoresInactiveFamilyAndFallbackSelectsAlphaN) {
	engineConfiguration->fuelAlgorithm = LM_SD_ALPHA_N;
	setTable(config->sdCylinderFuelTrims[0].table, 10);
	setTable(config->alphaNCylinderFuelTrims[0].table, 20);

	setTable(config->airmassBlendTable, 0);
	setLinearCurve(config->alphaNCylinderFuelTrimRpmBins, 0, 0);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();

	EXPECT_GT(engine->cylinders[1].getInjectionMass(), 0);
	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), engine->cylinders[1].getInjectionMass() * 1.10f, EPS4D);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD);

	setLinearCurve(config->alphaNCylinderFuelTrimRpmBins, 0, 7000);
	setTable(config->airmassBlendTable, 50);
	setLinearCurve(config->veLoadBins, 0, 0);
	setLinearCurve(config->sdCylinderFuelTrimLoadBins, 0, 0);
	setTable(config->sdCylinderFuelTrims[0].table, 26);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();

	EXPECT_GT(engine->cylinders[1].getInjectionMass(), 0);
	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), engine->cylinders[1].getInjectionMass() * 1.20f, EPS4D);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 100);
}

TEST_F(CylinderFuelTrimIntegrationTest, InvalidActiveFamilyRejectsWholeFuelPublication) {
	setTable(config->sdCylinderFuelTrims[0].table, 25);
	config->sdCylinderFuelTrimLoadBins[1] = config->sdCylinderFuelTrimLoadBins[0];
	invalidateAirmassCalibration();
	engine->periodicFastCallback();

	for (size_t i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(engine->cylinders[i].getInjectionMass(), 0) << i;
	}
	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 0), 1);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, 0);

	setLinearCurve(config->sdCylinderFuelTrimLoadBins, 0, 300);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_GT(getCylinderModelFuelTrimFactor(*engine, 0), 1);
}

TEST_F(CylinderFuelTrimIntegrationTest, AdditiveLuaAndFixedCrankingFuelStayOutsideModelTrim) {
	engineConfiguration->tpsAccelEnrichmentThreshold = 5;
	engineConfiguration->tpsAccelLookback = 2;
	engineConfiguration->tpsAccelFractionPeriod = 1;
	engineConfiguration->tpsAccelFractionDivisor = 1;
	setTable(config->tpsTpsAccelTable, 5);
	initAccelEnrichment();

	engine->rpmCalculator.setRpmValue(2500);
	engine->module<TpsAccelEnrichment>()->setLength(2);
	engine->module<TpsAccelEnrichment>()->onNewValue(0);
	engine->module<TpsAccelEnrichment>()->onNewValue(10);
	engine->periodicFastCallback();

	FuelAirmassCapture capture;
	ASSERT_TRUE(captureFuelAirmassInputs(2500, engine->airmassInjectionState.publicationEpoch(), capture, false));
	const auto candidate = calculateFuelPublication(2500, false, capture, false);
	ASSERT_TRUE(candidate.Valid);
	const float modeled = candidate.ModeledCycleFuelMass;
	const float additive = candidate.AdditiveCycleFuelMass;
	ASSERT_GT(modeled, 0);
	ASSERT_GT(additive, 0);
	EXPECT_FLOAT_EQ(getCycleInjectionMass(2500, false), candidate.CycleInjectionMass);

	setTable(config->sdCylinderFuelTrims[0].table, 25);
	engine->engineState.lua.fuelMult = 2;
	engine->engineState.lua.fuelAdd = 0.003f;
	invalidateAirmassCalibration();
	engine->periodicFastCallback();

	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), (modeled * 1.25f + additive) * 2 + 0.003f, EPS4D);
	for (size_t i = 1; i < 4; i++) {
		EXPECT_NEAR(engine->cylinders[i].getInjectionMass(), (modeled + additive) * 2 + 0.003f, EPS4D);
	}

	engine->engineState.lua.fuelMult = 1;
	engine->engineState.lua.fuelAdd = 0;
	engineConfiguration->tpsAccelLookback = 0;
	engine->module<TpsAccelEnrichment>()->resetAE();
	setTable(config->sdCylinderFuelTrims[0].table, 0);
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
	engine->periodicFastCallback();
	const float fixedCrankingMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(fixedCrankingMass, 0);

	setTable(config->sdCylinderFuelTrims[0].table, 25);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();
	for (size_t i = 0; i < 4; i++) {
		EXPECT_NEAR(engine->cylinders[i].getInjectionMass(), fixedCrankingMass, EPS4D) << i;
	}

	engineConfiguration->useRunningMathForCranking = true;
	setTable(config->sdCylinderFuelTrims[0].table, 0);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();
	const float modeledCrankingMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(modeledCrankingMass, 0);

	setTable(config->sdCylinderFuelTrims[0].table, 25);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();
	EXPECT_NEAR(engine->cylinders[0].getInjectionMass(), modeledCrankingMass * 1.25f, EPS4D);
	for (size_t i = 1; i < 4; i++) {
		EXPECT_NEAR(engine->cylinders[i].getInjectionMass(), modeledCrankingMass, EPS4D) << i;
	}
}

TEST_F(CylinderFuelTrimIntegrationTest, DisabledInjectionAndDfcoPublishZeroFuel) {
	setTable(config->sdCylinderFuelTrims[0].table, 25);
	engineConfiguration->isInjectionEnabled = false;
	config->sdCylinderFuelTrimLoadBins[1] = config->sdCylinderFuelTrimLoadBins[0];
	invalidateAirmassCalibration();
	engine->periodicFastCallback();

	for (size_t i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(engine->cylinders[i].getInjectionMass(), 0) << i;
	}
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 0), 1);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, 0);

	engineConfiguration->isInjectionEnabled = true;
	setLinearCurve(config->sdCylinderFuelTrimLoadBins, 0, 300);
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
	engineConfiguration->tpsAccelEnrichmentThreshold = 5;
	engineConfiguration->tpsAccelLookback = 2;
	engineConfiguration->tpsAccelFractionPeriod = 1;
	engineConfiguration->tpsAccelFractionDivisor = 1;
	setTable(config->tpsTpsAccelTable, 5);
	initAccelEnrichment();
	engine->module<TpsAccelEnrichment>()->setLength(2);
	engine->module<TpsAccelEnrichment>()->onNewValue(0);
	engine->module<TpsAccelEnrichment>()->onNewValue(10);
	engine->engineState.lua.fuelAdd = 0.003f;
	Sensor::setMockValue(SensorType::Rpm, 2500);
	Sensor::setMockValue(SensorType::Map, 20);
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 0);
	engine->rpmCalculator.setRpmValue(2500);
	invalidateAirmassCalibration();
	auto& dfco = engine->module<DfcoController>().unmock();
	engine->periodicFastCallback();

	EXPECT_TRUE(dfco.cutFuel());
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(engine->engineState.tpsAccelEnrich, 0);
	for (size_t i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(engine->cylinders[i].getInjectionMass(), 0) << i;
	}
}

TEST_F(CylinderFuelTrimIntegrationTest, LiveTuneWriteBeforeCommitPreservesLastGoodMasses) {
	setTable(config->sdCylinderFuelTrims[0].table, 10);
	invalidateAirmassCalibration();
	engine->periodicFastCallback();
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());

	const auto version = engine->getGlobalConfigurationVersion();
	const auto generation = engine->airmassCalibration.Generation;
	float previousMasses[MAX_CYLINDER_COUNT] = {};
	for (size_t i = 0; i < 4; i++) {
		previousMasses[i] = engine->cylinders[i].getInjectionMass();
		ASSERT_GT(previousMasses[i], 0) << i;
	}
	ASSERT_GT(getCylinderModelFuelTrimFactor(*engine, 0), 1);

	invalidateAirmassCalibration();
	bool wrote = false;
	engine->onAirmassCalibrationScanned = [&] {
		if (wrote) {
			return;
		}
		wrote = true;
		liveCalibrationWrite(config->sdCylinderFuelTrims[1].table[2][5], 20.0f);
	};
	engine->periodicFastCallback();
	engine->onAirmassCalibrationScanned = nullptr;
	ASSERT_TRUE(wrote);

	EXPECT_EQ(engine->getGlobalConfigurationVersion(), version);
	EXPECT_GT(engine->airmassCalibration.Generation, generation);
	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	for (size_t i = 0; i < 4; i++) {
		EXPECT_FLOAT_EQ(engine->cylinders[i].getInjectionMass(), previousMasses[i]) << i;
	}
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 0), 1);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, 0);

	engine->periodicFastCallback();
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_GT(getCylinderModelFuelTrimFactor(*engine, 1), 1);
	EXPECT_GT(engine->cylinders[1].getInjectionMass(), 0);
}
