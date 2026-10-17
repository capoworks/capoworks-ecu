#include "pch.h"

#include "alphan_airmass.h"
#include "blended_airmass.h"
#include "cylinder_model_fuel_trim.h"
#include "speed_density_airmass.h"

namespace {

void configureSdTrimAxes() {
	setLinearCurve(config->sdCylinderFuelTrimLoadBins, 0, 300, 1);
	setLinearCurve(config->sdCylinderFuelTrimRpmBins, 0, 16000, 1);
}

void configureAlphaNTrimAxes() {
	setLinearCurve(config->alphaNCylinderFuelTrimTpsBins, 0, 100, 1);
	setLinearCurve(config->alphaNCylinderFuelTrimRpmBins, 0, 16000, 1);
}

void configureTrimTables() {
	configureSdTrimAxes();
	configureAlphaNTrimAxes();
	for (size_t cylinder = 0; cylinder < MAX_CYLINDER_COUNT; cylinder++) {
		setTable(config->sdCylinderFuelTrims[cylinder].table, 0);
		setTable(config->alphaNCylinderFuelTrims[cylinder].table, 0);
	}
}

template <typename TTable>
void setConstantTrim(TTable& cells, float value) {
	for (auto& row : cells) {
		for (auto& cell : row) {
			cell = value;
		}
	}
}

AirmassInputs makeTrimInputs(uint8_t cylinderCount = 4) {
	AirmassInputs inputs;
	inputs.HasPublicationContext = true;
	inputs.PublicationEpoch = engine->airmassInjectionState.publicationEpoch();
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
	inputs.Rpm = 3000;
	inputs.CylinderCount = cylinderCount;
	return inputs;
}

AirmassEvaluation makeEvidence(float sdMass, float sdLoad, float alphaNMass, float alphaNTps) {
	AirmassEvaluation evaluation;
	evaluation.Valid = true;
	evaluation.TrimSdActive = true;
	evaluation.TrimSdWeightedMass = sdMass;
	evaluation.TrimSdLoadKpa = sdLoad;
	evaluation.TrimAlphaNActive = true;
	evaluation.TrimAlphaNWeightedMass = alphaNMass;
	evaluation.TrimAlphaNTps = alphaNTps;
	return evaluation;
}

} // namespace

TEST(CylinderModelFuelTrim, WeightedBlendFactorsPublishTransactionally) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();
	setConstantTrim(config->sdCylinderFuelTrims[0].table, 4);
	setConstantTrim(config->alphaNCylinderFuelTrims[0].table, 12);
	setConstantTrim(config->sdCylinderFuelTrims[1].table, -4);
	setConstantTrim(config->alphaNCylinderFuelTrims[1].table, -12);

	const auto inputs = makeTrimInputs();
	const auto evidence = makeEvidence(3, 60, 1, 25);

	CylinderModelFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	ASSERT_TRUE(candidate.Valid);
	EXPECT_EQ(candidate.CylinderCount, 4);
	EXPECT_EQ(candidate.Branches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD | CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN);
	EXPECT_FLOAT_EQ(candidate.Factor[0], 1.06f);
	EXPECT_FLOAT_EQ(candidate.Factor[1], 0.94f);
	EXPECT_FLOAT_EQ(candidate.Factor[2], 1);
	EXPECT_FLOAT_EQ(candidate.Factor[3], 1);
	EXPECT_FLOAT_EQ(candidate.Factor[4], 1);

	// Preparing is pure: no factor or telemetry cursor changes before commit.
	EXPECT_TRUE(isCylinderModelFuelTrimPublicationValid(*engine));
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 0), 1);
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.sdCylinderFuelTrimLoad, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.alphaNCylinderFuelTrimLoad, 0);

	ASSERT_TRUE(publishCylinderModelFuelTrim(*engine, candidate));
	EXPECT_TRUE(isCylinderModelFuelTrimPublicationValid(*engine));
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 0), 1.06f);
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 1), 0.94f);
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 2), 1);
	EXPECT_FLOAT_EQ(getCylinderModelFuelTrimFactor(*engine, 11), 1);
	EXPECT_FLOAT_EQ(engine->outputChannels.sdCylinderFuelTrimLoad, 60);
	EXPECT_FLOAT_EQ(engine->outputChannels.alphaNCylinderFuelTrimLoad, 25);
	EXPECT_EQ(
			engine->outputChannels.cylinderModelFuelTrimBranches,
			CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD | CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN);
	EXPECT_NEAR(engine->outputChannels.cylinderModelFuelTrim[0], 6, EPS4D);
	EXPECT_NEAR(engine->outputChannels.cylinderModelFuelTrim[1], -6, EPS4D);
}

TEST(CylinderModelFuelTrim, EndpointFamiliesIgnoreInactiveCalibration) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();
	setConstantTrim(config->sdCylinderFuelTrims[0].table, 8);

	// Invalid Alpha-N axes and cells must not affect an SD-only evaluation.
	config->alphaNCylinderFuelTrimTpsBins[1] = config->alphaNCylinderFuelTrimTpsBins[0];
	config->alphaNCylinderFuelTrims[0].table[0][0] = 26;

	AirmassEvaluation sdOnly;
	sdOnly.Valid = true;
	sdOnly.TrimSdActive = true;
	sdOnly.TrimSdWeightedMass = 2;
	sdOnly.TrimSdLoadKpa = 60;

	CylinderModelFuelTrimCandidate sdCandidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), sdOnly, sdCandidate));
	EXPECT_FLOAT_EQ(sdCandidate.Factor[0], 1.08f);
	EXPECT_EQ(sdCandidate.Branches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD);

	// Moving to the other endpoint ignores the now-invalid SD calibration too.
	config->sdCylinderFuelTrimLoadBins[1] = config->sdCylinderFuelTrimLoadBins[0];
	configureAlphaNTrimAxes();
	setConstantTrim(config->alphaNCylinderFuelTrims[0].table, -5);

	AirmassEvaluation alphaNOnly;
	alphaNOnly.Valid = true;
	alphaNOnly.TrimAlphaNActive = true;
	alphaNOnly.TrimAlphaNWeightedMass = 2;
	alphaNOnly.TrimAlphaNTps = 25;

	CylinderModelFuelTrimCandidate alphaNCandidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), alphaNOnly, alphaNCandidate));
	EXPECT_FLOAT_EQ(alphaNCandidate.Factor[0], 0.95f);
	EXPECT_EQ(alphaNCandidate.Branches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_AN);
}

TEST(CylinderModelFuelTrim, ZeroAirStaysNeutralButValidatesActiveFamily) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();

	const auto zeroEvidence = [] {
		AirmassEvaluation evaluation;
		evaluation.Valid = true;
		evaluation.TrimSdActive = true;
		evaluation.TrimSdWeightedMass = 0;
		evaluation.TrimSdLoadKpa = 20;
		return evaluation;
	}();

	CylinderModelFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), zeroEvidence, candidate));
	ASSERT_TRUE(candidate.Valid);
	EXPECT_FLOAT_EQ(candidate.Factor[0], 1);
	EXPECT_FLOAT_EQ(candidate.Factor[3], 1);
	ASSERT_TRUE(publishCylinderModelFuelTrim(*engine, candidate));
	EXPECT_TRUE(isCylinderModelFuelTrimPublicationValid(*engine));
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, CYLINDER_MODEL_FUEL_TRIM_BRANCH_SD);
	EXPECT_FLOAT_EQ(engine->outputChannels.sdCylinderFuelTrimLoad, 20);

	// A true zero must not bypass an invalid active family.
	config->sdCylinderFuelTrimLoadBins[1] = config->sdCylinderFuelTrimLoadBins[0];
	invalidateAirmassCalibration();
	CylinderModelFuelTrimCandidate invalid;
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), zeroEvidence, invalid));
	EXPECT_FALSE(invalid.Valid);
	EXPECT_FLOAT_EQ(invalid.Factor[0], 1);
}

TEST(CylinderModelFuelTrim, NoModelEvidencePublishesNeutralFactors) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();

	CylinderModelFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), AirmassEvaluation{}, candidate));
	ASSERT_TRUE(candidate.Valid);
	EXPECT_EQ(candidate.Branches, 0);
	for (float factor : candidate.Factor) {
		EXPECT_FLOAT_EQ(factor, 1);
	}

	ASSERT_TRUE(publishCylinderModelFuelTrim(*engine, candidate));
	EXPECT_EQ(engine->outputChannels.cylinderModelFuelTrimBranches, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.sdCylinderFuelTrimLoad, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.alphaNCylinderFuelTrimLoad, 0);
}

TEST(CylinderModelFuelTrim, InvalidEvidenceFailsWholeCandidate) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();

	CylinderModelFuelTrimCandidate candidate;

	auto invalidMass = makeEvidence(NAN, 50, 1, 20);
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), invalidMass, candidate));
	EXPECT_FALSE(candidate.Valid);

	auto negativeMass = makeEvidence(-1, 50, 0, 0);
	negativeMass.TrimAlphaNActive = false;
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), negativeMass, candidate));

	auto invalidLoad = makeEvidence(1, INFINITY, 0, 0);
	invalidLoad.TrimAlphaNActive = false;
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), invalidLoad, candidate));
	EXPECT_FLOAT_EQ(candidate.Factor[0], 1);

	auto negativeLoad = makeEvidence(1, -1, 0, 0);
	negativeLoad.TrimAlphaNActive = false;
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), negativeLoad, candidate));

	auto invalidCylinderCount = makeTrimInputs();
	invalidCylinderCount.CylinderCount = 4.5f;
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, invalidCylinderCount, makeEvidence(1, 50, 0, 0), candidate));

	config->sdCylinderFuelTrimRpmBins[2] = config->sdCylinderFuelTrimRpmBins[1];
	invalidateAirmassCalibration();
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), makeEvidence(1, 50, 0, 0), candidate));

	config->sdCylinderFuelTrimRpmBins[2] = 2000;
	config->sdCylinderFuelTrims[0].table[3][4] = 25.2f;
	invalidateAirmassCalibration();
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, makeTrimInputs(), makeEvidence(1, 50, 0, 0), candidate));
}

TEST(CylinderModelFuelTrim, CacheRescansAfterGenerationConfigurationAndCountChanges) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimTables();
	auto inputs = makeTrimInputs();
	auto evidence = makeEvidence(1, 50, 0, 0);
	evidence.TrimAlphaNActive = false;

	CylinderModelFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	EXPECT_EQ(engine->cylinderTrimCalibration.Scans, 1);

	// Calibration is cached until an identity field changes.
	config->sdCylinderFuelTrimLoadBins[1] = config->sdCylinderFuelTrimLoadBins[0];
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	EXPECT_EQ(engine->cylinderTrimCalibration.Scans, 1);

	invalidateAirmassCalibration();
	inputs = makeTrimInputs();
	EXPECT_FALSE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	EXPECT_EQ(engine->cylinderTrimCalibration.Scans, 2);

	configureSdTrimAxes();
	engine->globalConfigurationVersion++;
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	EXPECT_EQ(engine->cylinderTrimCalibration.Scans, 3);

	inputs.CylinderCount = 5;
	ASSERT_TRUE(prepareCylinderModelFuelTrim(*engine, inputs, evidence, candidate));
	EXPECT_EQ(engine->cylinderTrimCalibration.Scans, 4);
}

TEST(CylinderModelFuelTrimEvidence, StandaloneModelsOwnTheirCapturedCoordinates) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);

	config->airmassTemperatureSource = AirmassTemperatureSource::Tcharge;
	setLinearCurve(config->veLoadBins, 0, 200, 1);
	setLinearCurve(config->veRpmBins, 0, 9000, 1);
	setTable(config->veTable, 60);
	setLinearCurve(config->alphaNTpsBins, 0, 100, 1);
	setLinearCurve(config->alphaNRpmBins, 0, 9000, 1);
	setTable(config->alphaNTable, 40);

	AirmassInputs inputs;
	inputs.Rpm = 3000;
	inputs.NativeMap = {50, 0, true, true, false};
	inputs.MeasuredMap = expected<float>(50);
	inputs.Tps = expected<float>(20);
	inputs.TemperatureK = 300;
	inputs.TemperatureValid = true;
	inputs.Displacement = 2.4f;
	inputs.CylinderCount = 4;

	MockVp3d mapEstimate;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	const auto sdResult = sd.evaluateAirmass(inputs, nullptr);
	ASSERT_TRUE(sdResult.Valid);
	EXPECT_TRUE(sdResult.TrimSdActive);
	EXPECT_FALSE(sdResult.TrimAlphaNActive);
	EXPECT_FLOAT_EQ(sdResult.TrimSdWeightedMass, sdResult.Result.CylinderAirmass);
	EXPECT_FLOAT_EQ(sdResult.TrimSdLoadKpa, 50);

	inputs.AlphaNMultiplyMap = true;
	AlphaNAirmass alphaN;
	const auto alphaNResult = alphaN.evaluateAirmass(inputs, AlphaNPressurePolicy::EffectiveMap, nullptr);
	ASSERT_TRUE(alphaNResult.Valid);
	EXPECT_TRUE(alphaNResult.TrimAlphaNActive);
	EXPECT_FALSE(alphaNResult.TrimSdActive);
	EXPECT_FLOAT_EQ(alphaNResult.TrimAlphaNWeightedMass, alphaNResult.Result.CylinderAirmass);
	EXPECT_FLOAT_EQ(alphaNResult.TrimAlphaNTps, 20);
}

TEST(CylinderModelFuelTrimEvidence, BlendUsesEffectivePostFallbackWeights) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->fuelAlgorithm = LM_SD_ALPHA_N;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);

	setLinearCurve(config->veLoadBins, 0, 200, 1);
	setLinearCurve(config->veRpmBins, 0, 9000, 1);
	setTable(config->veTable, 60);
	setLinearCurve(config->alphaNTpsBins, 0, 100, 1);
	setLinearCurve(config->alphaNRpmBins, 0, 9000, 1);
	setTable(config->alphaNTable, 40);
	setLinearCurve(config->airmassBlendLoadBins, 0, 100, 1);
	setLinearCurve(config->airmassBlendRpmBins, 0, 9000, 1);
	config->airmassBlendLoadSource = AirmassBlendLoadSource::Tps;
	setTable(config->airmassBlendTable, 25);

	AirmassInputs inputs;
	inputs.Rpm = 3000;
	inputs.NativeMap = {50, 0, true, true, false};
	inputs.MeasuredMap = expected<float>(50);
	inputs.Tps = expected<float>(20);
	inputs.TemperatureK = 300;
	inputs.TemperatureValid = true;
	inputs.Displacement = 2.4f;
	inputs.CylinderCount = 4;
	inputs.BlendLoadSource = AirmassBlendLoadSource::Tps;

	MockVp3d mapEstimate;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	AlphaNAirmass alphaN;
	BlendedAirmass blended(sd, alphaN);
	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 25);
	EXPECT_TRUE(result.Airmass.TrimSdActive);
	EXPECT_TRUE(result.Airmass.TrimAlphaNActive);
	EXPECT_FLOAT_EQ(result.Airmass.TrimSdWeightedMass, diagnostics.SdMass * 0.75f);
	EXPECT_FLOAT_EQ(result.Airmass.TrimAlphaNWeightedMass, diagnostics.AlphaNMass * 0.25f);
	EXPECT_FLOAT_EQ(result.Airmass.TrimSdWeightedMass + result.Airmass.TrimAlphaNWeightedMass, diagnostics.RawMass);
	EXPECT_FLOAT_EQ(result.Airmass.TrimSdLoadKpa, 50);
	EXPECT_FLOAT_EQ(result.Airmass.TrimAlphaNTps, 20);

	inputs.Tps = expected<float>(NAN);
	const auto fallback = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(fallback.Airmass.Valid);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 0);
	EXPECT_TRUE(fallback.Airmass.TrimSdActive);
	EXPECT_FALSE(fallback.Airmass.TrimAlphaNActive);
	EXPECT_FLOAT_EQ(fallback.Airmass.TrimSdWeightedMass, diagnostics.SdMass);
	EXPECT_FLOAT_EQ(fallback.Airmass.TrimAlphaNWeightedMass, 0);
}
