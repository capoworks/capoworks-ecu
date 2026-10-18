#include "pch.h"

#include "airmass_calibration.h"
#include "alphan_airmass.h"
#include "maf_airmass.h"
#include "speed_density_airmass.h"

using testing::StrictMock;

namespace {
constexpr float AirGasConstant = 0.28705f;

mass_t expectedIdealGasMass(float displacement, int cylinders, float vePercent, float pressure, float temperatureK) {
	return (vePercent * 0.01f) * displacement * pressure / (AirGasConstant * temperatureK) / cylinders;
}

void configureMapEstimateAxes() {
	setLinearCurve(config->mapEstimateTpsBins, 0, 100, 1);
	setLinearCurve(config->mapEstimateRpmBins, 500, 8000, 1);
}

void configureSpeedDensityAxes() {
	setLinearCurve(config->veLoadBins, 0, 140, 1);
	setLinearCurve(config->veRpmBins, 500, 8000, 1);
}

void configureMafTrimAxes() {
	setLinearCurve(config->mafTrimLoadBins, 0, 140, 1);
	setLinearCurve(config->mafTrimRpmBins, 500, 8000, 1);
	setTable(config->mafTrimTable, 100);
}
} // namespace

TEST(AirmassEvaluation, AlphaNUsesTpsNativeLoadAndSelectedTemperature) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 3.2f;
	setCylinderCount(4);
	config->airmassTemperatureSource = AirmassTemperatureSource::Iat;
	Sensor::setMockValue(SensorType::Tps1, 12.5f);
	Sensor::setMockValue(SensorType::Iat, -3);
	setTable(config->alphaNTable, 52);

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	auto evaluation = alphaN.evaluateAirmass(1800, &diagnostics);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_FLOAT_EQ(diagnostics.TemperatureK, 270.15f);
	EXPECT_FALSE(diagnostics.TemperatureFallback);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, expectedIdealGasMass(3.2f, 4, 52, 101.325f, 270.15f), EPS4D);
}

TEST(AirmassEvaluation, InvalidTchargeUsesIatThenStandardTemperature) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	engine->engineState.sd.tChargeK = NAN;
	Sensor::setMockValue(SensorType::Tps1, 24);
	Sensor::setMockValue(SensorType::Map, 46);
	Sensor::setMockValue(SensorType::Iat, 30);

	StrictMock<MockVp3d> veTable;
	StrictMock<MockVp3d> mapEstimate;
	EXPECT_CALL(veTable, getValue(2100, 46)).Times(3).WillRepeatedly(testing::Return(58));
	SpeedDensityAirmass speedDensity(&veTable, mapEstimate);
	AirmassDiagnostics diagnostics;

	auto result = speedDensity.evaluateAirmass(2100, &diagnostics);
	ASSERT_TRUE(result.Valid);
	EXPECT_TRUE(result.Degraded);
	EXPECT_FLOAT_EQ(diagnostics.TemperatureK, 303.15f);
	EXPECT_TRUE(diagnostics.TemperatureFallback);

	Sensor::setInvalidMockValue(SensorType::Iat);
	result = speedDensity.evaluateAirmass(2100, &diagnostics);
	ASSERT_TRUE(result.Valid);
	EXPECT_FLOAT_EQ(diagnostics.TemperatureK, 293.15f);
	EXPECT_TRUE(diagnostics.TemperatureFallback);

	engine->engineState.sd.tChargeK = 310;
	result = speedDensity.evaluateAirmass(2100, &diagnostics);
	ASSERT_TRUE(result.Valid);
	EXPECT_FALSE(diagnostics.TemperatureFallback);
	EXPECT_FLOAT_EQ(diagnostics.TemperatureK, 310);
}

TEST(AirmassEvaluation, MapEstimateRequiresExplicitPermission) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.0f;
	setCylinderCount(4);
	setTable(config->veTable, 60);
	Sensor::setMockValue(SensorType::Tps1, 20);
	Sensor::setInvalidMockValue(SensorType::Map);

	StrictMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);

	config->useMapEstimateTable = false;
	EXPECT_FALSE(speedDensity.evaluateAirmass(2500).Valid);

	config->useMapEstimateTable = true;
	EXPECT_CALL(mapEstimate, getValue(2500, 20)).WillOnce(testing::Return(74));
	auto evaluation = speedDensity.evaluateAirmass(2500);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_TRUE(evaluation.Degraded);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, expectedIdealGasMass(2.0f, 4, 60, 74, 293.15f), EPS4D);
}

TEST(AirmassEvaluation, AlphaNBaroCompensationUsesCalibrationReference) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.0f;
	setCylinderCount(4);
	config->alphaNMultiplyMap = false;
	config->alphaNBaroCompensation = true;
	config->alphaNBaroReferencePressure = 100;
	setTable(config->alphaNTable, 50);
	Sensor::setMockValue(SensorType::Tps1, 30);
	Sensor::setMockValue(SensorType::BarometricPressure, 95);

	AirmassDiagnostics diagnostics;
	AlphaNAirmass alphaN;
	auto evaluation = alphaN.evaluateAirmass(2400, &diagnostics);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_FLOAT_EQ(diagnostics.BaroCoefficient, 0.95f);
	EXPECT_FALSE(diagnostics.BaroFallback);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, expectedIdealGasMass(2.0f, 4, 50, 101.325f * 0.95f, 293.15f), EPS4D);
}

TEST(AirmassEvaluation, AlphaNMultiplyMapOverridesReferencePressurePolicy) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.0f;
	setCylinderCount(4);
	config->alphaNMultiplyMap = true;
	config->alphaNBaroCompensation = true;
	setTable(config->alphaNTable, 40);
	Sensor::setMockValue(SensorType::Tps1, 45);
	Sensor::setMockValue(SensorType::Map, 80);
	Sensor::setMockValue(SensorType::BarometricPressure, 95);

	AlphaNAirmass alphaN;
	auto evaluation = alphaN.evaluateAirmass(3000);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, expectedIdealGasMass(2.0f, 4, 40, 80, 293.15f), EPS4D);
}

TEST(AirmassEvaluation, InvalidAlphaNTpsReturnsExpectedFailureWithoutPublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engine->engineState.currentVe = 91;
	engine->engineState.veTableYAxis = 92;
	engine->engineState.idleVeTableYAxis = 93;
	Sensor::setInvalidMockValue(SensorType::Tps1);

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	diagnostics.Ve.HasValue = true;
	diagnostics.Ve.Valid = true;
	auto evaluation = alphaN.evaluateAirmass(1900, &diagnostics);
	EXPECT_FALSE(evaluation.Valid);
	EXPECT_FALSE(diagnostics.Ve.HasValue);
	EXPECT_FALSE(diagnostics.Ve.Valid);

	const auto result = alphaN.getAirmass(1900, true);
	EXPECT_FALSE(result);
	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 91);
	EXPECT_FLOAT_EQ(engine->engineState.veTableYAxis, 92);
	EXPECT_FLOAT_EQ(engine->engineState.idleVeTableYAxis, 93);
}

TEST(AirmassEvaluation, DirectModelsUseNativeAxesDespiteLegacyVeOverride) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);
	Sensor::setMockValue(SensorType::Map, 63);
	Sensor::setMockValue(SensorType::Tps1, 12.5f);

	engineConfiguration->veOverrideMode = VE_TPS;
	StrictMock<MockVp3d> speedDensityVe;
	StrictMock<MockVp3d> mapEstimate;
	EXPECT_CALL(speedDensityVe, getValue(2200, 63)).WillOnce(testing::Return(60));
	SpeedDensityAirmass speedDensity(&speedDensityVe, mapEstimate);

	AirmassDiagnostics diagnostics;
	auto speedDensityEvaluation = speedDensity.evaluateAirmass(2200, &diagnostics);
	ASSERT_TRUE(speedDensityEvaluation.Valid);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Load, 63);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 60);

	engineConfiguration->veOverrideMode = VE_MAP;
	setTable(config->alphaNTable, 55);
	AlphaNAirmass alphaN;
	auto alphaNEvaluation = alphaN.evaluateAirmass(2200, &diagnostics);
	ASSERT_TRUE(alphaNEvaluation.Valid);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Load, 12.5f);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 55);
}

TEST(AirmassEvaluation, MapEstimateTpsBoundaryPolicyMatchesAcrossModels) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	engineConfiguration->tpsErrorDetectionTooLow = -10;
	engineConfiguration->tpsErrorDetectionTooHigh = 110;
	setCylinderCount(4);
	Sensor::setInvalidMockValue(SensorType::Map);
	config->useMapEstimateTable = true;
	configureMapEstimateAxes();
	setTable(config->mapEstimateTable, 73);
	setTable(config->alphaNTable, 55);

	StrictMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AlphaNAirmass alphaN;

	Sensor::setMockValue(SensorType::Tps1, -10);
	EXPECT_CALL(mapEstimate, getValue(2500, 0)).WillOnce(testing::Return(73));
	auto speedDensityBelow = speedDensity.evaluateMap(2500);
	AirmassDiagnostics diagnostics;
	auto alphaNBelow = alphaN.evaluateAirmass(2500, AlphaNPressurePolicy::EffectiveMap, &diagnostics);
	ASSERT_TRUE(alphaNBelow.Valid);
	EXPECT_TRUE(speedDensityBelow.Valid);
	EXPECT_TRUE(speedDensityBelow.UsesEstimate);
	EXPECT_TRUE(speedDensityBelow.Fallback);
	EXPECT_FLOAT_EQ(speedDensityBelow.FallbackMap, diagnostics.Map.FallbackMap);
	EXPECT_FLOAT_EQ(speedDensityBelow.Map, diagnostics.Map.Map);
	EXPECT_TRUE(diagnostics.Map.UsesEstimate);
	EXPECT_TRUE(diagnostics.Map.Fallback);

	Sensor::setMockValue(SensorType::Tps1, 110);
	EXPECT_CALL(mapEstimate, getValue(2500, 100)).WillOnce(testing::Return(73));
	auto speedDensityAbove = speedDensity.evaluateMap(2500);
	auto alphaNAbove = alphaN.evaluateAirmass(2500, AlphaNPressurePolicy::EffectiveMap, &diagnostics);
	ASSERT_TRUE(alphaNAbove.Valid);
	EXPECT_TRUE(speedDensityAbove.Valid);
	EXPECT_TRUE(speedDensityAbove.UsesEstimate);
	EXPECT_TRUE(speedDensityAbove.Fallback);
	EXPECT_FLOAT_EQ(speedDensityAbove.FallbackMap, diagnostics.Map.FallbackMap);
	EXPECT_FLOAT_EQ(speedDensityAbove.Map, diagnostics.Map.Map);
	EXPECT_TRUE(diagnostics.Map.UsesEstimate);
	EXPECT_TRUE(diagnostics.Map.Fallback);
}

TEST(AirmassEvaluation, InvalidTpsUsesZeroMapEstimateCoordinateAndFallbackFlag) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setInvalidMockValue(SensorType::Map);
	Sensor::setInvalidMockValue(SensorType::Tps1);
	config->useMapEstimateTable = true;

	StrictMock<MockVp3d> mapEstimate;
	EXPECT_CALL(mapEstimate, getValue(2500, 0)).WillOnce(testing::Return(73));
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	auto evaluation = speedDensity.evaluateMap(2500);
	EXPECT_TRUE(evaluation.HasValue);
	EXPECT_TRUE(evaluation.Valid);
	EXPECT_TRUE(evaluation.UsesEstimate);
	EXPECT_TRUE(evaluation.Fallback);
	EXPECT_FLOAT_EQ(evaluation.Map, 73);
}

TEST(AirmassEvaluation, InvalidMapEstimateIsNotAdmitted) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->tpsErrorDetectionTooLow = 0;
	engineConfiguration->tpsErrorDetectionTooHigh = 100;
	Sensor::setInvalidMockValue(SensorType::Map);
	Sensor::setMockValue(SensorType::Tps1, 20);
	config->useMapEstimateTable = true;

	StrictMock<MockVp3d> mapEstimate;
	EXPECT_CALL(mapEstimate, getValue(2500, 20)).WillOnce(testing::Return(601));
	auto evaluation = evaluateAirmassMap(2500, &mapEstimate);
	EXPECT_TRUE(evaluation.HasValue);
	EXPECT_FALSE(evaluation.Valid);
	EXPECT_FALSE(evaluation.UsesEstimate);
	EXPECT_TRUE(evaluation.Fallback);
	EXPECT_FLOAT_EQ(evaluation.Map, 0);
}

TEST(AirmassEvaluation, BrokenOptionalMapEstimateDoesNotDiscardMeasuredMap) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);
	Sensor::setMockValue(SensorType::Tps1, 20);
	Sensor::setMockValue(SensorType::Map, 54);
	config->useMapEstimateTable = true;
	configureMapEstimateAxes();
	config->mapEstimateTpsBins[1] = config->mapEstimateTpsBins[0];

	StrictMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	auto evaluation = speedDensity.evaluateMap(2500);
	EXPECT_TRUE(evaluation.Valid);
	EXPECT_FALSE(evaluation.Fallback);
	EXPECT_FALSE(evaluation.HasValue);
	EXPECT_FLOAT_EQ(evaluation.Map, 54);

	setTable(config->alphaNTable, 55);
	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	auto alphaNEvaluation = alphaN.evaluateAirmass(2500, AlphaNPressurePolicy::EffectiveMap, &diagnostics);
	EXPECT_TRUE(alphaNEvaluation.Valid);
	EXPECT_TRUE(diagnostics.Map.Valid);
	EXPECT_FALSE(diagnostics.Map.Fallback);
	EXPECT_FLOAT_EQ(diagnostics.Map.Map, 54);
	EXPECT_TRUE(isAirmassModelConfigurationValid(LM_SPEED_DENSITY));

	auto liveEvaluation = speedDensity.getAirmassForFuel(2500);
	EXPECT_TRUE(liveEvaluation.Valid);
	EXPECT_FLOAT_EQ(liveEvaluation.Result.EngineLoadPercent, 54);
}

TEST(AirmassEvaluation, CapturedCoreInputsRemainImmutable) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);
	config->airmassTemperatureSource = AirmassTemperatureSource::Tcharge;
	setTable(config->alphaNTable, 52);
	Sensor::setMockValue(SensorType::Tps1, 20);

	AirmassInputs inputs;
	captureAirmassInputs(1800, inputs, nullptr, false);

	engine->engineState.sd.tChargeK = 373.15f;
	engineConfiguration->displacement = 6.0f;
	setCylinderCount(8);
	Sensor::setMockValue(SensorType::Tps1, 80);

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	const auto evaluation = alphaN.evaluateAirmass(inputs, AlphaNPressurePolicy::PureReference, &diagnostics);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_FLOAT_EQ(evaluation.Result.EngineLoadPercent, 20);
	EXPECT_FLOAT_EQ(diagnostics.TemperatureK, 293.15f);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, expectedIdealGasMass(2.4f, 4, 52, 101.325f, 293.15f), EPS4D);
}

TEST(AirmassEvaluation, VeCorrectionsUseCapturedPreviousLoads) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	setCylinderCount(4);
	setTable(config->veTable, 60);
	configureSpeedDensityAxes();

	for (auto& blend : config->veBlends) {
		blend.blendParameter = GPPWM_Zero;
	}
	auto& correction = config->veBlends[0];
	correction.blendParameter = GPPWM_FuelLoad;
	correction.yAxisOverride = GPPWM_IgnLoad;
	correction.veTableSelect = VeTableType::SpeedDensity;
	setLinearCurve(correction.loadBins, 0, 140, 1);
	setLinearCurve(correction.rpmBins, 500, 8000, 1);
	setTable(correction.table, 20);
	setLinearCurve(correction.blendBins, 0, 100, 1);
	setArrayValues(correction.blendValues, 100);

	Sensor::setMockValue(SensorType::Map, 54);
	engine->engineState.fuelingLoad = 55;
	engine->engineState.ignitionLoad = 77;

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, true);
	ASSERT_TRUE(inputs.FuelingLoad.Valid);
	ASSERT_TRUE(inputs.IgnitionLoad.Valid);
	EXPECT_FLOAT_EQ(inputs.FuelingLoad.Value, 55);
	EXPECT_FLOAT_EQ(inputs.IgnitionLoad.Value, 77);

	engine->engineState.fuelingLoad = 90;
	engine->engineState.ignitionLoad = 3;

	MockVp3d mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AirmassDiagnostics diagnostics;
	const auto ve =
			speedDensity.evaluateVe(inputs, inputs.NativeMap.Map, true, VeTableType::SpeedDensity, &diagnostics.Ve);
	ASSERT_TRUE(ve.Valid);
	EXPECT_NEAR(ve.Ve, 72, EPS4D);
	EXPECT_NEAR(engine->outputChannels.veBlendParameter[0], 55, EPS4D);
	EXPECT_NEAR(engine->outputChannels.veBlendYAxis[0], 77, EPS4D);
	EXPECT_NEAR(engine->outputChannels.veBlendOutput[0], 20, EPS4D);
}

TEST(AirmassEvaluation, RawBranchesDeferCommonCorrectionsUntilAfterCombination) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	setCylinderCount(4);
	setTable(config->veTable, 60);
	setTable(config->alphaNTable, 40);

	for (auto& blend : config->veBlends) {
		blend.blendParameter = GPPWM_Zero;
	}

	auto& sdCorrection = config->veBlends[0];
	sdCorrection.blendParameter = GPPWM_FuelLoad;
	sdCorrection.yAxisOverride = GPPWM_Zero;
	sdCorrection.veTableSelect = VeTableType::SpeedDensity;
	setTable(sdCorrection.table, 20);
	setArrayValues(sdCorrection.blendValues, 100);

	auto& alphaNCorrection = config->veBlends[1];
	alphaNCorrection.blendParameter = GPPWM_IgnLoad;
	alphaNCorrection.yAxisOverride = GPPWM_Zero;
	alphaNCorrection.veTableSelect = VeTableType::AlphaN;
	setTable(alphaNCorrection.table, -10);
	setArrayValues(alphaNCorrection.blendValues, 100);

	config->alphaNMultiplyMap = true;
	Sensor::setMockValue(SensorType::Map, 50);
	Sensor::setMockValue(SensorType::Tps1, 25);
	engine->engineState.fuelingLoad = 55;
	engine->engineState.ignitionLoad = 77;

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, true);
	// Supply captured consumers explicitly for this pure raw-evaluation test.
	inputs.FuelingLoad = 55;
	inputs.IgnitionLoad = 77;
	ASSERT_TRUE(inputs.FuelingLoad.Valid);
	ASSERT_TRUE(inputs.IgnitionLoad.Valid);

	engine->engineState.sd.tChargeK = 373.15f;
	engineConfiguration->displacement = 6.0f;
	setCylinderCount(8);
	Sensor::setMockValue(SensorType::Map, 90);
	Sensor::setMockValue(SensorType::Tps1, 90);
	engine->engineState.fuelingLoad = 0;
	engine->engineState.ignitionLoad = 0;

	MockVp3d mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AlphaNAirmass alphaN;
	RawAirmassDiagnostics sdDiagnostics;
	RawAirmassDiagnostics alphaNDiagnostics;
	const auto rawSpeedDensity = speedDensity.evaluateRawAirmass(inputs, &sdDiagnostics);
	const auto rawAlphaN = alphaN.evaluateRawAirmass(inputs, &alphaNDiagnostics);

	ASSERT_TRUE(rawSpeedDensity.Valid);
	ASSERT_TRUE(rawAlphaN.Valid);
	EXPECT_NEAR(rawSpeedDensity.Result.CylinderAirmass, expectedIdealGasMass(2.4f, 4, 60, 50, 293.15f), EPS4D);
	EXPECT_NEAR(rawAlphaN.Result.CylinderAirmass, expectedIdealGasMass(2.4f, 4, 40, 101.325f, 293.15f), EPS4D);
	EXPECT_FLOAT_EQ(sdDiagnostics.TableValue, 60);
	EXPECT_FLOAT_EQ(alphaNDiagnostics.TableValue, 40);
	EXPECT_FLOAT_EQ(alphaNDiagnostics.BaroCoefficient, 1);

	const auto effectiveMapAlphaN = alphaN.evaluateRawAirmass(inputs, nullptr, AlphaNPressurePolicy::EffectiveMap);
	ASSERT_TRUE(effectiveMapAlphaN.Valid);
	EXPECT_NEAR(effectiveMapAlphaN.Result.CylinderAirmass, expectedIdealGasMass(2.4f, 4, 40, 50, 293.15f), EPS4D);

	VeCorrectionDiagnostics correctionDiagnostics;
	const auto corrections = evaluateAirmassCorrections(inputs, &correctionDiagnostics);
	ASSERT_TRUE(corrections.Valid);
	EXPECT_FALSE(corrections.Fallback);
	EXPECT_NEAR(corrections.Multiplier, 1.08f, EPS4D);
	EXPECT_TRUE(correctionDiagnostics.HasValue);
	EXPECT_TRUE(correctionDiagnostics.Valid);
	EXPECT_NEAR(correctionDiagnostics.Blends[0].Value, 20, EPS4D);
	EXPECT_NEAR(correctionDiagnostics.Blends[1].Value, -10, EPS4D);

	const float rawCombined = (rawSpeedDensity.Result.CylinderAirmass + rawAlphaN.Result.CylinderAirmass) / 2;
	EXPECT_NEAR(rawCombined * corrections.Multiplier, rawCombined * 1.08f, EPS4D);

	// The compatibility wrappers retain their per-consumer filter and apply
	// exactly the correction assigned to each standalone model.
	AirmassDiagnostics sdStandaloneDiagnostics;
	const auto sdStandalone = speedDensity.evaluateAirmass(inputs, &sdStandaloneDiagnostics);
	ASSERT_TRUE(sdStandalone.Valid);
	EXPECT_NEAR(sdStandalone.Result.CylinderAirmass, rawSpeedDensity.Result.CylinderAirmass * 1.2f, EPS4D);

	AirmassDiagnostics alphaNStandaloneDiagnostics;
	const auto alphaNStandalone =
			alphaN.evaluateAirmass(inputs, AlphaNPressurePolicy::PureReference, &alphaNStandaloneDiagnostics);
	ASSERT_TRUE(alphaNStandalone.Valid);
	EXPECT_NEAR(alphaNStandalone.Result.CylinderAirmass, rawAlphaN.Result.CylinderAirmass * 0.9f, EPS4D);
}

TEST(AirmassEvaluation, TpsOvertravelClampsOnlyWithinFiniteOrderedLimits) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	engineConfiguration->tpsErrorDetectionTooLow = -10;
	engineConfiguration->tpsErrorDetectionTooHigh = 110;
	setTable(config->alphaNTable, 50);

	Sensor::setMockValue(SensorType::Tps1, -5);
	AlphaNAirmass alphaN;
	auto below = alphaN.evaluateAirmass(2000);
	ASSERT_TRUE(below.Valid);
	EXPECT_FLOAT_EQ(below.Result.EngineLoadPercent, 0);

	Sensor::setMockValue(SensorType::Tps1, 105);
	auto above = alphaN.evaluateAirmass(2000);
	ASSERT_TRUE(above.Valid);
	EXPECT_FLOAT_EQ(above.Result.EngineLoadPercent, 100);

	Sensor::setMockValue(SensorType::Tps1, 111);
	EXPECT_FALSE(alphaN.evaluateAirmass(2000).Valid);

	Sensor::setMockValue(SensorType::Tps1, 50);
	engineConfiguration->tpsErrorDetectionTooLow = 60;
	EXPECT_FALSE(alphaN.evaluateAirmass(2000).Valid);
}

TEST(AirmassCalibrationCache, ReusesKnownAxesWithoutRescanning) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityAxes();

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	const auto scans = engine->airmassCalibration.Scans;
	EXPECT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
	EXPECT_EQ(engine->airmassCalibration.Scans, scans + 1);
	EXPECT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
	EXPECT_EQ(engine->airmassCalibration.Scans, scans + 1);
}

TEST(AirmassCalibrationCache, ConfigurationVersionChangeRefreshesKnownAxes) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityAxes();

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	ASSERT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
	const auto scans = engine->airmassCalibration.Scans;

	engine->globalConfigurationVersion++;
	AirmassInputs nextVersion;
	captureAirmassInputs(2500, nextVersion, nullptr, false, true);
	EXPECT_TRUE(isCapturedAirmassModelValid(nextVersion, LM_SPEED_DENSITY));
	EXPECT_EQ(engine->airmassCalibration.Scans, scans + 1);
}

TEST(AirmassCalibrationCache, ExplicitInvalidationRejectsStaleGeneration) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityAxes();

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	ASSERT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
	const auto generation = engine->airmassCalibration.Generation;

	invalidateAirmassCalibration();
	EXPECT_EQ(engine->airmassCalibration.Generation, generation + 1);
	EXPECT_EQ(engine->airmassCalibration.Known, 0u);
	EXPECT_FALSE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));

	config->veLoadBins[1] = config->veLoadBins[0];
	AirmassInputs changed;
	captureAirmassInputs(2500, changed, nullptr, false, true);
	EXPECT_FALSE(isCapturedAirmassModelValid(changed, LM_SPEED_DENSITY));
}

TEST(AirmassCalibrationCache, MapEstimateScanIncludesEveryCellAndInvalidationRechecks) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	configureMapEstimateAxes();
	setTable(config->mapEstimateTable, 60);

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	ASSERT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::MapEstimate));

	const size_t lastLoad = efi::size(config->mapEstimateTable) - 1;
	const size_t lastRpm = efi::size(config->mapEstimateTable[0]) - 1;
	config->mapEstimateTable[lastLoad][lastRpm] = 601;
	EXPECT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::MapEstimate));

	invalidateAirmassCalibration();
	AirmassInputs invalidated;
	captureAirmassInputs(2500, invalidated, nullptr, false, true);
	EXPECT_FALSE(isCapturedAirmassCalibrationValid(invalidated, AirmassCalibration::MapEstimate));
	EXPECT_FALSE(isMapEstimateConfigurationValid());

	// A usable measured MAP does not require the optional estimate to be valid.
	configureSpeedDensityAxes();
	setTable(config->veTable, 60);
	config->useMapEstimateTable = true;
	Sensor::setMockValue(SensorType::Map, 54);
	MockVp3d mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	const auto measured = speedDensity.getAirmassForFuel(2500);
	ASSERT_TRUE(measured.Valid);
	EXPECT_FLOAT_EQ(measured.Result.EngineLoadPercent, 54);
}

TEST(AirmassCalibrationCache, ScanRaceCannotPopulateCache) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureSpeedDensityAxes();

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	const auto scans = engine->airmassCalibration.Scans;
	bool raced = false;
	engine->onAirmassCalibrationScanned = [&] {
		if (!raced) {
			raced = true;
			invalidateAirmassCalibration();
		}
	};

	EXPECT_FALSE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
	engine->onAirmassCalibrationScanned = nullptr;
	EXPECT_TRUE(raced);
	EXPECT_EQ(engine->airmassCalibration.Known, 0u);
	EXPECT_EQ(engine->airmassCalibration.Scans, scans);

	AirmassInputs retry;
	captureAirmassInputs(2500, retry, nullptr, false, true);
	EXPECT_TRUE(isCapturedAirmassModelValid(retry, LM_SPEED_DENSITY));
	EXPECT_TRUE(isCapturedAirmassModelValid(retry, LM_SPEED_DENSITY));
	EXPECT_EQ(engine->airmassCalibration.Scans, scans + 1);
}

TEST(AirmassCalibrationCache, DryQueriesDoNotMutateCacheOrLiveOutputs) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engine->engineState.currentVe = 91;
	engine->engineState.veTableYAxis = 92;
	engine->engineState.idleVeTableYAxis = 93;
	Sensor::setMockValue(SensorType::Map, 54);
	Sensor::setMockValue(SensorType::Tps1, 20);
	setTable(config->veTable, 60);
	configureSpeedDensityAxes();

	const auto generation = engine->airmassCalibration.Generation;
	const auto known = engine->airmassCalibration.Known;
	const auto scans = engine->airmassCalibration.Scans;
	StrictMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AirmassDiagnostics diagnostics;
	EXPECT_TRUE(speedDensity.evaluateAirmass(2500, &diagnostics).Valid);

	EXPECT_EQ(engine->airmassCalibration.Generation, generation);
	EXPECT_EQ(engine->airmassCalibration.Known, known);
	EXPECT_EQ(engine->airmassCalibration.Scans, scans);
	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 91);
	EXPECT_FLOAT_EQ(engine->engineState.veTableYAxis, 92);
	EXPECT_FLOAT_EQ(engine->engineState.idleVeTableYAxis, 93);
}

TEST(AirmassCalibrationCache, DryQueriesBypassAnExistingLiveCacheEntry) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	setTable(config->veTable, 60);
	configureSpeedDensityAxes();
	Sensor::setMockValue(SensorType::Map, 54);

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	ASSERT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));

	config->veLoadBins[1] = config->veLoadBins[0];
	StrictMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AirmassDiagnostics diagnostics;
	EXPECT_FALSE(speedDensity.evaluateAirmass(2500, &diagnostics).Valid);

	// The captured live generation intentionally remains reusable until a write
	// path explicitly invalidates it.
	EXPECT_TRUE(isCapturedAirmassModelValid(inputs, LM_SPEED_DENSITY));
}

TEST(AirmassEvaluation, SdCanUseMeasuredMapWhenTpsSensorIsInvalid) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	setTable(config->veTable, 60);
	configureSpeedDensityAxes();
	Sensor::setMockValue(SensorType::Map, 54);
	Sensor::setInvalidMockValue(SensorType::Tps1);

	AirmassInputs inputs;
	MockVp3d mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	captureAirmassInputs(2500, inputs, nullptr, true);
	EXPECT_FALSE(inputs.Tps.Valid);
	EXPECT_TRUE(inputs.NativeMap.Valid);
	EXPECT_TRUE(speedDensity.evaluateAirmass(inputs).Valid);
}

TEST(AirmassCalibrationCache, IdleAndCorrectionBitsInvalidateIndependently) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setLinearCurve(config->idleVeLoadBins, 0, 100, 1);
	setLinearCurve(config->idleVeRpmBins, 500, 2500, 1);
	auto& correction = config->veBlends[0];
	setLinearCurve(correction.loadBins, 0, 140, 1);
	setLinearCurve(correction.rpmBins, 500, 8000, 1);
	setLinearCurve(correction.blendBins, 0, 70, 1);

	AirmassInputs inputs;
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	ASSERT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Idle));
	ASSERT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Correction0));
	const auto scans = engine->airmassCalibration.Scans;
	EXPECT_EQ(scans, 2u);
	EXPECT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Idle));
	EXPECT_EQ(engine->airmassCalibration.Scans, scans);

	invalidateAirmassCalibration();
	config->idleVeLoadBins[1] = config->idleVeLoadBins[0];
	captureAirmassInputs(2500, inputs, nullptr, false, true);
	EXPECT_FALSE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Idle));
	EXPECT_TRUE(isCapturedAirmassCalibrationValid(inputs, AirmassCalibration::Correction0));
}

TEST(AirmassEvaluation, MafRejectsNonfiniteAndNegativeInputsBeforeTrim) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->fuelAlgorithm = LM_REAL_MAF;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	configureMafTrimAxes();
	Sensor::setMockValue(SensorType::Maf, 72);

	MafAirmass maf;
	const auto valid = maf.getAirmassForFuel(3000);
	ASSERT_TRUE(valid.Valid);
	EXPECT_GT(valid.Result.CylinderAirmass, 0);
	EXPECT_LT(valid.Result.EngineLoadPercent, 1000);

	AirmassInputs inputs;
	captureAirmassInputs(3000, inputs, nullptr, false);
	for (float invalid : {NAN, INFINITY, -1.0f}) {
		EXPECT_FALSE(maf.evaluateAirmassImpl(invalid, inputs).Valid);
	}
	inputs.Rpm = NAN;
	EXPECT_FALSE(maf.evaluateAirmassImpl(72, inputs).Valid);
	inputs.Rpm = 3000;
	inputs.Displacement = NAN;
	EXPECT_FALSE(maf.evaluateAirmassImpl(72, inputs).Valid);

	config->mafTrimLoadBins[1] = config->mafTrimLoadBins[0];
	EXPECT_FALSE(maf.getAirmassImpl(72, 3000).Valid);
	invalidateAirmassCalibration();
	EXPECT_FALSE(maf.getAirmassForFuel(3000).Valid);
}

TEST(AirmassEvaluation, MafUsesTrimEdgeForHighLoad) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->fuelAlgorithm = LM_REAL_MAF;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	configureMafTrimAxes();

	auto& edgeRow = config->mafTrimTable[efi::size(config->mafTrimTable) - 1];
	setArrayValues(edgeRow, 110);
	Sensor::setMockValue(SensorType::Maf, 3600);

	MafAirmass maf;
	const auto evaluation = maf.getAirmassForFuel(3000);
	ASSERT_TRUE(evaluation.Valid);
	EXPECT_GT(evaluation.Result.EngineLoadPercent, 1000);
	EXPECT_NEAR(evaluation.Result.CylinderAirmass, 11.0f, EPS4D);
}
