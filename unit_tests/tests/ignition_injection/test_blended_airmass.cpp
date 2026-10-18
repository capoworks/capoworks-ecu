#include "pch.h"

#include "airmass_calibration.h"
#include "alphan_airmass.h"
#include "airmass_loads.h"
#include "blended_airmass.h"
#include "fuel_math.h"
#include "speed_density_airmass.h"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrictMock;

namespace {
constexpr float AirGasConstant = 0.28705f;

mass_t expectedIdealGasMass(float displacement, int cylinders, float vePercent, float pressure, float temperatureK) {
	return (vePercent * 0.01f) * displacement * pressure / (AirGasConstant * temperatureK) / cylinders;
}

void configureAuthorityMapAxis() {
	copyArray(config->airmassBlendLoadBins, {0.0f, 20.0f, 40.0f, 80.0f, 110.0f, 150.0f, 200.0f, 250.0f});
}

void configureFlatCorrection(blend_table_s& blend, float correctionPercent) {
	blend.blendParameter = GPPWM_Tps;
	blend.yAxisOverride = GPPWM_Zero;
	blend.veTableSelect = VeTableType::SpeedDensity;
	setLinearCurve(blend.loadBins, 0, 200, 1);
	setLinearCurve(blend.rpmBins, 0, 8000, 1);
	setTable(blend.table, correctionPercent);
	setLinearCurve(blend.blendBins, 0, 100, 1);
	setArrayValues(blend.blendValues, 100);
}

void expectAirmassParity(
		const BlendedAirmassEvaluation& withoutDiagnostics, const BlendedAirmassEvaluation& withDiagnostics) {
	EXPECT_FLOAT_EQ(withoutDiagnostics.Airmass.Result.CylinderAirmass, withDiagnostics.Airmass.Result.CylinderAirmass);
	EXPECT_FLOAT_EQ(
			withoutDiagnostics.Airmass.Result.EngineLoadPercent, withDiagnostics.Airmass.Result.EngineLoadPercent);
	EXPECT_EQ(withoutDiagnostics.Airmass.Valid, withDiagnostics.Airmass.Valid);
	EXPECT_EQ(withoutDiagnostics.Airmass.Degraded, withDiagnostics.Airmass.Degraded);
	EXPECT_EQ(withoutDiagnostics.Fault, withDiagnostics.Fault);
}

class BlendedAirmassModelTest : public ::testing::Test {
protected:
	BlendedAirmassModelTest()
		: eth(engine_type_e::TEST_ENGINE) {
		engineConfiguration->fuelAlgorithm = LM_SD_ALPHA_N;
		engineConfiguration->displacement = 2.4f;
		engineConfiguration->useSeparateVeForIdle = false;
		setCylinderCount(4);
		engine->engineState.sd.tChargeK = 300;

		setLinearCurve(config->veLoadBins, 0, 200, 1);
		setLinearCurve(config->veRpmBins, 500, 9000, 1);
		setTable(config->veTable, 60);
		setLinearCurve(config->alphaNTpsBins, 0, 100, 1);
		setLinearCurve(config->alphaNRpmBins, 500, 9000, 1);
		setTable(config->alphaNTable, 40);

		Sensor::setMockValue(SensorType::Map, 50);
		Sensor::setMockValue(SensorType::Tps1, 20);
		Sensor::setMockValue(SensorType::Iat, 27);
	}

	AirmassInputs captureInputs(BlendedAirmass& blended) {
		AirmassInputs inputs;
		blended.captureInputs(2500, inputs);
		return inputs;
	}

	EngineTestHelper eth;
	NiceMock<MockVp3d> mapEstimate;
};
} // namespace

TEST(BlendedAirmassRuntimeOwnership, EngineConstructionDoesNotRequireInstalledGlobalOwner) {
	ASSERT_EQ(nullptr, engine);

	Engine local;

	EXPECT_EQ(AirmassInjectionStatus::Legacy, local.airmassInjectionState.status());
	EXPECT_EQ(AirmassInjectionFault::None, local.airmassInjectionState.fault());
	EXPECT_EQ(static_cast<uint8_t>(AirmassInjectionStatus::Legacy), local.outputChannels.blendedStatus);
	EXPECT_EQ(static_cast<uint8_t>(AirmassInjectionFault::None), local.outputChannels.blendedFault);
	EXPECT_EQ(0, local.outputChannels.blendedVeAnalyzeEndpoint);
}

TEST_F(BlendedAirmassModelTest, DefaultAuthorityAxisIsTpsAndEvaluationUsesCapturedSnapshot) {
	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	ASSERT_EQ(config->airmassBlendTable[0][0], 0);
	setTable(config->airmassBlendTable, 25);

	ASSERT_EQ(config->airmassBlendLoadSource, AirmassBlendLoadSource::Tps);
	auto inputs = captureInputs(blended);
	ASSERT_EQ(inputs.BlendLoadSource, AirmassBlendLoadSource::Tps);
	ASSERT_TRUE(inputs.HasPublicationContext);

	// Pure evaluation must use the caller snapshot, not sample these sensors again.
	Sensor::setMockValue(SensorType::Map, 180);
	Sensor::setMockValue(SensorType::Tps1, 95);
	engine->outputChannels.airmassPressureFlags = 77;
	engine->engineState.currentVe = 91;
	engine->engineState.veTableYAxis = 92;

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	const float sdMass = expectedIdealGasMass(2.4f, 4, 60, 50, 300);
	const float alphaNMass = expectedIdealGasMass(2.4f, 4, 40, 101.325f, 300);

	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 25);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 25);
	EXPECT_FLOAT_EQ(diagnostics.SdWeight, 0.75f);
	EXPECT_FLOAT_EQ(diagnostics.AlphaNWeight, 0.25f);
	EXPECT_NEAR(result.Airmass.Result.CylinderAirmass, 0.75f * sdMass + 0.25f * alphaNMass, EPS4D);
	EXPECT_FLOAT_EQ(result.Airmass.Result.EngineLoadPercent, 50);
	EXPECT_FLOAT_EQ(result.NativeLoad.Value, 50);
	EXPECT_EQ(result.NativeLoad.Source, AirmassLoadSource::EffectiveMap);
	EXPECT_FLOAT_EQ(diagnostics.Inputs.Tps.Value, 20);
	EXPECT_FLOAT_EQ(diagnostics.Inputs.MeasuredMap.Value, 50);
	EXPECT_TRUE(diagnostics.Flags & BlendedSdEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedAlphaNEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedCalculationValid);

	EXPECT_EQ(engine->outputChannels.airmassPressureFlags, 77);
	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 91);
	EXPECT_FLOAT_EQ(engine->engineState.veTableYAxis, 92);
}

TEST_F(BlendedAirmassModelTest, MeasuredMapAuthorityUsesMapAxisIndependentlyOfTps) {
	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 0);
	for (size_t row = 0; row < AIRMASS_BLEND_LOAD_COUNT; row++) {
		setArrayValues(config->airmassBlendTable[row], std::min(10.0f + row * 20.0f, 100.0f));
	}
	setArrayValues(config->airmassBlendTable[3], 61.0f);
	// At 60 kPa interpolation gives 55.5%; TPS 20 would have requested 10%.
	Sensor::setMockValue(SensorType::Map, 60);
	Sensor::setMockValue(SensorType::Tps1, 20);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);
	Sensor::setMockValue(SensorType::Map, 220);
	Sensor::setMockValue(SensorType::Tps1, 95);

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_FLOAT_EQ(diagnostics.AuthorityLoad, 60);
	EXPECT_TRUE(diagnostics.AuthorityLoadValid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 55.5f);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 55.5f);
	EXPECT_EQ(diagnostics.AuthorityLoadSource, AirmassBlendLoadSource::MeasuredMap);
	EXPECT_TRUE(diagnostics.Flags & BlendedAuthorityLoadValid);
	EXPECT_TRUE(diagnostics.Flags & BlendedSdEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedAlphaNEvaluated);
}

TEST_F(BlendedAirmassModelTest, MissingNanAndOutOfRangeMeasuredMapCannotAuthorizeBlend) {
	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 50);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	const auto captured = captureInputs(blended);

	const expected<float> invalidMaps[] = {
			expected<float>(unexpected),
			expected<float>(std::numeric_limits<float>::quiet_NaN()),
			expected<float>(-1),
			expected<float>(1001),
	};

	for (const auto& invalidMap : invalidMaps) {
		auto inputs = captured;
		inputs.MeasuredMap = invalidMap;
		// Make Speed Density unavailable so the healthy Alpha-N branch proves
		// authority fallback without inventing a pressure coordinate.
		inputs.NativeMap = {};

		BlendedAirmassDiagnostics diagnostics;
		const auto result = blended.evaluateAirmass(inputs, &diagnostics);
		ASSERT_TRUE(result.Airmass.Valid);
		EXPECT_FALSE(diagnostics.AuthorityLoadValid);
		EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 0);
		EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 100);
		EXPECT_TRUE(diagnostics.Flags & BlendedAuthorityUnavailable);
		EXPECT_TRUE(diagnostics.Flags & BlendedBranchFallback);
		EXPECT_TRUE(diagnostics.Flags & BlendedAlphaNValid);
		EXPECT_EQ(result.Fault, AirmassInjectionFault::Sensor);
	}
}

TEST_F(BlendedAirmassModelTest, MapEstimateCanRunSdButNeverQualifiesMeasuredMapAuthority) {
	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 50);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);
	inputs.MeasuredMap = unexpected;
	inputs.NativeMap = {70, 70, true, true, true, true};
	inputs.LambdaOverride = AFR_MAP;

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_FALSE(diagnostics.AuthorityLoadValid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 0);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 0);
	EXPECT_TRUE(diagnostics.Flags & BlendedAuthorityUnavailable);
	EXPECT_TRUE(diagnostics.Flags & BlendedEstimateEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedMapEstimateUsed);
	EXPECT_TRUE(diagnostics.Flags & BlendedSdValid);
	EXPECT_FALSE(diagnostics.Flags & BlendedAlphaNEvaluated);
	EXPECT_FLOAT_EQ(result.Airmass.Result.EngineLoadPercent, 70);
	EXPECT_TRUE(result.NativeLoad.UsesEstimate);
	EXPECT_FALSE(result.LambdaLoadStrict.Valid);
	EXPECT_TRUE(result.LambdaLoad.Valid);
	EXPECT_FLOAT_EQ(result.LambdaLoad.Value, 200);
	EXPECT_EQ(result.Fault, AirmassInjectionFault::Sensor);
}

TEST_F(BlendedAirmassModelTest, InvalidAuthorityConfigurationFallsBackOnlyToHealthyBranch) {
	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 50);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);
	config->airmassBlendLoadBins[2] = config->airmassBlendLoadBins[1];

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_FALSE(isBlendedAirmassConfigurationValid());
	EXPECT_FALSE(diagnostics.AuthorityLoadValid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 0);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 0);
	EXPECT_TRUE(diagnostics.Flags & BlendedAuthorityUnavailable);
	EXPECT_TRUE(diagnostics.Flags & BlendedBranchFallback);
	EXPECT_TRUE(diagnostics.Flags & BlendedSdValid);
	EXPECT_FALSE(diagnostics.Flags & BlendedAlphaNEvaluated);
	EXPECT_EQ(result.Fault, AirmassInjectionFault::Configuration);
}

TEST_F(BlendedAirmassModelTest, InvalidRequiredBranchUsesHealthyAlternateWithoutMixingItsFailure) {
	setTable(config->airmassBlendTable, 50);
	StrictMock<MockVp3d> invalidVe;
	EXPECT_CALL(invalidVe, getValue(2500, 50)).WillOnce(Return(std::numeric_limits<float>::quiet_NaN()));
	SpeedDensityAirmass sd(&invalidVe, mapEstimate);
	AlphaNAirmass alphaN;
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	ASSERT_TRUE(result.Airmass.Valid);
	EXPECT_TRUE(result.Airmass.Degraded);
	EXPECT_EQ(result.Fault, AirmassInjectionFault::Sensor);
	EXPECT_FALSE(diagnostics.Sd.HasValue);
	EXPECT_FALSE(diagnostics.Sd.Valid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 50);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 100);
	EXPECT_FLOAT_EQ(diagnostics.SdWeight, 0);
	EXPECT_FLOAT_EQ(diagnostics.AlphaNWeight, 1);
	EXPECT_TRUE(diagnostics.Flags & BlendedBranchFallback);
	EXPECT_NEAR(result.Airmass.Result.CylinderAirmass, expectedIdealGasMass(2.4f, 4, 40, 101.325f, 300), EPS4D);
}

TEST_F(BlendedAirmassModelTest, BothInvalidBranchesRejectCompositeEvaluation) {
	setTable(config->airmassBlendTable, 50);
	StrictMock<MockVp3d> invalidVe;
	StrictMock<MockVp3d> invalidAlphaNVe;
	EXPECT_CALL(invalidVe, getValue(2500, 50)).WillOnce(Return(std::numeric_limits<float>::quiet_NaN()));
	EXPECT_CALL(invalidAlphaNVe, getValue(2500, 20)).WillOnce(Return(std::numeric_limits<float>::quiet_NaN()));
	SpeedDensityAirmass sd(&invalidVe, mapEstimate);
	AlphaNAirmass alphaN(&invalidAlphaNVe);
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);

	BlendedAirmassDiagnostics diagnostics;
	const auto result = blended.evaluateAirmass(inputs, &diagnostics);
	EXPECT_FALSE(result.Airmass.Valid);
	EXPECT_EQ(result.Fault, AirmassInjectionFault::Sensor);
	EXPECT_FALSE(diagnostics.Flags & BlendedCalculationValid);
	EXPECT_TRUE(diagnostics.Flags & BlendedSdEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedAlphaNEvaluated);
	EXPECT_FALSE(diagnostics.Flags & BlendedSdValid);
	EXPECT_FALSE(diagnostics.Flags & BlendedAlphaNValid);
}

TEST_F(BlendedAirmassModelTest, NoDiagnosticsMatchesCapturedResultsForTemperatureFallback) {
	setTable(config->airmassBlendTable, 50);
	config->airmassTemperatureSource = AirmassTemperatureSource::Tcharge;
	engine->engineState.sd.tChargeK = std::numeric_limits<float>::quiet_NaN();

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	const auto inputs = captureInputs(blended);
	ASSERT_TRUE(inputs.TemperatureValid);
	ASSERT_TRUE(inputs.TemperatureFallback);

	const auto withoutDiagnostics = blended.evaluateAirmass(inputs);
	BlendedAirmassDiagnostics diagnostics;
	const auto withDiagnostics = blended.evaluateAirmass(inputs, &diagnostics);

	expectAirmassParity(withoutDiagnostics, withDiagnostics);
	EXPECT_TRUE(withoutDiagnostics.Airmass.Degraded);
	EXPECT_EQ(withoutDiagnostics.Fault, AirmassInjectionFault::Sensor);
	EXPECT_TRUE(diagnostics.Flags & BlendedTemperatureFallback);
}

TEST_F(BlendedAirmassModelTest, PureAlphaNAuthorityIgnoresInvalidUnusedMap) {
	setTable(config->airmassBlendTable, 100);
	config->useMapEstimateTable = false;
	Sensor::setInvalidMockValue(SensorType::Map);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	const auto inputs = captureInputs(blended);
	ASSERT_TRUE(inputs.Tps.Valid);
	ASSERT_FALSE(inputs.NativeMap.Valid);
	ASSERT_FALSE(inputs.NativeMap.HasValue);
	ASSERT_TRUE(inputs.NativeMap.Fallback);

	const auto withoutDiagnostics = blended.evaluateAirmass(inputs);
	BlendedAirmassDiagnostics diagnostics;
	const auto withDiagnostics = blended.evaluateAirmass(inputs, &diagnostics);

	expectAirmassParity(withoutDiagnostics, withDiagnostics);
	EXPECT_TRUE(withoutDiagnostics.Airmass.Valid);
	EXPECT_FALSE(withoutDiagnostics.Airmass.Degraded);
	EXPECT_EQ(withoutDiagnostics.Fault, AirmassInjectionFault::None);
	EXPECT_TRUE(diagnostics.Map.Fallback);
	EXPECT_FALSE(diagnostics.Map.Valid);
	EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, 100);
	EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, 100);
	EXPECT_TRUE(diagnostics.Flags & BlendedAlphaNValid);
	EXPECT_FALSE(diagnostics.Flags & BlendedSdEvaluated);
	EXPECT_FALSE(diagnostics.Flags & BlendedBranchFallback);
	EXPECT_FALSE(diagnostics.Flags & BlendedMapFallback);
}

TEST_F(BlendedAirmassModelTest, NoDiagnosticsMatchesCapturedResultsForMapEstimateFallback) {
	setTable(config->airmassBlendTable, 50);
	config->useMapEstimateTable = true;
	Sensor::setInvalidMockValue(SensorType::Map);
	EXPECT_CALL(mapEstimate, getValue(2500, 20)).WillOnce(Return(70));

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	const auto inputs = captureInputs(blended);
	ASSERT_TRUE(inputs.NativeMap.HasValue);
	ASSERT_TRUE(inputs.NativeMap.Valid);
	ASSERT_TRUE(inputs.NativeMap.UsesEstimate);
	ASSERT_TRUE(inputs.NativeMap.Fallback);

	const auto withoutDiagnostics = blended.evaluateAirmass(inputs);
	BlendedAirmassDiagnostics diagnostics;
	const auto withDiagnostics = blended.evaluateAirmass(inputs, &diagnostics);

	expectAirmassParity(withoutDiagnostics, withDiagnostics);
	EXPECT_TRUE(withoutDiagnostics.Airmass.Degraded);
	EXPECT_EQ(withoutDiagnostics.Fault, AirmassInjectionFault::Sensor);
	EXPECT_TRUE(diagnostics.Flags & BlendedEstimateEvaluated);
	EXPECT_TRUE(diagnostics.Flags & BlendedMapEstimateUsed);
	EXPECT_TRUE(diagnostics.Flags & BlendedMapFallback);
}

TEST_F(BlendedAirmassModelTest, InvalidWrapperReturnsUnexpectedWithoutPublishing) {
	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	engine->outputChannels.airmassPressureFlags = 77;

	const auto result = blended.getAirmass(0, false);

	EXPECT_FALSE(result);
	EXPECT_EQ(engine->outputChannels.airmassPressureFlags, 77);
}

TEST_F(BlendedAirmassModelTest, EndpointsAndInteriorWeightsApplyCommonCorrectionOnce) {
	for (auto& blend : config->veBlends) {
		blend.blendParameter = GPPWM_Zero;
	}
	configureFlatCorrection(config->veBlends[0], 20);

	AlphaNAirmass alphaN;
	SpeedDensityAirmass sd(nullptr, mapEstimate);
	BlendedAirmass blended(sd, alphaN);
	auto inputs = captureInputs(blended);
	const float sdMass = expectedIdealGasMass(2.4f, 4, 60, 50, 300);
	const float alphaNMass = expectedIdealGasMass(2.4f, 4, 40, 101.325f, 300);

	for (float authority : {0.0f, 50.0f, 100.0f}) {
		setTable(config->airmassBlendTable, authority);
		BlendedAirmassDiagnostics diagnostics;
		const auto result = blended.evaluateAirmass(inputs, &diagnostics);
		const float weight = authority * 0.01f;
		const float rawMass = authority == 0   ? sdMass
							: authority == 100 ? alphaNMass
											   : (1 - weight) * sdMass + weight * alphaNMass;

		ASSERT_TRUE(result.Airmass.Valid) << authority;
		EXPECT_EQ(result.Fault, AirmassInjectionFault::None);
		EXPECT_FLOAT_EQ(diagnostics.RequestedAuthority, authority);
		EXPECT_FLOAT_EQ(diagnostics.EffectiveAuthority, authority);
		EXPECT_FLOAT_EQ(diagnostics.SdWeight, authority < 100 ? 1 - weight : 0);
		EXPECT_FLOAT_EQ(diagnostics.AlphaNWeight, authority > 0 ? weight : 0);
		EXPECT_NEAR(diagnostics.RawMass, rawMass, EPS4D);
		EXPECT_FLOAT_EQ(diagnostics.CorrectionMultiplier, 1.2f);
		EXPECT_NEAR(diagnostics.FinalMass, rawMass * 1.2f, EPS4D);
		EXPECT_NEAR(result.Airmass.Result.CylinderAirmass, rawMass * 1.2f, EPS4D);
		EXPECT_TRUE(diagnostics.Corrections.HasValue);
		EXPECT_TRUE(diagnostics.Corrections.Valid);
		EXPECT_FLOAT_EQ(diagnostics.Corrections.Blends[0].Value, 20);
		EXPECT_EQ((diagnostics.Flags & BlendedSdEvaluated) != 0, authority < 100);
		EXPECT_EQ((diagnostics.Flags & BlendedAlphaNEvaluated) != 0, authority > 0);
		// The native blended load is effective MAP at every authority.
		EXPECT_FLOAT_EQ(result.Airmass.Result.EngineLoadPercent, 50);
		EXPECT_FLOAT_EQ(result.NativeLoad.Value, 50);
	}
}

TEST_F(BlendedAirmassModelTest, RuntimePublishesCompositeThroughFuelAdmission) {
	setTable(config->airmassBlendTable, 50);
	Sensor::setMockValue(SensorType::Rpm, 2500);

	engine->periodicFastCallback();

	ASSERT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_EQ(engine->airmassInjectionState.status(), AirmassInjectionStatus::Ready);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_EQ(engine->outputChannels.blendedStatus, static_cast<uint8_t>(AirmassInjectionStatus::Ready));
	EXPECT_EQ(engine->outputChannels.blendedFault, static_cast<uint8_t>(AirmassInjectionFault::None));

	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAuthorityLoad, 20);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedRequestedAuthority, 50);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 50);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedSdLoad, 50);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAlphaNLoad, 20);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedSdVe, 60);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAlphaNVe, 40);
	EXPECT_TRUE(engine->outputChannels.blendedFlags & BlendedCalculationValid);

	EXPECT_TRUE(engine->engineState.airmassLoads.Valid);
	EXPECT_FLOAT_EQ(engine->engineState.airmassLoads.Native, 50);
	EXPECT_FLOAT_EQ(getAirmassSelectedLoad(AFR_None, 0), 50);
	ASSERT_TRUE(getEffectiveAirmassMap());
	EXPECT_FLOAT_EQ(getEffectiveAirmassMap().Value, 50);
}

TEST_F(BlendedAirmassModelTest, MeasuredMapAuthorityPublishesCapturedYCoordinate) {
	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 0);
	setArrayValues(config->airmassBlendTable[2], 50);
	setArrayValues(config->airmassBlendTable[3], 70);
	Sensor::setMockValue(SensorType::Map, 60);
	Sensor::setMockValue(SensorType::Rpm, 2500);

	engine->periodicFastCallback();

	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAuthorityLoad, 60);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedRequestedAuthority, 60);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 60);
	EXPECT_TRUE(engine->outputChannels.blendedFlags & BlendedAuthorityLoadValid);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());

	// The editor cursor belongs to the published calculation, not the live MAP.
	Sensor::setMockValue(SensorType::Map, 220);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAuthorityLoad, 60);
}

TEST_F(BlendedAirmassModelTest, HealthyAlphaNFallbackUsesNativeSubstituteWithoutFakeMap) {
	setTable(config->airmassBlendTable, 100);
	config->useMapEstimateTable = false;
	Sensor::setInvalidMockValue(SensorType::Map);
	Sensor::setMockValue(SensorType::Rpm, 2500);

	engine->periodicFastCallback();

	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_TRUE(engine->engineState.airmassCalculationValid);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_EQ(engine->airmassInjectionState.status(), AirmassInjectionStatus::Degraded);
	EXPECT_EQ(engine->outputChannels.blendedStatus, static_cast<uint8_t>(AirmassInjectionStatus::Degraded));
	EXPECT_EQ(engine->outputChannels.blendedFault, static_cast<uint8_t>(AirmassInjectionFault::Load));
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 100);
	EXPECT_FLOAT_EQ(getAirmassSelectedLoad(AFR_None, 0), 200);
	EXPECT_FLOAT_EQ(engine->engineState.fuelingLoad, 200);
	EXPECT_FALSE(engine->engineState.airmassLoads.ValidSources & (1u << AFR_None));
	EXPECT_FALSE(engine->engineState.airmassLoads.ValidSources & (1u << AFR_MAP));
	EXPECT_FALSE(engine->engineState.airmassLoads.ValidSources & (1u << AFR_EffectiveMAP));
	EXPECT_FALSE(getEffectiveAirmassMap().Valid);

	Sensor::setMockValue(SensorType::Map, 80);
	engine->periodicFastCallback();
	EXPECT_EQ(engine->airmassInjectionState.status(), AirmassInjectionStatus::Ready);
	EXPECT_EQ(engine->airmassInjectionState.fault(), AirmassInjectionFault::None);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 100);
	EXPECT_FLOAT_EQ(getAirmassSelectedLoad(AFR_None, 0), 80);
	ASSERT_TRUE(getEffectiveAirmassMap());
	EXPECT_FLOAT_EQ(getEffectiveAirmassMap().Value, 80);
}

TEST_F(BlendedAirmassModelTest, BothBranchFailureClosesTheExistingFuelGate) {
	setTable(config->airmassBlendTable, 50);
	config->useMapEstimateTable = false;
	Sensor::setInvalidMockValue(SensorType::Map);
	Sensor::setInvalidMockValue(SensorType::Tps1);
	Sensor::setMockValue(SensorType::Rpm, 2500);

	engine->periodicFastCallback();

	EXPECT_FALSE(engine->engineState.airmassCalculationValid);
	EXPECT_FLOAT_EQ(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_EQ(engine->airmassInjectionState.status(), AirmassInjectionStatus::Faulted);
	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedSdMass, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAuthorityLoad, 0);
}

TEST_F(BlendedAirmassModelTest, StalePreparedBlendCannotReplaceCurrentPublication) {
	setTable(config->airmassBlendTable, 50);
	Sensor::setMockValue(SensorType::Rpm, 2500);
	engine->periodicFastCallback();
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
	const float publishedMass = engine->cylinders[0].getInjectionMass();
	const float publishedAuthority = engine->outputChannels.blendedRequestedAuthority;
	ASSERT_GT(publishedMass, 0);

	invalidateAirmassCalibration();
	int scanCallbacks = 0;
	engine->onAirmassCalibrationScanned = [&] {
		if (scanCallbacks++ != 0) {
			return;
		}
		engine->airmassInjectionState.beginCalculation(LM_SD_ALPHA_N, 2500, engine->getGlobalConfigurationVersion());
	};
	engine->periodicFastCallback();
	engine->onAirmassCalibrationScanned = nullptr;

	EXPECT_GE(scanCallbacks, 1);
	EXPECT_FLOAT_EQ(engine->cylinders[0].getInjectionMass(), publishedMass);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedRequestedAuthority, publishedAuthority);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
}

TEST_F(BlendedAirmassModelTest, ConfigurationWriteDuringPreparationCannotPublishStaleBlend) {
	setTable(config->airmassBlendTable, 50);
	Sensor::setMockValue(SensorType::Rpm, 2500);
	engine->periodicFastCallback();
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
	const float publishedMass = engine->cylinders[0].getInjectionMass();
	ASSERT_GT(publishedMass, 0);

	invalidateAirmassCalibration();
	int scanCallbacks = 0;
	engine->onAirmassCalibrationScanned = [&] {
		if (scanCallbacks++ != 0) {
			return;
		}
		++engine->globalConfigurationVersion;
		engine->airmassInjectionState.onConfigurationWrite(LM_SD_ALPHA_N, false);
	};
	engine->periodicFastCallback();
	engine->onAirmassCalibrationScanned = nullptr;

	EXPECT_GE(scanCallbacks, 1);
	EXPECT_FLOAT_EQ(engine->cylinders[0].getInjectionMass(), publishedMass);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedSdMass, 0);
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedAuthorityLoad, 0);
	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());

	engine->periodicFastCallback();
	EXPECT_GT(engine->cylinders[0].getInjectionMass(), 0);
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
}

TEST_F(BlendedAirmassModelTest, VeAnalyzeEndpointIsQualifiedAndPermanentlyInvalidatedPerSession) {
	config->useMapEstimateTable = false;
	Sensor::setMockValue(SensorType::Rpm, 2500);
	setTable(config->airmassBlendTable, 0);
	engine->periodicFastCallback();
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 1);

	config->airmassBlendTable[3][4] = 50;
	engine->airmassInjectionState.onConfigurationWrite(LM_SD_ALPHA_N, false);
	engine->periodicFastCallback();
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 0);

	setTable(config->airmassBlendTable, 0);
	engine->airmassInjectionState.onConfigurationWrite(LM_SD_ALPHA_N, false);
	engine->periodicFastCallback();
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 0);

	engine->OnTriggerSynchronizationLost();
	engine->periodicFastCallback();
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 1);

	engine->OnTriggerSynchronizationLost();
	setTable(config->airmassBlendTable, 100);
	engine->airmassInjectionState.onConfigurationWrite(LM_SD_ALPHA_N, false);
	engine->periodicFastCallback();
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 2);

	config->airmassBlendLoadSource = AirmassBlendLoadSource::MeasuredMap;
	configureAuthorityMapAxis();
	setTable(config->airmassBlendTable, 0);
	Sensor::setInvalidMockValue(SensorType::Map);
	engine->OnTriggerSynchronizationLost();
	engine->airmassInjectionState.onConfigurationWrite(LM_SD_ALPHA_N, false);
	engine->periodicFastCallback();
	EXPECT_TRUE(engine->airmassInjectionState.allowInjection());
	EXPECT_FLOAT_EQ(engine->outputChannels.blendedEffectiveAuthority, 100);
	EXPECT_EQ(engine->outputChannels.blendedVeAnalyzeEndpoint, 0);
}

TEST_F(BlendedAirmassModelTest, OwnerEngineStopClearsRuntimeStateAndPreservesPendingCallbacks) {
	setTable(config->airmassBlendTable, 50);
	Sensor::setMockValue(SensorType::Rpm, 2500);
	engine->periodicFastCallback();
	ASSERT_TRUE(engine->engineState.veAnalyzeSessionStarted);
	ASSERT_EQ(AirmassInjectionStatus::Ready, engine->airmassInjectionState.status());

	{
		chibios_rt::CriticalSectionLocker csl;
		ASSERT_TRUE(engine->airmassInjectionState.locked(csl).callbacksAccepted(2));
	}

	engine->OnTriggerSynchronizationLost();

	EXPECT_FALSE(engine->engineState.veAnalyzeSessionStarted);
	EXPECT_FALSE(engine->engineState.veAnalyzeSessionInvalid);
	EXPECT_EQ(0, engine->engineState.veAnalyzeEndpoint);
	EXPECT_EQ(0, engine->outputChannels.blendedVeAnalyzeEndpoint);
	EXPECT_EQ(static_cast<uint8_t>(AirmassInjectionStatus::NotReady), engine->outputChannels.blendedStatus);
	EXPECT_EQ(static_cast<uint8_t>(AirmassInjectionFault::None), engine->outputChannels.blendedFault);
	EXPECT_EQ(2, engine->airmassInjectionState.pendingCallbacks());

	engine->airmassInjectionState.callbackCompleted();
	engine->airmassInjectionState.callbackCompleted();
}
