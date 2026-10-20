#include "pch.h"

#include "alphan_airmass.h"
#include "speed_density_airmass.h"

namespace {
template <typename TValue, size_t TCount>
void expectStrictlyAscending(const TValue (&values)[TCount]) {
	for (size_t i = 1; i < TCount; i++) {
		const float previous = values[i - 1];
		const float current = values[i];
		EXPECT_LT(previous, current) << "axis index " << i;
	}
}
} // namespace

TEST(TableProfileDimensions, MinimumPinsDefaultAxesAreMonotonicAndFullRange) {
	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);

	expectStrictlyAscending(config->veLoadBins);
	expectStrictlyAscending(config->veRpmBins);
	expectStrictlyAscending(config->alphaNTpsBins);
	expectStrictlyAscending(config->alphaNRpmBins);
	expectStrictlyAscending(config->airmassBlendLoadBins);
	expectStrictlyAscending(config->airmassBlendRpmBins);

	const uint16_t expectedVeStart = 650;
	EXPECT_EQ(config->veRpmBins[0], expectedVeStart);
	EXPECT_EQ(config->veRpmBins[VE_RPM_COUNT - 1], 7000);
	EXPECT_EQ(config->alphaNRpmBins[0], expectedVeStart);
	EXPECT_EQ(config->alphaNRpmBins[ALPHA_N_RPM_COUNT - 1], 7000);
	EXPECT_FLOAT_EQ(config->veLoadBins[0], 10);
	EXPECT_FLOAT_EQ(config->veLoadBins[FUEL_LOAD_COUNT - 1], 160);
	EXPECT_FLOAT_EQ(config->alphaNTpsBins[0], 0);
	EXPECT_FLOAT_EQ(config->alphaNTpsBins[ALPHA_N_LOAD_COUNT - 1], 100);
	EXPECT_FLOAT_EQ(config->airmassBlendLoadBins[0], 0);
	EXPECT_FLOAT_EQ(config->airmassBlendLoadBins[AIRMASS_BLEND_LOAD_COUNT - 1], 100);
	EXPECT_EQ(config->airmassBlendRpmBins[0], 800);
	EXPECT_EQ(config->airmassBlendRpmBins[AIRMASS_BLEND_RPM_COUNT - 1], AIRMASS_BLEND_RPM_COUNT == 16 ? 18000 : 8000);
}

TEST(TableProfileDimensions, MinimumPinsMatricesReflectResolvedMacros) {
	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);

	EXPECT_EQ(FOME_VE_RPM_COUNT, VE_RPM_COUNT);
	EXPECT_EQ(ALPHA_N_RPM_COUNT, VE_RPM_COUNT);
	EXPECT_EQ(FOME_AIRMASS_BLEND_SIZE, AIRMASS_BLEND_LOAD_COUNT);
	EXPECT_EQ(AIRMASS_BLEND_RPM_COUNT, AIRMASS_BLEND_LOAD_COUNT);

	EXPECT_EQ(std::size(config->veTable), FUEL_LOAD_COUNT);
	EXPECT_EQ(std::size(config->veTable[0]), VE_RPM_COUNT);
	EXPECT_EQ(std::size(config->veLoadBins), FUEL_LOAD_COUNT);
	EXPECT_EQ(std::size(config->veRpmBins), VE_RPM_COUNT);

	EXPECT_EQ(std::size(config->alphaNTable), ALPHA_N_LOAD_COUNT);
	EXPECT_EQ(std::size(config->alphaNTable[0]), ALPHA_N_RPM_COUNT);
	EXPECT_EQ(std::size(config->alphaNTpsBins), ALPHA_N_LOAD_COUNT);
	EXPECT_EQ(std::size(config->alphaNRpmBins), ALPHA_N_RPM_COUNT);

	EXPECT_EQ(std::size(config->airmassBlendTable), AIRMASS_BLEND_LOAD_COUNT);
	EXPECT_EQ(std::size(config->airmassBlendTable[0]), AIRMASS_BLEND_RPM_COUNT);
	EXPECT_EQ(std::size(config->airmassBlendLoadBins), AIRMASS_BLEND_LOAD_COUNT);
	EXPECT_EQ(std::size(config->airmassBlendRpmBins), AIRMASS_BLEND_RPM_COUNT);
}

#if VE_RPM_COUNT > 16
TEST(TableProfileDimensions, MinimumPinsRectangularTablesInterpolateBeyondSixteenColumns) {
	EngineTestHelper eth(engine_type_e::MINIMAL_PINS);
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->useSeparateVeForIdle = false;
	engineConfiguration->veOverrideMode = VE_None;
	config->airmassTemperatureSource = AirmassTemperatureSource::Tcharge;
	setCylinderCount(4);
	engine->engineState.sd.tChargeK = 300;

	for (size_t i = 0; i < std::size(config->veLoadBins); i++) {
		config->veLoadBins[i] = 10 + i * 5;
	}
	for (size_t i = 0; i < std::size(config->veRpmBins); i++) {
		config->veRpmBins[i] = 500 + i * 200;
	}
	for (size_t i = 0; i < std::size(config->alphaNTpsBins); i++) {
		config->alphaNTpsBins[i] = 10 + i * 5;
	}
	for (size_t i = 0; i < std::size(config->alphaNRpmBins); i++) {
		config->alphaNRpmBins[i] = 500 + i * 200;
	}
	for (auto& blend : config->veBlends) {
		blend.blendParameter = GPPWM_Zero;
	}

	setTable(config->veTable, 10);
	config->veTable[2][20] = 40;
	config->veTable[3][20] = 60;
	config->veTable[2][21] = 80;
	config->veTable[3][21] = 100;

	setTable(config->alphaNTable, 10);
	config->alphaNTable[2][20] = 40;
	config->alphaNTable[3][20] = 60;
	config->alphaNTable[2][21] = 80;
	config->alphaNTable[3][21] = 100;

	const float loadMidpoint = (config->veLoadBins[2] + config->veLoadBins[3]) * 0.5f;
	const float rpmMidpoint = (config->veRpmBins[20] + config->veRpmBins[21]) * 0.5f;
	EXPECT_FLOAT_EQ(
			50,
			interpolate3d(config->veTable, config->veLoadBins, loadMidpoint, config->veRpmBins, config->veRpmBins[20]));
	EXPECT_FLOAT_EQ(
			60,
			interpolate3d(config->veTable, config->veLoadBins, config->veLoadBins[2], config->veRpmBins, rpmMidpoint));
	EXPECT_FLOAT_EQ(
			70, interpolate3d(config->veTable, config->veLoadBins, loadMidpoint, config->veRpmBins, rpmMidpoint));

	testing::NiceMock<MockVp3d> mapEstimate;
	SpeedDensityAirmass speedDensity(nullptr, mapEstimate);
	AirmassDiagnostics speedDensityDiagnostics;
	const auto speedDensityResult = speedDensity.evaluateAirmass(rpmMidpoint, loadMidpoint, &speedDensityDiagnostics);
	ASSERT_TRUE(speedDensityResult.Valid);
	EXPECT_FLOAT_EQ(speedDensityDiagnostics.Ve.Load, loadMidpoint);
	EXPECT_FLOAT_EQ(speedDensityDiagnostics.Ve.Ve, 70);

	Sensor::setMockValue(SensorType::Tps1, loadMidpoint);
	Sensor::setMockValue(SensorType::Map, loadMidpoint);
	AlphaNAirmass alphaN;
	AirmassDiagnostics alphaNDiagnostics;
	const auto alphaNResult = alphaN.evaluateAirmass(rpmMidpoint, &alphaNDiagnostics);
	ASSERT_TRUE(alphaNResult.Valid);
	EXPECT_FLOAT_EQ(alphaNDiagnostics.Ve.Load, loadMidpoint);
	EXPECT_FLOAT_EQ(alphaNDiagnostics.Ve.Ve, 70);
}
#endif
