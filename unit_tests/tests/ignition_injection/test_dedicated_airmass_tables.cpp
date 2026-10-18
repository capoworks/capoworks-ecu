#include "pch.h"

#include "alphan_airmass.h"
#include "fuel_math.h"

using testing::StrictMock;

namespace {
constexpr float AirGasConstant = 0.28705f;

mass_t
expectedIdealGasMass(float displacement, int cylinders, float fillingPercent, float pressure, float temperatureK) {
	return (fillingPercent * 0.01f) * displacement * pressure / (AirGasConstant * temperatureK) / cylinders;
}

void configureAlphaNAxesForInterpolation() {
	setLinearCurve(config->alphaNTpsBins, 0, 7.5f, 0.1f);
	setLinearCurve(config->alphaNRpmBins, 500, 8000, 1);
	setTable(config->alphaNTable, 10);

	// At TPS 1.25 and 1750 RPM all four interpolation weights are 0.5.
	config->alphaNTable[2][2] = 40;
	config->alphaNTable[3][2] = 60;
	config->alphaNTable[2][3] = 80;
	config->alphaNTable[3][3] = 100;
	config->alphaNTable[0][0] = 33;
	config->alphaNTable[ALPHA_N_LOAD_COUNT - 1][ALPHA_N_RPM_COUNT - 1] = 99;
}

struct DedicatedTableIdleController : public MockIdleController {
	bool isIdlingOrTaper() const override {
		return true;
	}
};
} // namespace

TEST(DedicatedAirmassTables, M73AlphaNCalibrationUsesItsDedicatedTable) {
	EngineTestHelper eth(engine_type_e::FRANKENSO_BMW_M73_F);
	engine->engineState.sd.tChargeK = 293.15f;
	Sensor::setMockValue(SensorType::Tps1, 25);

	EXPECT_EQ(ALPHA_N_LOAD_COUNT, 16);
	EXPECT_EQ(ALPHA_N_RPM_COUNT, 16);
	EXPECT_FLOAT_EQ(config->alphaNTable[0][0], 45);
	EXPECT_FLOAT_EQ(config->alphaNTpsBins[1], 0.5f);
	for (size_t load = 0; load < ALPHA_N_LOAD_COUNT; load++) {
		for (size_t rpm = 0; rpm < ALPHA_N_RPM_COUNT; rpm++) {
			EXPECT_FLOAT_EQ(config->alphaNTable[load][rpm], 45);
		}
	}

	engineConfiguration->fuelAlgorithm = LM_ALPHA_N;
	const auto airmass = getAirmassModel(engineConfiguration->fuelAlgorithm)->getAirmass(1800, true);
	ASSERT_TRUE(airmass);

	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 45);
	EXPECT_FLOAT_EQ(airmass.Value.EngineLoadPercent, 25);
	EXPECT_NEAR(
			airmass.Value.CylinderAirmass,
			expectedIdealGasMass(
					engineConfiguration->displacement, engine->engineState.cylinderCount, 45, 101.325f, 293.15f),
			EPS4D);
}

TEST(DedicatedAirmassTables, AlphaNUsesFractionalTpsAndIndependentRpmAxes) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->fuelAlgorithm = LM_REAL_MAF;
	engineConfiguration->displacement = 3.2f;
	setCylinderCount(4);
	configureAlphaNAxesForInterpolation();

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	Sensor::setMockValue(SensorType::Tps1, 1.25f);
	auto midpoint = alphaN.evaluateAirmass(1750, &diagnostics);
	EXPECT_TRUE(midpoint.Valid);
	EXPECT_FLOAT_EQ(midpoint.Result.EngineLoadPercent, 1.25f);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 70);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Load, 1.25f);
	EXPECT_NEAR(midpoint.Result.CylinderAirmass, expectedIdealGasMass(3.2f, 4, 70, 101.325f, 293.15f), EPS4D);

	Sensor::setMockValue(SensorType::Tps1, 0);
	auto belowAxes = alphaN.evaluateAirmass(100, &diagnostics);
	EXPECT_TRUE(belowAxes.Valid);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 33);
	EXPECT_NEAR(belowAxes.Result.CylinderAirmass, expectedIdealGasMass(3.2f, 4, 33, 101.325f, 293.15f), EPS4D);

	Sensor::setMockValue(SensorType::Tps1, 100);
	auto aboveAxes = alphaN.evaluateAirmass(9000, &diagnostics);
	EXPECT_TRUE(aboveAxes.Valid);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 99);
	EXPECT_NEAR(aboveAxes.Result.CylinderAirmass, expectedIdealGasMass(3.2f, 4, 99, 101.325f, 293.15f), EPS4D);
}

TEST(DedicatedAirmassTables, ExplicitProviderOverridesDedicatedAlphaNMap) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->displacement = 3.2f;
	setCylinderCount(4);
	setTable(config->alphaNTable, 10);
	config->alphaNTpsBins[2] = config->alphaNTpsBins[1];
	Sensor::setMockValue(SensorType::Tps1, 12.5f);

	StrictMock<MockVp3d> provider;
	EXPECT_CALL(provider, getValue(1800, testing::FloatNear(12.5f, EPS4D))).WillOnce(testing::Return(52));
	AlphaNAirmass alphaN(&provider);

	const auto result = alphaN.getAirmass(1800, true);
	ASSERT_TRUE(result);
	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 52);
	EXPECT_NEAR(result.Value.CylinderAirmass, expectedIdealGasMass(3.2f, 4, 52, 101.325f, 293.15f), EPS4D);
}

TEST(DedicatedAirmassTables, DedicatedAxesMustBeStrictlyAscending) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	ASSERT_TRUE(isAirmassModelConfigurationValid(LM_ALPHA_N));
	config->alphaNTpsBins[2] = config->alphaNTpsBins[1];
	EXPECT_FALSE(isAirmassModelConfigurationValid(LM_ALPHA_N));
	setLinearCurve(config->alphaNTpsBins, 0, 100, 1);
	config->alphaNRpmBins[ALPHA_N_RPM_COUNT - 1] = 18001;
	EXPECT_FALSE(isAirmassModelConfigurationValid(LM_ALPHA_N));
}

TEST(DedicatedAirmassTables, InvalidAxisDryReadDoesNotPublishOrRaiseFault) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->fuelAlgorithm = LM_ALPHA_N;
	config->alphaNTpsBins[2] = config->alphaNTpsBins[1];
	Sensor::setMockValue(SensorType::Tps1, 12);
	engine->engineState.currentVe = 91;
	const int warningCount = eth.getWarningCounter();

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	AirmassEvaluation evaluation;
	EXPECT_NO_FATAL_ERROR(evaluation = alphaN.evaluateAirmass(2200, &diagnostics));
	EXPECT_FALSE(evaluation.Valid);
	EXPECT_FLOAT_EQ(evaluation.Result.CylinderAirmass, 0);
	EXPECT_FALSE(diagnostics.Ve.HasValue);
	EXPECT_EQ(eth.getWarningCounter(), warningCount);
	EXPECT_FALSE(hasFirmwareError());
	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 91);
}

TEST(DedicatedAirmassTables, IdleVeLoadSourceIsIndependentOfMainAlphaNAxis) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->engineState.sd.tChargeK = 293.15f;
	engineConfiguration->veOverrideMode = VE_None;
	engineConfiguration->useSeparateVeForIdle = true;
	config->idleVeLoadSource = IdleVeLoadSource::MeasuredMap;
	engineConfiguration->idlePidDeactivationTpsThreshold = 10;
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	setTable(config->alphaNTable, 80);
	setTable(config->idleVeTable, 40);
	setLinearCurve(config->idleVeLoadBins, 0, 100, 1);
	setLinearCurve(config->idleVeRpmBins, 0, 2500, 1);

	DedicatedTableIdleController idle;
	engine->engineModules.get<IdleController>().set(&idle);
	Sensor::setMockValue(SensorType::Tps1, 12.5f);
	Sensor::setMockValue(SensorType::Map, 77);
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 7.5f);

	AlphaNAirmass alphaN;
	AirmassDiagnostics diagnostics;
	auto evaluation = alphaN.evaluateAirmass(2300, &diagnostics);

	EXPECT_TRUE(evaluation.Valid);
	EXPECT_FLOAT_EQ(evaluation.Result.EngineLoadPercent, 12.5f);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Load, 12.5f);
	EXPECT_FLOAT_EQ(diagnostics.Ve.IdleLoad, 77);
	EXPECT_FLOAT_EQ(diagnostics.Ve.Ve, 60);
}
