#include "pch.h"

#include "airmass_loads.h"
#include "cylinder_fuel_trim.h"

#ifdef MODULE_CYLINDER_FUEL_TRIM

#if CYLINDER_FUEL_TRIM_COUNT >= 4

namespace {

constexpr size_t TrimCylinders = 4;
constexpr float TrimFrameMass = 0.5f;
constexpr float StandardChargeTemperatureK = 293.15f;
constexpr float StandardChargePressureKpa = 101.325f;
constexpr float AirGasConstant = 0.28705f;

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

void configureTrimCalibration() {
	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 15, 1);
	setLinearCurve(config->cylinderFuelTrimRpmBins, 0, 15000, 1000);
	for (auto& source : config->cylinderFuelTrimLoadSource) {
		source = AFR_None;
	}
	for (auto& trim : config->cylinderFuelTrims) {
		setTable(trim.table, 0);
	}
	engineConfiguration->isInjectionEnabled = true;
	engineConfiguration->displacement = 2.4f;
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	setCylinderCount(TrimCylinders);
	invalidateAirmassCalibration();
}

AirmassInputs makeTrimInputs(uint8_t cylinderCount = TrimCylinders, float rpm = 3500, float tps = 20) {
	AirmassInputs inputs;
	inputs.HasPublicationContext = true;
	inputs.PublicationEpoch = engine->airmassInjectionState.publicationEpoch();
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
	inputs.ActiveStrategy = LM_SPEED_DENSITY;
	inputs.Rpm = rpm;
	inputs.MeasuredMap = expected<float>(50);
	inputs.NativeMap = {71.25f, 0, true, true, false};
	inputs.NativeLoad = expected<float>(65);
	inputs.Tps = expected<float>(tps);
	inputs.Pedal = expected<float>(30);
	inputs.TemperatureK = 300;
	inputs.TemperatureValid = true;
	inputs.Displacement = engineConfiguration->displacement;
	inputs.CylinderCount = cylinderCount;
	return inputs;
}

AirmassConsumerLoadCandidate prepareLoads(const AirmassInputs& inputs) {
	auto loads = prepareAirmassConsumerLoads(inputs, TrimFrameMass);
	EXPECT_TRUE(loads.Prepared);
	EXPECT_TRUE(loads.Snapshot.Valid);
	return loads;
}

float expectedFillingPercent() {
	const float displacementPerCylinder = engineConfiguration->displacement / TrimCylinders;
	const float standardCharge =
			displacementPerCylinder * StandardChargePressureKpa / (AirGasConstant * StandardChargeTemperatureK);
	return 100 * TrimFrameMass / standardCharge;
}

float expectedLoad(load_override_e source) {
	switch (source) {
		case AFR_None:
		case AFR_EffectiveMAP:
			return 71.25f;
		case AFR_MAP:
			return 50;
		case AFR_Tps:
			return 20;
		case AFR_AccPedal:
			return 30;
		case AFR_CylFilling:
			return expectedFillingPercent();
		default:
			return NAN;
	}
}

void configureAllTrimSources(load_override_e source) {
	for (auto& configured : config->cylinderFuelTrimLoadSource) {
		configured = source;
	}
}

void expectTrimToMatchSource(const AirmassConsumerLoadCandidate& loads, load_override_e source) {
	CylinderFuelTrimCandidate trim;
	const auto inputs = makeTrimInputs();
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, trim)) << static_cast<int>(source);
	ASSERT_TRUE(trim.Valid);
	EXPECT_EQ(trim.CylinderCount, TrimCylinders);
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_NEAR(trim.Load[cylinder], expectedLoad(source), EPS4D) << cylinder;
		EXPECT_FLOAT_EQ(trim.Factor[cylinder], 1) << cylinder;
	}
}

} // namespace

TEST(CylinderFuelTrim, PublishesOneSelectedLoadAndNeutralizesUnusedCapacity) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();
	configureAllTrimSources(AFR_Tps);
	setTable(config->cylinderFuelTrims[0].table, 6);
	setTable(config->cylinderFuelTrims[1].table, -4);

	const auto inputs = makeTrimInputs();
	const auto loads = prepareLoads(inputs);
	CylinderFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	ASSERT_TRUE(candidate.Valid);
	EXPECT_EQ(candidate.CylinderCount, TrimCylinders);
	EXPECT_FLOAT_EQ(candidate.Factor[0], 1.06f);
	EXPECT_FLOAT_EQ(candidate.Factor[1], 0.96f);
	EXPECT_FLOAT_EQ(candidate.Factor[2], 1);
	EXPECT_FLOAT_EQ(candidate.Factor[3], 1);
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_FLOAT_EQ(candidate.Load[cylinder], 20) << cylinder;
	}

	// Preparation is side-effect free.
	for (const auto& output : engine->outputChannels.cylinderFuelTrim) {
		EXPECT_FLOAT_EQ(output, 0);
	}
	for (const auto& output : engine->outputChannels.cylinderFuelTrimLoad) {
		EXPECT_FLOAT_EQ(output, 0);
	}

	ASSERT_TRUE(publishCylinderFuelTrim(*engine, candidate));
	EXPECT_NEAR(engine->outputChannels.cylinderFuelTrim[0], 6, EPS4D);
	EXPECT_NEAR(engine->outputChannels.cylinderFuelTrim[1], -4, EPS4D);
	EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrimLoad[0], 20);
	EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrimLoad[1], 20);
	for (size_t cylinder = TrimCylinders; cylinder < CYLINDER_FUEL_TRIM_COUNT; cylinder++) {
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrim[cylinder], 0) << cylinder;
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrimLoad[cylinder], 0) << cylinder;
	}

	// Invalid publication clears the previous trim transactionally.
	candidate.Factor[0] = NAN;
	EXPECT_FALSE(publishCylinderFuelTrim(*engine, candidate));
	for (size_t cylinder = 0; cylinder < CYLINDER_FUEL_TRIM_COUNT; cylinder++) {
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrim[cylinder], 0) << cylinder;
		EXPECT_FLOAT_EQ(engine->outputChannels.cylinderFuelTrimLoad[cylinder], 0) << cylinder;
	}
}

TEST(CylinderFuelTrim, CapturesEverySupportedLoadSourcePerCylinder) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();

	for (uint8_t rawSource = AFR_None; rawSource <= AFR_EffectiveMAP; rawSource++) {
		const auto source = static_cast<load_override_e>(rawSource);
		configureAllTrimSources(source);
		const auto inputs = makeTrimInputs();
		const auto loads = prepareLoads(inputs);
		expectTrimToMatchSource(loads, source);
	}
}

TEST(CylinderFuelTrim, InterpolatesNonconstantTrimInLoadAndRpm) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();
	configureAllTrimSources(AFR_Tps);
	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 7.5f, 0.5f);

	auto& table = config->cylinderFuelTrims[0].table;
	table[2][2] = 4;
	table[3][2] = 6;
	table[2][3] = 6;
	table[3][3] = 8;
	table[2][4] = 10;
	table[3][4] = 12;

	const auto midpointInputs = makeTrimInputs(TrimCylinders, 3500, 1.25f);
	const auto midpointLoads = prepareLoads(midpointInputs);
	CylinderFuelTrimCandidate midpoint;
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, midpointInputs, midpointLoads, midpoint));
	EXPECT_FLOAT_EQ(midpoint.Load[0], 1.25f);
	EXPECT_NEAR(midpoint.Factor[0], 1.09f, EPS4D);

	const auto lowerRpmInputs = makeTrimInputs(TrimCylinders, 2500, 1.25f);
	const auto lowerRpmLoads = prepareLoads(lowerRpmInputs);
	CylinderFuelTrimCandidate lowerRpm;
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, lowerRpmInputs, lowerRpmLoads, lowerRpm));
	EXPECT_FLOAT_EQ(lowerRpm.Load[0], 1.25f);
	EXPECT_NEAR(lowerRpm.Factor[0], 1.06f, EPS4D);
}

TEST(CylinderFuelTrim, RejectsInvalidSourceAxesAndCells) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();

	auto prepareCurrentFrame = [] {
		const auto inputs = makeTrimInputs();
		const auto loads = prepareLoads(inputs);
		CylinderFuelTrimCandidate candidate;
		const bool prepared = prepareCylinderFuelTrim(*engine, inputs, loads, candidate);
		return std::make_pair(prepared, candidate);
	};

	configureAllTrimSources(AFR_Invalid);
	auto invalidSource = prepareCurrentFrame().second;
	EXPECT_FALSE(invalidSource.Valid);
	EXPECT_FLOAT_EQ(invalidSource.Factor[0], 1);

	configureAllTrimSources(AFR_None);
	config->cylinderFuelTrimLoadBins[1] = config->cylinderFuelTrimLoadBins[0];
	invalidateAirmassCalibration();
	auto [invalidLoadAxisPrepared, invalidLoadAxis] = prepareCurrentFrame();
	EXPECT_FALSE(invalidLoadAxisPrepared);
	EXPECT_FALSE(invalidLoadAxis.Valid);

	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 15, 1);
	config->cylinderFuelTrimRpmBins[2] = config->cylinderFuelTrimRpmBins[1];
	invalidateAirmassCalibration();
	auto [invalidRpmAxisPrepared, invalidRpmAxis] = prepareCurrentFrame();
	EXPECT_FALSE(invalidRpmAxisPrepared);
	EXPECT_FALSE(invalidRpmAxis.Valid);

	setLinearCurve(config->cylinderFuelTrimRpmBins, 0, 15000, 1000);
	config->cylinderFuelTrims[0].table[3][4] = 25.2f;
	invalidateAirmassCalibration();
	auto [invalidCellPrepared, invalidCell] = prepareCurrentFrame();
	EXPECT_FALSE(invalidCellPrepared);
	EXPECT_FALSE(invalidCell.Valid);
	EXPECT_FLOAT_EQ(invalidCell.Factor[0], 1);
}

TEST(CylinderFuelTrim, KeepsCapturedSourcesAfterLiveSelectorChange) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();
	configureAllTrimSources(AFR_Tps);

	const auto inputs = makeTrimInputs();
	const auto capturedLoads = prepareLoads(inputs);
	configureAllTrimSources(AFR_MAP);
	Sensor::setMockValue(SensorType::Tps1, 90);

	CylinderFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, capturedLoads, candidate));
	for (size_t cylinder = 0; cylinder < TrimCylinders; cylinder++) {
		EXPECT_FLOAT_EQ(candidate.Load[cylinder], 20) << cylinder;
	}
}

TEST(CylinderFuelTrim, CalibrationCacheUsesGenerationConfigurationAndCylinderCount) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();
	configureAllTrimSources(AFR_None);

	auto inputs = makeTrimInputs();
	auto loads = prepareLoads(inputs);
	CylinderFuelTrimCandidate candidate;
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_EQ(engine->cylinderFuelTrimCalibration.Scans, 1);

	// A live mutation is only rescanned after its generation is invalidated.
	config->cylinderFuelTrimLoadBins[1] = config->cylinderFuelTrimLoadBins[0];
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_EQ(engine->cylinderFuelTrimCalibration.Scans, 1);

	setLinearCurve(config->cylinderFuelTrimLoadBins, 0, 15, 1);
	invalidateAirmassCalibration();
	inputs = makeTrimInputs();
	loads = prepareLoads(inputs);
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_EQ(engine->cylinderFuelTrimCalibration.Scans, 2);

	engine->globalConfigurationVersion++;
	inputs = makeTrimInputs();
	loads = prepareLoads(inputs);
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_EQ(engine->cylinderFuelTrimCalibration.Scans, 3);

	inputs = makeTrimInputs(TrimCylinders - 1);
	loads = prepareLoads(inputs);
	ASSERT_TRUE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_EQ(engine->cylinderFuelTrimCalibration.Scans, 4);
}

TEST(CylinderFuelTrim, CapacityMismatchIsSilentAtRuntimeAndRejectedByConfiguration) {
#if CYLINDER_FUEL_TRIM_COUNT < MAX_CYLINDER_COUNT
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureTrimCalibration();
	configureAllTrimSources(AFR_Invalid);
	const auto inputs = makeTrimInputs(oversizedCylinderCount());

	const auto loads = prepareAirmassConsumerLoads(inputs, TrimFrameMass);
	EXPECT_FALSE(loads.Prepared);
	EXPECT_FALSE(loads.Snapshot.Valid);
	CylinderFuelTrimCandidate candidate;
	EXPECT_FALSE(prepareCylinderFuelTrim(*engine, inputs, loads, candidate));
	EXPECT_FALSE(candidate.Valid);
	EXPECT_FALSE(hasFirmwareError());

	setCylinderCount(oversizedCylinderCount());
	engineConfiguration->isInjectionEnabled = false;
	EXPECT_TRUE(validateConfig());
	engineConfiguration->isInjectionEnabled = true;
	EXPECT_THROW(validateConfig(), std::logic_error);
#else
	GTEST_SKIP() << "A 12-cylinder trim capacity has no larger legal MAX_CYLINDER_COUNT victim";
#endif
}

#endif // CYLINDER_FUEL_TRIM_COUNT >= 4

#endif // MODULE_CYLINDER_FUEL_TRIM
