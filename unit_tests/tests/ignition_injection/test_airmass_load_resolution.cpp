#include "pch.h"

#include "airmass.h"

namespace {
constexpr float AirGasConstant = 0.28705f;

float standardCharge(float displacement, float cylinderCount) {
	return (displacement / cylinderCount) * 101.325f / (AirGasConstant * 293.15f);
}

AirmassInputs baseInputs(engine_load_mode_e strategy = LM_SPEED_DENSITY) {
	AirmassInputs inputs;
	inputs.Rpm = 2400;
	inputs.ActiveStrategy = strategy;
	inputs.TpsToleranceMin = -10;
	inputs.TpsToleranceMax = 110;
	inputs.MeasuredMap = 92;
	inputs.NativeMap.Map = 71.25f;
	inputs.NativeMap.Valid = true;
	inputs.NativeMap.UsesEstimate = true;
	inputs.Tps = 23.5f;
	inputs.Pedal = 41.5f;
	inputs.Displacement = 2.4f;
	inputs.CylinderCount = 4;
	inputs.TemperatureK = 313.15f;
	inputs.TemperatureValid = true;
	return inputs;
}

void expectLoad(
		const AirmassLoad& load,
		float value,
		AirmassLoadSource source,
		AirmassLoadUnit unit,
		bool valid,
		bool usesEstimate = false) {
	EXPECT_FLOAT_EQ(value, load.Value);
	EXPECT_EQ(source, load.Source);
	EXPECT_EQ(unit, load.Unit);
	EXPECT_EQ(valid, load.Valid);
	EXPECT_EQ(usesEstimate, load.UsesEstimate);
}
} // namespace

TEST(AirmassLoadResolution, NativeNoneUsesEachModelsCapturedCoordinate) {
	auto speedDensity = baseInputs();
	expectLoad(
			resolveAirmassLoad(speedDensity, 0.3f, AFR_None),
			71.25f,
			AirmassLoadSource::EffectiveMap,
			AirmassLoadUnit::Kpa,
			true,
			true);
	expectLoad(
			resolveAirmassLoad(speedDensity, 0.3f, AFR_EffectiveMAP),
			71.25f,
			AirmassLoadSource::EffectiveMap,
			AirmassLoadUnit::Kpa,
			true,
			true);
	expectLoad(
			resolveAirmassLoad(speedDensity, 0.3f, AFR_MAP),
			92,
			AirmassLoadSource::MeasuredMap,
			AirmassLoadUnit::Kpa,
			true);

	auto alphaN = baseInputs(LM_ALPHA_N);
	alphaN.NativeMap = {};
	expectLoad(
			resolveAirmassLoad(alphaN, 0.3f, AFR_None), 23.5f, AirmassLoadSource::Tps, AirmassLoadUnit::Percent, true);
	EXPECT_FALSE(resolveAirmassLoad(alphaN, 0.3f, AFR_EffectiveMAP).Valid);
	EXPECT_FALSE(alphaN.NativeMap.Valid);

	auto maf = baseInputs(LM_REAL_MAF);
	maf.NativeMap = {};
	maf.NativeLoad = 138.75f;
	expectLoad(
			resolveAirmassLoad(maf, 0.3f, AFR_None),
			138.75f,
			AirmassLoadSource::CylinderFilling,
			AirmassLoadUnit::Percent,
			true);
	EXPECT_FALSE(resolveAirmassLoad(maf, 0.3f, AFR_EffectiveMAP).Valid);
	EXPECT_FALSE(maf.NativeMap.Valid);
}

TEST(AirmassLoadResolution, CylinderFillingUsesCorrectedMassAndCapturedGeometry) {
	auto inputs = baseInputs();
	inputs.NativeMap = {};
	inputs.MeasuredMap = unexpected;
	inputs.Tps = unexpected;
	inputs.Pedal = unexpected;

	const float mass = 0.42f;
	const auto filling = resolveAirmassLoad(inputs, mass, AFR_CylFilling);
	expectLoad(
			filling,
			100 * mass / standardCharge(inputs.Displacement, inputs.CylinderCount),
			AirmassLoadSource::CylinderFilling,
			AirmassLoadUnit::Percent,
			true);

	inputs.Displacement = 3.6f;
	inputs.CylinderCount = 6;
	const auto changedGeometry = resolveAirmassLoad(inputs, mass, AFR_CylFilling);
	EXPECT_TRUE(changedGeometry.Valid);
	EXPECT_NEAR(changedGeometry.Value, 100 * mass / standardCharge(inputs.Displacement, inputs.CylinderCount), EPS4D);

	inputs.CylinderCount = 0;
	expectLoad(
			resolveAirmassLoad(inputs, mass, AFR_CylFilling),
			0,
			AirmassLoadSource::CylinderFilling,
			AirmassLoadUnit::Percent,
			false);
}

TEST(AirmassLoadResolution, ConsumerFallbackKeepsStrictProvenanceIndependent) {
	auto inputs = baseInputs();
	inputs.NativeMap = {};
	inputs.MeasuredMap = unexpected;
	inputs.Tps = unexpected;
	inputs.Pedal = unexpected;
	const float mass = 0.3f;

	const auto measuredStrict = resolveAirmassLoad(inputs, mass, AFR_MAP);
	expectLoad(measuredStrict, 0, AirmassLoadSource::MeasuredMap, AirmassLoadUnit::Kpa, false);
	const auto measuredConsumer = resolveAirmassConsumerLoad(inputs, mass, AFR_MAP);
	expectLoad(measuredConsumer, 200, AirmassLoadSource::MeasuredMap, AirmassLoadUnit::Kpa, true);

	const auto effectiveStrict = resolveAirmassLoad(inputs, mass, AFR_EffectiveMAP);
	expectLoad(effectiveStrict, 0, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, false);
	const auto effectiveConsumer = resolveAirmassConsumerLoad(inputs, mass, AFR_EffectiveMAP);
	expectLoad(effectiveConsumer, 200, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, true);
	const auto nativeStrict = resolveAirmassLoad(inputs, mass, AFR_None);
	expectLoad(nativeStrict, 0, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, false);
	const auto nativeConsumer = resolveAirmassConsumerLoad(inputs, mass, AFR_None);
	expectLoad(nativeConsumer, 200, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, true);

	const auto tpsStrict = resolveAirmassLoad(inputs, mass, AFR_Tps);
	expectLoad(tpsStrict, 0, AirmassLoadSource::Tps, AirmassLoadUnit::Percent, false);
	const auto tpsConsumer = resolveAirmassConsumerLoad(inputs, mass, AFR_Tps);
	expectLoad(tpsConsumer, 100, AirmassLoadSource::Tps, AirmassLoadUnit::Percent, true);

	const auto pedalStrict = resolveAirmassLoad(inputs, mass, AFR_AccPedal);
	expectLoad(pedalStrict, 0, AirmassLoadSource::Pedal, AirmassLoadUnit::Percent, false);
	const auto pedalConsumer = resolveAirmassConsumerLoad(inputs, mass, AFR_AccPedal);
	expectLoad(pedalConsumer, 100, AirmassLoadSource::Pedal, AirmassLoadUnit::Percent, true);

	inputs.ActiveStrategy = LM_ALPHA_N;
	EXPECT_FALSE(resolveAirmassLoad(inputs, mass, AFR_None).Valid);
	expectLoad(
			resolveAirmassConsumerLoad(inputs, mass, AFR_None),
			100,
			AirmassLoadSource::Tps,
			AirmassLoadUnit::Percent,
			true);

	EXPECT_TRUE(resolveAirmassLoad(inputs, mass, AFR_CylFilling).Valid)
			<< "an unavailable optional coordinate must not invalidate good mass";
	EXPECT_FALSE(resolveAirmassConsumerLoad(inputs, NAN, AFR_MAP).Valid)
			<< "an invalid result must not invent a downstream coordinate";
}

TEST(AirmassLoadResolution, MissingMapSourcesRemainIndependent) {
	auto inputs = baseInputs();
	inputs.MeasuredMap = unexpected;

	EXPECT_FALSE(resolveAirmassLoad(inputs, 0.3f, AFR_MAP).Valid);
	EXPECT_FLOAT_EQ(resolveAirmassConsumerLoad(inputs, 0.3f, AFR_MAP).Value, 200);
	EXPECT_FLOAT_EQ(resolveAirmassLoad(inputs, 0.3f, AFR_EffectiveMAP).Value, 71.25f);
	EXPECT_TRUE(resolveAirmassLoad(inputs, 0.3f, AFR_EffectiveMAP).Valid);

	inputs.NativeMap = {};
	inputs.MeasuredMap = 92;
	EXPECT_FALSE(resolveAirmassLoad(inputs, 0.3f, AFR_EffectiveMAP).Valid);
	EXPECT_FLOAT_EQ(resolveAirmassConsumerLoad(inputs, 0.3f, AFR_EffectiveMAP).Value, 200);
	EXPECT_FLOAT_EQ(resolveAirmassLoad(inputs, 0.3f, AFR_MAP).Value, 92);
	EXPECT_TRUE(resolveAirmassLoad(inputs, 0.3f, AFR_MAP).Valid);
}

TEST(AirmassLoadResolution, InvalidSelectorsDoNotInventCoordinates) {
	const auto inputs = baseInputs();
	for (load_override_e selector : {AFR_Invalid, static_cast<load_override_e>(7)}) {
		expectLoad(
				resolveAirmassLoad(inputs, 0.3f, selector),
				0,
				AirmassLoadSource::Invalid,
				AirmassLoadUnit::Percent,
				false);
		expectLoad(
				resolveAirmassConsumerLoad(inputs, 0.3f, selector),
				0,
				AirmassLoadSource::Invalid,
				AirmassLoadUnit::Percent,
				false);
	}
}

TEST(AirmassLoadResolution, StoreKeepsSelectorsIndependentAndSnapshotOwned) {
	auto inputs = baseInputs();
	inputs.NativeLoad = 137.5f;
	const AirmassResolvedLoads loads(inputs, 0.3f);

	expectLoad(loads.strict(AFR_None), 71.25f, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, true, true);
	expectLoad(
			loads.strict(AFR_EffectiveMAP), 71.25f, AirmassLoadSource::EffectiveMap, AirmassLoadUnit::Kpa, true, true);
	expectLoad(loads.strict(AFR_MAP), 92, AirmassLoadSource::MeasuredMap, AirmassLoadUnit::Kpa, true);
	expectLoad(loads.strict(AFR_Tps), 23.5f, AirmassLoadSource::Tps, AirmassLoadUnit::Percent, true);
	expectLoad(loads.strict(AFR_AccPedal), 41.5f, AirmassLoadSource::Pedal, AirmassLoadUnit::Percent, true);
	expectLoad(
			loads.strict(AFR_CylFilling),
			100 * 0.3f / standardCharge(inputs.Displacement, inputs.CylinderCount),
			AirmassLoadSource::CylinderFilling,
			AirmassLoadUnit::Percent,
			true);

	inputs.NativeMap.Map = 12;
	inputs.MeasuredMap = 14;
	inputs.Tps = 16;
	inputs.Pedal = 18;
	inputs.Displacement = 8;
	EXPECT_FLOAT_EQ(loads.strict(AFR_EffectiveMAP).Value, 71.25f);
	EXPECT_FLOAT_EQ(loads.strict(AFR_MAP).Value, 92);
	EXPECT_FLOAT_EQ(loads.strict(AFR_Tps).Value, 23.5f);
	EXPECT_FLOAT_EQ(loads.strict(AFR_AccPedal).Value, 41.5f);
	EXPECT_NE(loads.strict(AFR_CylFilling).Value, 100 * 0.3f / standardCharge(8, 4));
}

TEST(AirmassLoadResolution, DryResolutionDoesNotReadOrWriteLiveState) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setMockValue(SensorType::Map, 199);
	Sensor::setMockValue(SensorType::Tps1, 88);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 77);
	engine->engineState.currentVe = 91;
	engine->outputChannels.fallbackMap = 123;

	const auto inputs = baseInputs();
	const AirmassResolvedLoads loads(inputs, 0.3f);
	EXPECT_FLOAT_EQ(loads.strict(AFR_MAP).Value, 92);
	EXPECT_FLOAT_EQ(loads.strict(AFR_Tps).Value, 23.5f);
	EXPECT_FLOAT_EQ(loads.strict(AFR_AccPedal).Value, 41.5f);

	EXPECT_FLOAT_EQ(engine->engineState.currentVe, 91);
	EXPECT_FLOAT_EQ(engine->outputChannels.fallbackMap, 123);
	EXPECT_FLOAT_EQ(Sensor::getOrZero(SensorType::Map), 199);
	EXPECT_FLOAT_EQ(Sensor::getOrZero(SensorType::Tps1), 88);
	EXPECT_FLOAT_EQ(Sensor::getOrZero(SensorType::AcceleratorPedal), 77);

	const auto first = loads.strict(AFR_EffectiveMAP);
	const auto second = loads.strict(AFR_EffectiveMAP);
	EXPECT_EQ(first.Value, second.Value);
	EXPECT_EQ(first.Valid, second.Valid);
	EXPECT_EQ(first.Source, second.Source);
	EXPECT_EQ(first.Unit, second.Unit);
	EXPECT_EQ(first.UsesEstimate, second.UsesEstimate);
}
