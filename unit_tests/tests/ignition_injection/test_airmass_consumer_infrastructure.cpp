#include "pch.h"

#include "airmass_loads.h"

namespace {
using Token = AirmassInjectionState::CalculationToken;

void configureConsumers() {
	engineConfiguration->displacement = 2.4f;
	setCylinderCount(4);
	engineConfiguration->isInjectionEnabled = true;
	engineConfiguration->isIgnitionEnabled = true;
	engineConfiguration->enableStagedInjection = false;
	engineConfiguration->fuelClosedLoopCorrectionEnabled = false;
	engineConfiguration->lambdaProtectionEnable = false;
}

AirmassInputs captureConsumerInputs(engine_load_mode_e strategy) {
	engineConfiguration->fuelAlgorithm = strategy;
	const Token token =
			engine->airmassInjectionState.beginCalculation(strategy, 2400, engine->getGlobalConfigurationVersion());

	AirmassInputs inputs;
	inputs.HasPublicationContext = true;
	inputs.PublicationEpoch = token;
	inputs.ConfigurationVersion = engine->getGlobalConfigurationVersion();
	inputs.CalibrationGeneration = engine->airmassCalibration.Generation;
	inputs.ActiveStrategy = strategy;
	inputs.Displacement = engineConfiguration->displacement;
	inputs.CylinderCount = engine->engineState.cylinderCount;
	inputs.Rpm = 2400;
	inputs.MeasuredMap = 92;
	inputs.NativeMap.Map = 71.25f;
	inputs.NativeMap.Valid = true;
	inputs.NativeMap.UsesEstimate = true;
	inputs.Tps = 23.5f;
	inputs.Pedal = 41.5f;
	inputs.TpsToleranceMin = -10;
	inputs.TpsToleranceMax = 110;
	return inputs;
}

bool publish(const AirmassConsumerLoadCandidate& candidate) {
	chibios_rt::CriticalSectionLocker csl;
	auto admission = engine->airmassInjectionState.locked(csl);
	return publishAirmassConsumerLoads(candidate, admission);
}
} // namespace

TEST(AirmassConsumerInfrastructure, SelectorsRemainIndependentAndFullPrecision) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->injectionPhaseLoadSource = AFR_Tps;
	config->ignitionTrimLoadSource[0] = AFR_MAP;
	config->ignitionTrimLoadSource[1] = AFR_AccPedal;
	config->ignitionTrimLoadSource[2] = AFR_CylFilling;

	const auto candidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3125f);
	ASSERT_TRUE(candidate.Prepared);
	ASSERT_TRUE(publish(candidate));

	EXPECT_FLOAT_EQ(23.5f, getAirmassConsumerLoad(AirmassConsumer::InjectionPhase));
	EXPECT_FLOAT_EQ(92.0f, getAirmassConsumerLoad(AirmassConsumer::IgnitionTrim, 0));
	EXPECT_FLOAT_EQ(41.5f, getAirmassConsumerLoad(AirmassConsumer::IgnitionTrim, 1));
	EXPECT_GT(getAirmassConsumerLoad(AirmassConsumer::IgnitionTrim, 2), 0);
	EXPECT_FALSE(std::isfinite(getAirmassConsumerLoad(AirmassConsumer::IgnitionTrim, 4)));
	EXPECT_FLOAT_EQ(23.5f, engine->outputChannels.injectionPhaseLoad);
}

TEST(AirmassConsumerInfrastructure, StrictProvenanceAndPressureFallbacksStaySeparate) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->ignitionTrimLoadSource[0] = AFR_MAP;
	config->hpfpTargetLoadSource = AFR_MAP;
	auto inputs = captureConsumerInputs(LM_SPEED_DENSITY);
	inputs.MeasuredMap = unexpected;

	const auto candidate = prepareAirmassConsumerLoads(inputs, 0.3f);
	ASSERT_TRUE(candidate.Prepared);
	ASSERT_TRUE(candidate.Snapshot.Valid);
	EXPECT_FALSE(candidate.Snapshot.ValidSources & (1u << AFR_MAP));
	ASSERT_TRUE(publish(candidate));

	EXPECT_FLOAT_EQ(200, getAirmassConsumerLoad(AirmassConsumer::IgnitionTrim));
	EXPECT_FLOAT_EQ(0, getAirmassConsumerLoad(AirmassConsumer::HpfpTarget));
	EXPECT_FLOAT_EQ(200, getAirmassSelectedLoad(AFR_MAP, 0));
	EXPECT_TRUE(hasAirmassLoadFallback());
}

TEST(AirmassConsumerInfrastructure, MafNativeLoadIsPretrimAndFillingUsesFinalMass) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	auto inputs = captureConsumerInputs(LM_REAL_MAF);
	inputs.NativeLoad = 138.75f;

	const auto candidate = prepareAirmassConsumerLoads(inputs, 0.375f);
	ASSERT_TRUE(candidate.Prepared);
	EXPECT_FLOAT_EQ(138.75f, candidate.Snapshot.Native);
	EXPECT_TRUE(std::isfinite(candidate.Snapshot.Filling));
	EXPECT_NE(138.75f, candidate.Snapshot.Filling);
	EXPECT_TRUE(candidate.Snapshot.ValidSources & (1u << AFR_None));
	EXPECT_TRUE(candidate.Snapshot.ValidSources & (1u << AFR_CylFilling));
}

TEST(AirmassConsumerInfrastructure, DisabledConsumersAndUnusedCylindersHaveZeroCursors) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->stagingLoadSource = AFR_Tps;
	engineConfiguration->enableStagedInjection = false;
	engine->outputChannels.stagingLoad = 12.3f;
	engine->outputChannels.ignitionTrimLoad[5] = 45.6f;
	engine->outputChannels.knockGainLoad[5] = 67.8f;

	const auto candidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(candidate.Prepared);
	EXPECT_FALSE(candidate.Snapshot.EnabledConsumers & (1u << static_cast<unsigned>(AirmassConsumer::Staging)));
	ASSERT_TRUE(publish(candidate));

	EXPECT_FLOAT_EQ(0, engine->outputChannels.stagingLoad);
	EXPECT_FLOAT_EQ(0, engine->outputChannels.ignitionTrimLoad[5]);
	EXPECT_FLOAT_EQ(0, engine->outputChannels.knockGainLoad[5]);
}

TEST(AirmassConsumerInfrastructure, ContextKeepsCapturedSelectorWhileNextContextSeesUpdate) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->injectionPhaseLoadSource = AFR_Tps;
	const auto firstCandidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(firstCandidate));

	const AirmassConsumerLoadContext first(AirmassConsumer::InjectionPhase);
	ASSERT_TRUE(first.isCurrent());
	EXPECT_FLOAT_EQ(23.5f, first.get(AirmassConsumer::InjectionPhase));

	config->injectionPhaseLoadSource = AFR_MAP;
	const AirmassConsumerLoadContext second(AirmassConsumer::InjectionPhase);
	EXPECT_FLOAT_EQ(23.5f, first.get(AirmassConsumer::InjectionPhase));
	EXPECT_FLOAT_EQ(92.0f, second.get(AirmassConsumer::InjectionPhase));
}

TEST(AirmassConsumerInfrastructure, CandidateContextUsesCurrentFrameAndKeepsCapturedSelectors) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->injectionPhaseLoadSource = AFR_Tps;

	const auto published = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(published));

	auto currentInputs = captureConsumerInputs(LM_SPEED_DENSITY);
	currentInputs.Tps = 61.25f;
	currentInputs.MeasuredMap = 94.125f;
	const auto current = prepareAirmassConsumerLoads(currentInputs, 0.4f);
	ASSERT_TRUE(current.Prepared);
	const AirmassConsumerLoadContext publishedBeforeChange(AirmassConsumer::InjectionPhase);
	EXPECT_FLOAT_EQ(23.5f, publishedBeforeChange.get(AirmassConsumer::InjectionPhase));

	// The selector changes after preparation. The candidate context must retain
	// its captured selector and coordinates, while a new context resolves the
	// previous completed frame using the new selector.
	config->injectionPhaseLoadSource = AFR_MAP;
	const AirmassConsumerLoadContext currentContext(current);
	const AirmassConsumerLoadContext publishedContext(AirmassConsumer::InjectionPhase);
	ASSERT_TRUE(currentContext.isCurrent());
	EXPECT_FLOAT_EQ(61.25f, currentContext.get(AirmassConsumer::InjectionPhase));
	EXPECT_FLOAT_EQ(61.25f, currentContext.getSelectedLoad(AFR_Tps));
	EXPECT_FLOAT_EQ(92.0f, publishedContext.get(AirmassConsumer::InjectionPhase));
	EXPECT_FLOAT_EQ(92.0f, getAirmassConsumerLoad(AirmassConsumer::InjectionPhase));

	(void)captureConsumerInputs(LM_SPEED_DENSITY);
	EXPECT_FALSE(currentContext.isCurrent());
	EXPECT_FLOAT_EQ(61.25f, currentContext.get(AirmassConsumer::InjectionPhase));
}

TEST(AirmassConsumerInfrastructure, StaleCandidateCannotReplaceNewerPublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->injectionPhaseLoadSource = AFR_Tps;
	const auto firstCandidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(firstCandidate));
	const AirmassConsumerLoadContext firstContext(AirmassConsumer::InjectionPhase);

	const auto staleCandidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	config->injectionPhaseLoadSource = AFR_AccPedal;
	const auto newerCandidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(newerCandidate));
	EXPECT_FALSE(publish(staleCandidate));
	EXPECT_FALSE(firstContext.isCurrent());
	EXPECT_FLOAT_EQ(41.5f, getAirmassConsumerLoad(AirmassConsumer::InjectionPhase));
}

TEST(AirmassConsumerInfrastructure, ConfigurationWriteAndStopInvalidatePublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	config->injectionPhaseLoadSource = AFR_Tps;
	const auto candidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(candidate));
	ASSERT_TRUE(engine->engineState.airmassLoads.Valid);

	engine->airmassInjectionState.onConfigurationWrite(LM_SPEED_DENSITY, false);
	EXPECT_FALSE(engine->engineState.airmassLoads.Valid);
	EXPECT_FALSE(std::isfinite(getAirmassConsumerLoad(AirmassConsumer::InjectionPhase)));
	EXPECT_FLOAT_EQ(0, engine->outputChannels.injectionPhaseLoad);

	const auto stopCandidate = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	ASSERT_TRUE(publish(stopCandidate));
	invalidateAirmassConsumerLoads();
	EXPECT_FALSE(engine->engineState.airmassLoads.Valid);
	EXPECT_FALSE(publish(candidate));
}

TEST(AirmassConsumerInfrastructure, LambdaTargetRequirementFollowsConsumersNotIgnition) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->fuelClosedLoopCorrectionEnabled = false;
	engineConfiguration->lambdaProtectionEnable = false;
	const auto ignitionOnly = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	EXPECT_FALSE(ignitionOnly.Snapshot.LambdaTargetRequired);

	engineConfiguration->isInjectionEnabled = true;
	const auto injection = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	EXPECT_TRUE(injection.Snapshot.LambdaTargetRequired);
}

TEST(AirmassConsumerInfrastructure, LambdaTargetRequirementTracksProtectionAndLatches) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	configureConsumers();
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->lambdaProtectionEnable = true;
	engineConfiguration->lambdaProtectionMinRpm = 1000;
	engineConfiguration->lambdaProtectionMinLoad = 0;
	engineConfiguration->lambdaProtectionMinTps = 0;
	engineConfiguration->noFuelTrimAfterDfcoTime = 0;
	Sensor::setMockValue(SensorType::Lambda1, 1);
	setTimeNowUs(10e6);

	auto inputs = captureConsumerInputs(LM_SPEED_DENSITY);
	const auto active = prepareAirmassConsumerLoads(inputs, 0.3f);
	EXPECT_TRUE(active.Snapshot.LambdaTargetRequired);

	Sensor::setInvalidMockValue(SensorType::Lambda1);
	const auto noSample = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	EXPECT_FALSE(noSample.Snapshot.LambdaTargetRequired);

	Sensor::setMockValue(SensorType::Lambda1, 1);
	engineConfiguration->lambdaProtectionEnable = false;
	engine->lambdaMonitor.lambdaMonitorCut = true;
	const auto latched = prepareAirmassConsumerLoads(captureConsumerInputs(LM_SPEED_DENSITY), 0.3f);
	EXPECT_FALSE(latched.Snapshot.LambdaTargetRequired);
	EXPECT_TRUE(latched.Snapshot.EnabledConsumers & (1u << static_cast<unsigned>(AirmassConsumer::LambdaMonitor)));
}
