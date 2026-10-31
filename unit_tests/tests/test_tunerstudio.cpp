#include "pch.h"
#include "airmass_injection_state.h"
#include "tunerstudio.h"
#include "tunerstudio_io.h"
#include "status_loop.h"
#include "limp_manager.h"
#include "ve_analyze_gate.h"

#include <cmath>

namespace {
template <typename Reason>
uint32_t reasonBit(Reason reason) {
	return static_cast<uint32_t>(reason);
}

void expectGateReason(const output_channels_s& output, VeAnalyzeGateReason reason) {
	EXPECT_NE(0u, output.veAnalyzeSampleRejectReasons & reasonBit(reason));
}

void expectOperatingReason(const output_channels_s& output, VeAnalyzeOperatingReason reason) {
	EXPECT_NE(0u, output.veAnalyzeSampleRejectReasons & reasonBit(reason));
}

class VeAnalyzeIntegrationFixture {
public:
	VeAnalyzeIntegrationFixture()
		: eth(engine_type_e::TEST_ENGINE) {
		engineConfiguration->isInjectionEnabled = true;
		engineConfiguration->isIgnitionEnabled = true;
		engine->rpmCalculator.setRpmValue(1500);
		engine->engineState.airmassCalculationValid = true;
		engine->engineState.tpsAccelEnrich = 0;
		Sensor::setMockValue(SensorType::Tps1, 20);
		engine->engineState.resetVeAnalyzeSampleGate(getTimeNowNt());
	}

	void observe(float tps, bool updateSensor = true) {
		advanceTimeUs(4'000);
		if (updateSensor) {
			Sensor::setMockValue(SensorType::Tps1, tps);
		}
		engine->engineState.updateVeAnalyzeSampleGate(getTimeNowNt());
	}

	void feed(int count, float tps = 20) {
		for (int index = 0; index < count; index++) {
			observe(tps);
		}
	}

	void warmToReady() {
		feed(400);
	}

	void exportState() {
		updateTunerStudioState();
	}

	EngineTestHelper eth;
};
} // namespace

static uint8_t st5TestBuffer[16000];

TEST(VeAnalyzeSampleGateIntegration, tpsMovementIsIndependentOfAeLookback) {
	for (const float destination : {25.0f, 15.0f}) {
		VeAnalyzeIntegrationFixture fixture;
		fixture.warmToReady();
		fixture.exportState();
		ASSERT_TRUE(engine->outputChannels.veAnalyzeSampleReady);

		engine->engineState.tpsAccelEnrich = 0;
		fixture.observe(destination);
		fixture.exportState();

		EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
		EXPECT_GT(engine->outputChannels.veAnalyzeTpsRateAbs, 5.0f);
		EXPECT_FLOAT_EQ(0, engine->outputChannels.veAnalyzeTimeSinceMovement);
		expectGateReason(engine->outputChannels, VeAnalyzeGateReason::TpsMovementRecovery);
	}
}

TEST(VeAnalyzeSampleGateIntegration, shortOutAndBackSurvivesSlowPolling) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.observe(25);
	fixture.observe(20);

	for (const int pollDelayMs : {20, 20}) {
		advanceTimeUs(pollDelayMs * 1'000);
		fixture.exportState();
		EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
		expectGateReason(engine->outputChannels, VeAnalyzeGateReason::TpsMovementRecovery);
	}
}

TEST(VeAnalyzeSampleGateIntegration, invalidTpsIsNotAcceptedAsZero) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.exportState();
	const uint32_t acceptedBefore = engine->outputChannels.veAnalyzeAcceptedObservations;

	Sensor::setInvalidMockValue(SensorType::Tps1);
	fixture.observe(0, false);
	fixture.exportState();

	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeRateValid);
	EXPECT_EQ(acceptedBefore, engine->outputChannels.veAnalyzeAcceptedObservations);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::InvalidTps);
	expectOperatingReason(engine->outputChannels, VeAnalyzeOperatingReason::InvalidSensor);
}

TEST(VeAnalyzeSampleGateIntegration, stagnantProviderFailsFreshnessAfterTolerance) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();

	advanceTimeUs(56'000);
	engine->engineState.updateVeAnalyzeSampleGate(getTimeNowNt());
	fixture.exportState();

	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeRateValid);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::StaleTps);
	EXPECT_GT(engine->outputChannels.veAnalyzeRejectedObservations, 0u);
}

TEST(VeAnalyzeSampleGateIntegration, invalidRpmFuelAndCrankingHaveExplicitReasons) {
	VeAnalyzeIntegrationFixture fixture;
	engine->rpmCalculator.setRpmValue(0);
	engine->engineState.airmassCalculationValid = false;
	fixture.observe(20);
	fixture.exportState();

	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	expectOperatingReason(engine->outputChannels, VeAnalyzeOperatingReason::InvalidRpm);
	expectOperatingReason(engine->outputChannels, VeAnalyzeOperatingReason::InvalidFuelCalculation);

	engine->rpmCalculator.setRpmValue(300);
	engine->engineState.airmassCalculationValid = true;
	fixture.observe(20);
	fixture.exportState();

	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	expectOperatingReason(engine->outputChannels, VeAnalyzeOperatingReason::Cranking);
}

TEST(VeAnalyzeSampleGateIntegration, freshStationarySamplesBecomeReadyAndExportFiniteUnits) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.exportState();

	EXPECT_TRUE(engine->outputChannels.veAnalyzeRateValid);
	EXPECT_TRUE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTpsRateAbs));
	EXPECT_NEAR(0, engine->outputChannels.veAnalyzeTpsRateAbs, 0.001f);
	EXPECT_GT(engine->outputChannels.veAnalyzeAcceptedObservations, 0u);
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceMovement));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceCorrection));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeRecoveryRemaining));
	EXPECT_FLOAT_EQ(655.35f, engine->outputChannels.veAnalyzeTimeSinceMovement);
	EXPECT_FLOAT_EQ(655.35f, engine->outputChannels.veAnalyzeTimeSinceCorrection);
	EXPECT_FLOAT_EQ(0, engine->outputChannels.veAnalyzeRecoveryRemaining);

	const uint32_t acceptedBefore = engine->outputChannels.veAnalyzeAcceptedObservations;
	const uint32_t rejectedBefore = engine->outputChannels.veAnalyzeRejectedObservations;
	fixture.observe(20);
	fixture.exportState();
	EXPECT_EQ(acceptedBefore + 1, engine->outputChannels.veAnalyzeAcceptedObservations);
	EXPECT_EQ(rejectedBefore, engine->outputChannels.veAnalyzeRejectedObservations);
}

TEST(VeAnalyzeSampleGateIntegration, synchronizationLossClearsAndRequiresWindowHold) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.exportState();
	ASSERT_TRUE(engine->outputChannels.veAnalyzeSampleReady);

	engine->OnTriggerSynchronizationLost();
	fixture.exportState();
	EXPECT_EQ(0u, engine->outputChannels.veAnalyzeAcceptedObservations);
	EXPECT_EQ(0u, engine->outputChannels.veAnalyzeRejectedObservations);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::ResetRecovery);

	engine->rpmCalculator.setRpmValue(1500);
	engine->engineState.airmassCalculationValid = true;
	fixture.feed(200);
	fixture.exportState();
	EXPECT_TRUE(engine->outputChannels.veAnalyzeRateValid);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::ResetRecovery);

	fixture.feed(200);
	fixture.exportState();
	EXPECT_TRUE(engine->outputChannels.veAnalyzeSampleReady);
}

TEST(VeAnalyzeSampleGateIntegration, fuelCorrectionTailRenewsWhileActive) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.exportState();
	const uint32_t acceptedBefore = engine->outputChannels.veAnalyzeAcceptedObservations;
	const uint32_t rejectedBefore = engine->outputChannels.veAnalyzeRejectedObservations;

	engine->engineState.tpsAccelEnrich = 0.02f;
	fixture.observe(20);
	fixture.exportState();
	EXPECT_EQ(acceptedBefore, engine->outputChannels.veAnalyzeAcceptedObservations);
	EXPECT_EQ(rejectedBefore + 1, engine->outputChannels.veAnalyzeRejectedObservations);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_NEAR(1.5f, engine->outputChannels.veAnalyzeRecoveryRemaining, 0.01f);
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceMovement));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceCorrection));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeRecoveryRemaining));
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::FuelCorrectionRecovery);

	fixture.feed(250);
	fixture.exportState();
	EXPECT_FLOAT_EQ(0, engine->outputChannels.veAnalyzeTimeSinceCorrection);
	EXPECT_NEAR(1.5f, engine->outputChannels.veAnalyzeRecoveryRemaining, 0.01f);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::FuelCorrectionRecovery);

	engine->engineState.tpsAccelEnrich = 0;
	fixture.observe(20);
	fixture.feed(350);
	fixture.exportState();
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_LE(engine->outputChannels.veAnalyzeRecoveryRemaining, 0.11f);

	fixture.feed(50);
	fixture.exportState();
	EXPECT_TRUE(engine->outputChannels.veAnalyzeSampleReady);
}

TEST(VeAnalyzeSampleGateIntegration, cutTailRenewsAndUsesCutHold) {
	VeAnalyzeIntegrationFixture fixture;
	fixture.warmToReady();
	fixture.exportState();
	const uint32_t acceptedBefore = engine->outputChannels.veAnalyzeAcceptedObservations;
	const uint32_t rejectedBefore = engine->outputChannels.veAnalyzeRejectedObservations;
	auto& limpManager = engine->module<LimpManager>().unmock();

	engineConfiguration->isInjectionEnabled = false;
	limpManager.updateState(1500, getTimeNowNt());
	fixture.observe(20);
	fixture.exportState();
	EXPECT_EQ(acceptedBefore, engine->outputChannels.veAnalyzeAcceptedObservations);
	EXPECT_EQ(rejectedBefore + 1, engine->outputChannels.veAnalyzeRejectedObservations);
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	EXPECT_NEAR(2.0f, engine->outputChannels.veAnalyzeRecoveryRemaining, 0.01f);
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceMovement));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeTimeSinceCorrection));
	EXPECT_TRUE(std::isfinite(engine->outputChannels.veAnalyzeRecoveryRemaining));
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::FuelCutRecovery);
	expectOperatingReason(engine->outputChannels, VeAnalyzeOperatingReason::FuelCut);

	fixture.feed(450);
	fixture.exportState();
	EXPECT_NEAR(2.0f, engine->outputChannels.veAnalyzeRecoveryRemaining, 0.01f);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::FuelCutRecovery);

	engineConfiguration->isInjectionEnabled = true;
	limpManager.updateState(1500, getTimeNowNt());
	fixture.observe(20);
	fixture.feed(450);
	fixture.exportState();
	EXPECT_FALSE(engine->outputChannels.veAnalyzeSampleReady);
	expectGateReason(engine->outputChannels, VeAnalyzeGateReason::FuelCutRecovery);

	fixture.feed(50);
	fixture.exportState();
	EXPECT_TRUE(engine->outputChannels.veAnalyzeSampleReady);
}

TEST(VeAnalyzeTelemetry, crankingAndWallModelDisable) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	WallFuelController wallController;
	engine->engineModules.get<WallFuelController>().set(&wallController);

	engine->rpmCalculator.setRpmValue(300);
	Sensor::setMockValue(SensorType::Rpm, 300);
	updateTunerStudioState();
	EXPECT_EQ(engine->outputChannels.veAnalyzeIsCranking, 1);

	engine->rpmCalculator.setRpmValue(1500);
	Sensor::setMockValue(SensorType::Rpm, 1500);
	engineConfiguration->wwaeTau = 0.3f;
	engineConfiguration->wwaeBeta = 0.3f;
	wallController.onFastCallback();
	engine->injectionEvents.elements[0].getWallFuel().wallFuelCorrection = -0.001f;
	updateTunerStudioState();
	EXPECT_EQ(engine->outputChannels.veAnalyzeIsCranking, 0);
	EXPECT_NEAR(engine->outputChannels.wallFuelCorrectionValue, -1.0f, 0.01f);

	// A previous correction must not keep filtering samples after the model is disabled.
	engineConfiguration->wwaeTau = 0;
	wallController.onFastCallback();
	updateTunerStudioState();
	EXPECT_FLOAT_EQ(engine->outputChannels.wallFuelCorrectionValue, 0);
}

TEST(VeAnalyzeTelemetry, cutRecoveryResetProgressAndSaturation) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->rpmCalculator.setRpmValue(1500);
	Sensor::setMockValue(SensorType::Rpm, 1500);
	auto& limp = engine->module<LimpManager>();

	advanceTimeUs(10e6);
	// A transient Lua spark cut exercises the same ECU recovery timer as limiter cuts.
	engine->engineState.lua.luaIgnCut = true;
	limp->updateState(1500, getTimeNowNt());
	updateTunerStudioState();
	EXPECT_FLOAT_EQ(engine->outputChannels.veAnalyzeTimeSinceCut, 0);

	advanceTimeUs(1.5e6);
	engine->engineState.lua.luaIgnCut = false;
	limp->updateState(1500, getTimeNowNt());
	updateTunerStudioState();
	EXPECT_NEAR(engine->outputChannels.veAnalyzeTimeSinceCut, 1.5f, 0.01f);
	advanceTimeUs(1e6);
	updateTunerStudioState();
	EXPECT_NEAR(engine->outputChannels.veAnalyzeTimeSinceCut, 2.5f, 0.01f);

	advanceTimeUs(700e6);
	updateTunerStudioState();
	// The timer's 32-bit tick horizon may be reached before the packed field's maximum.
	EXPECT_GT(engine->outputChannels.veAnalyzeTimeSinceCut, 2.0f);
	EXPECT_LE(engine->outputChannels.veAnalyzeTimeSinceCut, 655.35f);

	// The newer cut must restart the quiet period.
	engine->engineState.lua.luaIgnCut = true;
	limp->updateState(1500, getTimeNowNt());
	updateTunerStudioState();
	EXPECT_FLOAT_EQ(engine->outputChannels.veAnalyzeTimeSinceCut, 0);
	engine->engineState.lua.luaIgnCut = false;
	limp->updateState(1500, getTimeNowNt());
	advanceTimeUs(0.5e6);
	updateTunerStudioState();
	EXPECT_NEAR(engine->outputChannels.veAnalyzeTimeSinceCut, 0.5f, 0.01f);
}

class BufferTsChannel : public TsChannelBase {
public:
	BufferTsChannel()
		: TsChannelBase("Test") {}

	void write(const uint8_t* buffer, size_t size, bool /*isLastWriteInTransaction*/) override {
		memcpy(&st5TestBuffer[writeIdx], buffer, size);
		writeIdx += size;
	}

	size_t readTimeout(uint8_t* buffer, size_t size, int timeout) override {
		// nothing to do here
		return size;
	}

	void reset() {
		writeIdx = 0;
	}

	size_t writeIdx = 0;
};

#define PAYLOAD "123"
#define SIZE strlen(PAYLOAD)

static void assertCrcPacket(BufferTsChannel& dut) {
	ASSERT_EQ(dut.writeIdx, SIZE + 7);

	// todo: proper uint16 comparison
	ASSERT_EQ(st5TestBuffer[0], 0);
	ASSERT_EQ(st5TestBuffer[1], SIZE + 1);

	ASSERT_EQ(st5TestBuffer[2], TS_RESPONSE_OK);

	ASSERT_EQ(memcmp(&st5TestBuffer[3], PAYLOAD, SIZE), 0);

	// todo: proper uint32 comparison
	ASSERT_EQ(st5TestBuffer[6], 86);
	EXPECT_EQ(st5TestBuffer[7], 77);
	EXPECT_EQ(st5TestBuffer[8], 101);
	EXPECT_EQ(st5TestBuffer[9], 220);
}

TEST(binary, testWriteCrc) {
	BufferTsChannel test;

	// Small impl
	test.reset();
	test.copyAndWriteSmallCrcPacket((const uint8_t*)PAYLOAD, SIZE);
	assertCrcPacket(test);

	// Large impl
	test.reset();
	test.writeCrcPacketLocked((const uint8_t*)PAYLOAD, SIZE);
	assertCrcPacket(test);
}

TEST(TunerstudioCommands, writeChunkEngineConfig) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	::testing::NiceMock<MockTsChannel> channel;

	uint8_t* configBytes = reinterpret_cast<uint8_t*>(config);

	// Contains zero before the write
	configBytes[100] = 0;
	EXPECT_EQ(configBytes[100], 0);

	// two step - writes to the engineConfiguration section require a burn
	uint8_t val = 50;
	TunerStudio instance;
	instance.handleWriteChunkCommand(&channel, 100, 1, &val);

	EXPECT_EQ(configBytes[100], 50);
}

TEST(TunerstudioCommands, modeWriteClosesAdmissionAndStaleCalculation) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	::testing::NiceMock<MockTsChannel> channel;

	engineConfiguration->fuelAlgorithm = LM_ALPHA_N;
	Sensor::setMockValue(SensorType::Rpm, 1000);
	engine->engineState.airmassCalculationValid = true;

	auto& gate = engine->airmassInjectionState;
	auto stale = gate.beginCalculation(LM_ALPHA_N, 1000, engine->getGlobalConfigurationVersion());
	gate.completeCalculation(stale, true);
	ASSERT_TRUE(gate.allowInjection());

	stale = gate.beginCalculation(LM_ALPHA_N, 1000, engine->getGlobalConfigurationVersion());
	gate.acceptCalculation();
	ASSERT_TRUE(gate.isCalculationCurrent(stale));

	const auto generation = engine->airmassCalibration.Generation;
	const auto pending = gate.pendingCallbacks();
	constexpr size_t modeOffset = offsetof(engine_configuration_s, fuelAlgorithm);
	uint8_t newMode = static_cast<uint8_t>(LM_SPEED_DENSITY);
	ASSERT_NE(reinterpret_cast<const uint8_t*>(engineConfiguration)[modeOffset], newMode);

	TunerStudio instance;
	instance.handleWriteChunkCommand(&channel, static_cast<uint16_t>(modeOffset), 1, &newMode);

	EXPECT_EQ(engineConfiguration->fuelAlgorithm, LM_SPEED_DENSITY);
	EXPECT_EQ(engine->airmassCalibration.Generation, generation + 1);
	EXPECT_EQ(gate.pendingCallbacks(), pending);
	EXPECT_FALSE(gate.isCalculationCurrent(stale));

	gate.completeCalculation(stale, true);
	EXPECT_FALSE(gate.allowInjection());

	auto current = gate.beginCalculation(LM_SPEED_DENSITY, 1000, engine->getGlobalConfigurationVersion());
	gate.completeCalculation(current, true);
	EXPECT_TRUE(gate.allowInjection()) << "a fresh valid standalone publication recovers admission";
}

TEST(TunerstudioCommands, liveWriteInvalidatesCacheWithoutBurningVersion) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	::testing::NiceMock<MockTsChannel> channel;

	const auto generation = engine->airmassCalibration.Generation;
	const auto version = engine->getGlobalConfigurationVersion();
	constexpr size_t modeOffset = offsetof(engine_configuration_s, fuelAlgorithm);
	const auto mode = engineConfiguration->fuelAlgorithm;
	uint8_t value = 50;

	TunerStudio instance;
	instance.handleWriteChunkCommand(&channel, static_cast<uint16_t>(modeOffset + 1), 1, &value);

	EXPECT_EQ(reinterpret_cast<uint8_t*>(config)[modeOffset + 1], value);
	EXPECT_EQ(engineConfiguration->fuelAlgorithm, mode);
	EXPECT_EQ(engine->airmassCalibration.Generation, generation + 1);
	EXPECT_EQ(engine->getGlobalConfigurationVersion(), version);
}
