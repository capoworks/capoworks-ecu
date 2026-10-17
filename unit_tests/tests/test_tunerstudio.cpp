#include "pch.h"
#include "airmass_injection_state.h"
#include "tunerstudio.h"
#include "tunerstudio_io.h"
#include "status_loop.h"
#include "limp_manager.h"

static uint8_t st5TestBuffer[16000];

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
