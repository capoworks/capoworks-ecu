#include "pch.h"

#include "can_msg_tx.h"
#include "shock_preload.h"

class ShockPreloadTxMock final : public ICanTransmitMock {
public:
	void onTxFrame(
			uint32_t id,
			uint8_t dlc,
			bool isExtended,
			CanBusIndex bus,
			uint8_t d0,
			uint8_t d1,
			uint8_t d2,
			uint8_t d3,
			uint8_t d4,
			uint8_t d5,
			uint8_t d6,
			uint8_t d7) override {
		lastBus = bus;
		lastExtended = isExtended;
		onTx(id, dlc, d0, d1, d2, d3, d4, d5, d6, d7);
	}

	void
	onTx(uint32_t id,
		 uint8_t dlc,
		 uint8_t d0,
		 uint8_t d1,
		 uint8_t d2,
		 uint8_t d3,
		 uint8_t d4,
		 uint8_t d5,
		 uint8_t d6,
		 uint8_t d7) override {
		lastId = id;
		lastDlc = dlc;
		data[0] = d0;
		data[1] = d1;
		data[2] = d2;
		data[3] = d3;
		data[4] = d4;
		data[5] = d5;
		data[6] = d6;
		data[7] = d7;
		count++;
	}

	uint32_t lastId = 0;
	uint8_t lastDlc = 0;
	CanBusIndex lastBus = CanBusIndex::Any;
	bool lastExtended = false;
	uint8_t data[8] = {};
	int count = 0;
};

static void enableShockPreload() {
	engineConfiguration->enableShockPreload = true;
	engineConfiguration->canReadEnabled = true;
	engineConfiguration->canWriteEnabled = true;
}

TEST(ShockPreload, DecodesStatusAndAlarmFrames) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	enableShockPreload();
	ShockPreload dut;
	EXPECT_FALSE(dut.isOnline());
	EXPECT_FALSE(dut.isAtSavedPreset());

	CANRxFrame status = {};
	status.IDE = false;
	status.SID = 0x721;
	status.DLC = 8;
	status.data8[0] = 42;
	status.data8[2] = 55;
	status.data8[4] = 3;
	status.data8[5] = 7;
	status.data8[6] = 0x34;
	status.data8[7] = 0x12;
	dut.processFrame(CanBusIndex::Bus1, status, getTimeNowNt());

	EXPECT_EQ(42, dut.getPosition());
	EXPECT_EQ(55, dut.getControllerTarget());
	EXPECT_EQ(3, dut.getSelectedPreset());
	EXPECT_EQ(7, dut.getState());
	EXPECT_EQ(0x1234, dut.getAlarms());
	EXPECT_TRUE(dut.isOnline());
	EXPECT_FALSE(dut.isAtSavedPreset());

	CANRxFrame preset = {};
	preset.IDE = false;
	preset.SID = 0x721;
	preset.DLC = 8;
	preset.data8[0] = 0xF0;
	preset.data8[1] = 2;
	preset.data8[2] = 42;
	preset.data8[3] = 2;
	preset.data8[4] = 43; // Match preset 2 within the one-percent tolerance.
	preset.data8[5] = 44;
	dut.processFrame(CanBusIndex::Bus1, preset, getTimeNowNt());
	EXPECT_EQ(2, dut.getSelectedPreset());
	EXPECT_EQ(43, dut.getPosition());
	EXPECT_TRUE(dut.isAtSavedPreset());
	EXPECT_TRUE(dut.isPresetActive(2));
	EXPECT_FALSE(dut.isPresetActive(1));

	CANRxFrame alarm = {};
	alarm.IDE = false;
	alarm.SID = 0x722;
	alarm.DLC = 8;
	alarm.data8[0] = 0xcd;
	alarm.data8[1] = 0xab;
	alarm.data8[2] = 101; // Clamp percentage diagnostics to their protocol range.
	alarm.data8[4] = 63;
	alarm.data8[6] = 0x78;
	alarm.data8[7] = 0x56;
	dut.processFrame(CanBusIndex::Bus1, alarm, getTimeNowNt());

	EXPECT_EQ(100, dut.getPosition());
	EXPECT_EQ(63, dut.getControllerTarget());
	EXPECT_EQ(0xabcd, dut.getAlarms());
	EXPECT_EQ(0x5678, dut.getCurrent());
	EXPECT_FALSE(dut.isAtSavedPreset());
	EXPECT_FALSE(dut.isPresetActive(2));

	preset.data8[3] = 0xff;
	dut.processFrame(CanBusIndex::Bus1, preset, getTimeNowNt());
	EXPECT_EQ(0xff, dut.getSelectedPreset());
	EXPECT_FALSE(dut.isPresetActive(2));
}

TEST(ShockPreload, AcceptsOnlySecondaryStandardProtocolFrames) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	enableShockPreload();
	ShockPreload dut;
	CANRxFrame frame = {};
	frame.DLC = 8;
	frame.IDE = false;
	frame.SID = 0x721;

	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus1, frame));
	frame.SID = 0x720;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus1, frame));
	frame.SID = 0x721;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, frame));
	frame.IDE = true;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus1, frame));
	frame.IDE = false;
	frame.DLC = 7;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus1, frame));
}

TEST(ShockPreload, SendsSpecifiedCommandsAndPolls) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	enableShockPreload();
	ShockPreload dut;
	ShockPreloadTxMock tx;
	setCanTxMockHandler(&tx);

	dut.handleTsCommand(SHOCK_PRELOAD_MOVE_DOWN_2);
	EXPECT_EQ(1, tx.count);
	EXPECT_EQ(0x720, tx.lastId);
	EXPECT_EQ(CanBusIndex::Bus1, tx.lastBus);
	EXPECT_FALSE(tx.lastExtended);
	EXPECT_EQ(8, tx.lastDlc);
	EXPECT_EQ(0x06, tx.data[0]);
	EXPECT_EQ(0xfe, tx.data[1]);
	EXPECT_EQ(0xff, tx.data[2]);
	EXPECT_THAT(tx.data, ::testing::ElementsAre(0x06, 0xfe, 0xff, 0, 0, 0, 0, 0));

	dut.handleTsCommand(SHOCK_PRELOAD_LOAD_SLOT_4);
	EXPECT_EQ(2, tx.count);
	EXPECT_EQ(8, tx.lastDlc);
	EXPECT_EQ(0x02, tx.data[0]);
	EXPECT_EQ(4, tx.data[1]);
	EXPECT_THAT(tx.data, ::testing::ElementsAre(0x02, 4, 0, 0, 0, 0, 0, 0));
	EXPECT_EQ(4, dut.getSelectedPreset());

	CANRxFrame status = {};
	status.IDE = false;
	status.SID = 0x721;
	status.DLC = 8;
	status.data8[0] = 42;
	dut.processFrame(CanBusIndex::Bus1, status, getTimeNowNt());
	dut.handleTsCommand(SHOCK_PRELOAD_SAVE_SLOT_1);
	EXPECT_EQ(3, tx.count);
	EXPECT_EQ(0x03, tx.data[0]);
	EXPECT_EQ(1, tx.data[1]);
	EXPECT_EQ(1, dut.getSelectedPreset());
	EXPECT_TRUE(dut.isAtSavedPreset());
	EXPECT_TRUE(dut.isPresetActive(1));
	dut.handleTsCommand(SHOCK_PRELOAD_LOAD_SLOT_1);
	EXPECT_EQ(4, tx.count);
	EXPECT_EQ(0x02, tx.data[0]);
	EXPECT_EQ(1, tx.data[1]);
	EXPECT_EQ(1, dut.getSelectedPreset());

	engineConfiguration->shockPreloadCommandTarget = 73;
	dut.handleTsCommand(SHOCK_PRELOAD_SET_TARGET);
	EXPECT_EQ(5, tx.count);
	EXPECT_EQ(8, tx.lastDlc);
	EXPECT_EQ(0x01, tx.data[0]);
	EXPECT_EQ(73, tx.data[1]);
	EXPECT_EQ(0, tx.data[2]);
	EXPECT_THAT(tx.data, ::testing::ElementsAre(0x01, 73, 0, 0, 0, 0, 0, 0));

	dut.onSlowCallback();
	EXPECT_EQ(5, tx.count);
	dut.pollStatus();
	EXPECT_EQ(6, tx.count);
	EXPECT_EQ(8, tx.lastDlc);
	EXPECT_EQ(0x10, tx.data[0]);
	EXPECT_THAT(tx.data, ::testing::ElementsAre(0x10, 0, 0, 0, 0, 0, 0, 0));
	advanceTimeUs(999000);
	dut.pollStatus();
	EXPECT_EQ(6, tx.count);
	advanceTimeUs(2000);
	dut.pollStatus();
	EXPECT_EQ(7, tx.count);

	setCanTxMockHandler(nullptr);
}

TEST(ShockPreload, DisabledControllerDoesNotTransmit) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	ShockPreload dut;
	ShockPreloadTxMock tx;
	setCanTxMockHandler(&tx);

	dut.handleTsCommand(SHOCK_PRELOAD_REQUEST_STATUS);
	dut.onSlowCallback();
	dut.pollStatus();
	EXPECT_EQ(0, tx.count);

	setCanTxMockHandler(nullptr);
}

TEST(ShockPreload, TimesOutAfterThreeSeconds) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	enableShockPreload();
	ShockPreload dut;
	CANRxFrame status = {};
	status.IDE = false;
	status.SID = 0x721;
	status.DLC = 8;
	dut.processFrame(CanBusIndex::Bus1, status, getTimeNowNt());
	EXPECT_TRUE(dut.isOnline());

	advanceTimeUs(2999000);
	EXPECT_TRUE(dut.isOnline());
	advanceTimeUs(2000);
	EXPECT_FALSE(dut.isOnline());
}
