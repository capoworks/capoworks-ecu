#include "pch.h"

#include "can_msg_tx.h"
#include "emp_pump.h"
#include "fan_control.h"

namespace {
constexpr uint32_t Status1Id = 0x18FF0396;
constexpr uint32_t CommandId = 0x18EF96A3;

EmpPumpConfig validConfig() {
	EmpPumpConfig result;
	result.enabled = true;
	result.afterRunEnabled = true;
	result.powerHoldEnabled = true;
	result.runDuringCranking = true;
	result.closedLoopEnabled = true;
	result.canBus = 0;
	result.controllerAddress = 0x96;
	result.sourceAddress = 0xA3;
	result.stopDebounce100ms = 1;
	result.statusTimeoutSeconds = 3;
	result.batteryCutoff10 = 110;
	result.batteryResume10 = 120;
	result.minRpm = 1500;
	result.afterRunMinRpm = 1800;
	result.failsafeRpm = 3000;
	result.maxRpm = 6000;
	result.afterRunStartTemp = 95;
	result.afterRunStopTemp = 85;
	return result;
}

class EmpPumpTxMock final : public ICanTransmitMock {
public:
	void onTxFrame(
			uint32_t id,
			uint8_t dlc,
			bool extended,
			CanBusIndex bus,
			uint8_t d0,
			uint8_t d1,
			uint8_t d2,
			uint8_t,
			uint8_t,
			uint8_t,
			uint8_t,
			uint8_t) override {
		lastId = id;
		lastDlc = dlc;
		lastExtended = extended;
		lastBus = bus;
		data[0] = d0;
		data[1] = d1;
		data[2] = d2;
		if (count < 16) {
			ids[count] = id;
			controls[count] = d0;
			buses[count] = bus;
		}
		count++;
	}

	void onTx(uint32_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t) override {}

	uint32_t lastId = 0;
	uint8_t lastDlc = 0;
	bool lastExtended = false;
	CanBusIndex lastBus = CanBusIndex::Bus1;
	uint8_t data[3] = {};
	uint32_t ids[16] = {};
	uint8_t controls[16] = {};
	CanBusIndex buses[16] = {};
	int count = 0;
};

CANRxFrame status1Frame() {
	CANRxFrame frame = {};
	frame.IDE = CAN_IDE_EXT;
	frame.EID = Status1Id;
	frame.DLC = 8;
	frame.data8[1] = 0x70; // 3000 RPM encoded in 0.5 RPM units
	frame.data8[2] = 0x17;
	return frame;
}
} // namespace

TEST(EmpPump, RejectsWrongCanFrameAndDecodesExtendedStatus) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	dut.setConfiguration(validConfig());

	auto frame = status1Frame();
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus0, frame));
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus1, frame));
	frame.IDE = CAN_IDE_STD;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, frame));
	frame.IDE = CAN_IDE_EXT;
	frame.DLC = 7;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, frame));
	frame.DLC = 8;
	frame.EID = 0x18FF0397;
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, frame));

	frame = status1Frame();
	dut.processFrame(CanBusIndex::Bus0, frame, getTimeNowNt());
	auto status = dut.getStatus();
	EXPECT_EQ(3000, status.actualRpm);
	EXPECT_NE(0, status.capabilities & EmpPumpCapStatus1);
}

TEST(EmpPump, ServiceTestUsesExtendedCommandAndIsSafeByDefault) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	dut.setConfiguration(validConfig());
	EmpPumpTxMock tx;
	setCanTxMockHandler(&tx);

	// Apply configuration first: a burn resets runtime telemetry and any prior service request.
	dut.onSlowCallback();
	dut.pollTx(CanBusIndex::Bus0);
	tx.count = 0;
	EXPECT_TRUE(dut.handleServiceCommand(EmpPumpServiceCommand::Start));
	advanceTimeUs(100000);
	dut.onSlowCallback();
	EXPECT_EQ(0, tx.count);
	dut.pollTx(CanBusIndex::Bus0);

	EXPECT_EQ(1, tx.count);
	EXPECT_EQ(CommandId, tx.lastId);
	EXPECT_EQ(8, tx.lastDlc);
	EXPECT_TRUE(tx.lastExtended);
	EXPECT_EQ(CanBusIndex::Bus0, tx.lastBus);
	EXPECT_EQ(0xFD, tx.data[0]);
	EXPECT_EQ(0xA0, tx.data[1]); // 2000 RPM * 2, little endian
	EXPECT_EQ(0x0F, tx.data[2]);
	EXPECT_EQ(EmpPumpState::ServiceTest, dut.getStatus().state);

	EXPECT_TRUE(dut.handleServiceCommand(EmpPumpServiceCommand::Stop));
	advanceTimeUs(100000);
	dut.onSlowCallback();
	dut.pollTx(CanBusIndex::Bus0);
	EXPECT_EQ(2, tx.count);
	EXPECT_EQ(0xFC, tx.data[0]);

	setCanTxMockHandler(nullptr);
}

TEST(EmpPump, FullTxQueueDoesNotSendFromSlowCallback) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto empConfig = validConfig();
	empConfig.rampRpmPerSecond = 0;
	dut.setConfiguration(empConfig);
	EmpPumpTxMock tx;
	setCanTxMockHandler(&tx);

	dut.onSlowCallback();
	for (int i = 0; i < 12; i++) {
		ASSERT_TRUE(dut.handleServiceCommand(EmpPumpServiceCommand::Start, 2000 + i * 100, 5));
		advanceTimeUs(100000);
		dut.onSlowCallback();
	}
	EXPECT_EQ(0, tx.count);
	EXPECT_EQ(EmpPumpState::ServiceTest, dut.getStatus().state);
	EXPECT_EQ(3100, dut.getStatus().targetRpm);
	EXPECT_NE(0, dut.getStatus().faults & EmpPumpFaultTx);
	EXPECT_GT(dut.getStatus().transmitFailureCount, 0);

	for (int i = 0; i < 8; i++) {
		dut.pollTx(CanBusIndex::Bus0);
	}
	EXPECT_EQ(8, tx.count);
	advanceTimeUs(100000);
	dut.onSlowCallback();
	dut.pollTx(CanBusIndex::Bus0);
	EXPECT_EQ(9, tx.count);
	EXPECT_EQ(0x38, tx.data[1]); // Latest target: 3100 RPM * 2.
	EXPECT_EQ(0x18, tx.data[2]);

	setCanTxMockHandler(nullptr);
}

TEST(EmpPump, BootstrapPublishesActiveConfigurationForServiceWithoutBurn) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	// Do not call setConfiguration(): this models normal boot before any Burn.
	engineConfiguration->enableEmpPump = true;
	EmpPump dut;
	dut.onSlowCallback();

	const auto active = dut.getConfiguration();
	EXPECT_TRUE(active.enabled);
	EXPECT_EQ(engineConfiguration->empPump.controllerAddress, active.controllerAddress);
	EXPECT_TRUE(dut.handleServiceCommand(EmpPumpServiceCommand::Start));
}

TEST(EmpPump, BurnReleasesPowerHoldOnOldEndpoint) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto empConfig = validConfig();
	empConfig.hotBootRecovery = true;
	Sensor::setMockValue(SensorType::Clt, 105);
	Sensor::setMockValue(SensorType::BatteryVoltage, 13);
	EmpPumpTxMock tx;
	setCanTxMockHandler(&tx);
	dut.setConfiguration(empConfig);
	dut.onSlowCallback();
	EXPECT_EQ(0, tx.count);
	dut.pollTx(CanBusIndex::Bus0);
	ASSERT_GE(tx.count, 1);
	EXPECT_EQ(CommandId, tx.ids[tx.count - 1]);
	EXPECT_EQ(0xF5, tx.controls[tx.count - 1]);

	empConfig.canBus = 1;
	empConfig.controllerAddress = 0x97;
	empConfig.sourceAddress = 0xA4;
	dut.setConfiguration(empConfig);
	// Burn only publishes a mailbox: RX remains on the old endpoint until F0
	// has left the software TX queue on the old bus.
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	dut.onSlowCallback();
	EXPECT_EQ(1, tx.count);
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	dut.pollTx(CanBusIndex::Bus1);
	EXPECT_EQ(1, tx.count);
	dut.pollTx(CanBusIndex::Bus0);
	ASSERT_GE(tx.count, 2);
	EXPECT_EQ(CommandId, tx.ids[1]);
	EXPECT_EQ(CanBusIndex::Bus0, tx.buses[1]);
	EXPECT_EQ(0xF0, tx.controls[1]);
	dut.onSlowCallback();
	dut.pollTx(CanBusIndex::Bus1);
	// Emulate a Bus0 frame that was accepted before the switch but reaches
	// decode afterward. It must not publish old-generation telemetry.
	dut.decodeFrame(CanBusIndex::Bus0, status1Frame(), getTimeNowNt());
	EXPECT_EQ(0, dut.getStatus().actualRpm);
	CANRxFrame newEndpointFrame = status1Frame();
	newEndpointFrame.EID = 0x18FF0397;
	newEndpointFrame.data8[1] = 0x40; // 4000 RPM, little endian / 2
	newEndpointFrame.data8[2] = 0x1f;
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus1, newEndpointFrame));
	dut.processFrame(CanBusIndex::Bus1, newEndpointFrame, getTimeNowNt());
	EXPECT_EQ(4000, dut.getStatus().actualRpm);
	for (int i = 2; i < tx.count && i < 8; i++) {
		EXPECT_EQ(CanBusIndex::Bus1, tx.buses[i]);
	}
	setCanTxMockHandler(nullptr);
}

TEST(EmpPump, BurnWaitsForFullOldBusQueueBeforeSwitching) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto empConfig = validConfig();
	empConfig.hotBootRecovery = true;
	empConfig.rampRpmPerSecond = 0;
	Sensor::setMockValue(SensorType::Clt, 105);
	Sensor::setMockValue(SensorType::BatteryVoltage, 13);
	EmpPumpTxMock tx;
	setCanTxMockHandler(&tx);
	dut.setConfiguration(empConfig);
	dut.onSlowCallback();
	for (int i = 0; i < 7; i++) {
		advanceTimeUs(500000);
		dut.onSlowCallback();
	}
	EXPECT_EQ(0, tx.count);

	empConfig.canBus = 1;
	empConfig.controllerAddress = 0x97;
	dut.setConfiguration(empConfig);
	dut.onSlowCallback();
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	EXPECT_EQ(0, tx.count);
	dut.pollTx(CanBusIndex::Bus0);
	dut.onSlowCallback();
	EXPECT_TRUE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	for (int i = 0; i < 8; i++) {
		dut.pollTx(CanBusIndex::Bus0);
	}
	ASSERT_EQ(9, tx.count);
	EXPECT_EQ(0xF0, tx.controls[8]);
	EXPECT_EQ(CanBusIndex::Bus0, tx.buses[8]);
	dut.onSlowCallback();
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	dut.pollTx(CanBusIndex::Bus1);
	EXPECT_EQ(CanBusIndex::Bus1, tx.lastBus);
	setCanTxMockHandler(nullptr);
}

TEST(EmpPump, Status2OperationDoesNotCreateHvilFault) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	dut.setConfiguration(validConfig());
	dut.onSlowCallback();

	CANRxFrame status2 = {};
	status2.IDE = CAN_IDE_EXT;
	status2.EID = 0x18FF2396;
	status2.DLC = 8;
	status2.data8[0] = 0x40; // command source external
	status2.data8[7] = 0x04; // operation=1, which occupies summary bit 6
	dut.processFrame(CanBusIndex::Bus0, status2, getTimeNowNt());
	advanceTimeUs(100000);
	dut.onSlowCallback();
	auto status = dut.getStatus();
	EXPECT_NE(0, status.faults & EmpPumpFaultDerated);
	EXPECT_EQ(0, status.faults & EmpPumpFaultHvil);

	CANRxFrame status3 = {};
	status3.IDE = CAN_IDE_EXT;
	status3.EID = 0x18FF2496;
	status3.DLC = 5;
	status3.data8[4] = 1;
	dut.processFrame(CanBusIndex::Bus0, status3, getTimeNowNt());
	CANRxFrame externalTemp = {};
	externalTemp.IDE = CAN_IDE_EXT;
	externalTemp.EID = 0x18FF4396;
	externalTemp.DLC = 3;
	externalTemp.data8[1] = 0x34;
	externalTemp.data8[2] = 0x12;
	dut.processFrame(CanBusIndex::Bus0, externalTemp, getTimeNowNt());
	CANRxFrame addressClaim = {};
	addressClaim.IDE = CAN_IDE_EXT;
	addressClaim.EID = 0x18EEFF96;
	dut.processFrame(CanBusIndex::Bus0, addressClaim, getTimeNowNt());
	advanceTimeUs(100000);
	dut.onSlowCallback();
	status = dut.getStatus();
	EXPECT_NE(0, status.faults & EmpPumpFaultHvil);
	EXPECT_EQ(0x1234, status.externalTemperatureRaw);
	EXPECT_NE(0, status.capabilities & EmpPumpCapStatus3);
	EXPECT_NE(0, status.capabilities & EmpPumpCapExternalTemperature);
	EXPECT_NE(0, status.capabilities & EmpPumpCapAddressClaim);
}

TEST(EmpPump, HotBootAfterRunHonorsBatteryCutoffAndResume) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto empConfig = validConfig();
	empConfig.hotBootRecovery = true;
	empConfig.rampRpmPerSecond = 0;
	Sensor::setMockValue(SensorType::Clt, 105);
	Sensor::setMockValue(SensorType::BatteryVoltage, 10.5f);
	dut.setConfiguration(empConfig);
	dut.onSlowCallback();
	auto status = dut.getStatus();
	EXPECT_EQ(EmpPumpState::AfterRun, status.state);
	EXPECT_EQ(0, status.targetRpm);
	EXPECT_NE(0, status.faults & EmpPumpFaultBatteryLow);

	Sensor::setMockValue(SensorType::BatteryVoltage, 12.5f);
	advanceTimeUs(100000);
	dut.onSlowCallback();
	EXPECT_EQ(EmpPumpState::AfterRun, dut.getStatus().state);
	EXPECT_GE(dut.getStatus().targetRpm, empConfig.afterRunMinRpm);
}

TEST(EmpPump, ActiveServiceReportsStatusTimeoutWithoutStatusFrame) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	dut.setConfiguration(validConfig());
	EXPECT_TRUE(dut.handleServiceCommand(EmpPumpServiceCommand::Start));
	dut.onSlowCallback();
	EXPECT_EQ(EmpPumpState::ServiceTest, dut.getStatus().state);
	advanceTimeUs(3100000);
	EXPECT_NE(0, dut.getStatus().faults & EmpPumpFaultStatusTimeout);
}

TEST(EmpPump, AppliesMinimumFlowRampAndFanCapacityProtection) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto empConfig = validConfig();
	empConfig.rampRpmPerSecond = 1000;
	empConfig.kp = 2000;
	empConfig.targetTemp = 90;
	Sensor::setMockValue(SensorType::Clt, 90);
	Sensor::setMockValue(SensorType::BatteryVoltage, 13);
	Sensor::setMockValue(SensorType::Rpm, 5000);
	engine->rpmCalculator.setRpmValue(5000);
	dut.setConfiguration(empConfig);
	dut.onSlowCallback();

	// At the target temperature, the PI controller still enforces the
	// engine-RPM dependent minimum flow (1800 RPM at 5000 engine RPM in this
	// test table).
	auto status = dut.getStatus();
	EXPECT_EQ(1800, status.minimumFlowRpm);
	EXPECT_EQ(1800, status.targetRpm);

	// A conventional relay fan is full capacity while on.  The thermal demand
	// saturates, but the command itself ramps by 100 RPM per 100 ms.
	engine->module<FanControl1>()->m_state = true;
	Sensor::setMockValue(SensorType::Clt, 105);
	advanceTimeUs(100000);
	dut.onSlowCallback();
	status = dut.getStatus();
	EXPECT_EQ(1900, status.targetRpm);
	EXPECT_EQ(EmpPumpThermalState::CapacityLimited, status.thermalState);
	EXPECT_NE(0, status.controlFlags & (1 << 7));
}

TEST(EmpPump, InvalidConfigurationStaysDisabled) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto invalidConfig = validConfig();
	invalidConfig.controllerAddress = invalidConfig.sourceAddress;
	dut.setConfiguration(invalidConfig);
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
	dut.onSlowCallback();
	EXPECT_EQ(EmpPumpState::Disabled, dut.getStatus().state);
}

TEST(EmpPump, RejectsInvalidSafetyInvariantCombinations) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	auto invalidConfig = validConfig();
	invalidConfig.batteryResume10 = invalidConfig.batteryCutoff10 - 1;
	dut.setConfiguration(invalidConfig);
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));

	invalidConfig = validConfig();
	invalidConfig.engineRpmBins[2] = invalidConfig.engineRpmBins[1];
	dut.setConfiguration(invalidConfig);
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));

	invalidConfig = validConfig();
	invalidConfig.coolingLimitedDelta = invalidConfig.overloadDelta + 1;
	dut.setConfiguration(invalidConfig);
	EXPECT_FALSE(dut.acceptFrame(CanBusIndex::Bus0, status1Frame()));
}

TEST(EmpPump, ServiceTestIsRejectedWhileEngineRunsWithoutExplicitPermission) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EmpPump dut;
	dut.setConfiguration(validConfig());
	engine->rpmCalculator.setRpmValue(1200);
	EXPECT_FALSE(dut.handleServiceCommand(EmpPumpServiceCommand::Start));
	EXPECT_EQ(EmpPumpState::Disabled, dut.getStatus().state);
}
