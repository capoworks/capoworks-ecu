/**
 * @file emp_pump.h
 * @brief Electronic coolant pump (EMP) controller using the Caponord J1939 protocol.
 *
 * Design and protocol are ported from Speeduino commit
 * df1df13547cccc9b4875227c28450d13b3f72872 ("emp pump").  The original
 * author has explicitly permitted reuse.  This is a native FOME adaptation:
 * CAN RX values are individual atomics because RX and EngineModule callbacks
 * execute in different threads.
 */

#pragma once

#include "can_listener.h"
#include "engine_module.h"

#include <atomic>
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

enum class EmpPumpState : uint8_t {
	Disabled,
	Stopped,
	EngineActive,
	AfterRun,
	ServiceTest,
};

enum class EmpPumpThermalState : uint8_t {
	Inactive,
	Warmup,
	ClosedLoop,
	CapacityLimited,
	Overload,
	AfterRun,
	Failsafe,
	Service,
};

enum EmpPumpFault : uint16_t {
	EmpPumpFaultStatusTimeout = 1 << 0,
	// Software TX queue overflow or a failure reported by a future driver hook.
	// CanTxMessage does not expose per-frame completion today.
	EmpPumpFaultTx = 1 << 1,
	EmpPumpFaultCltInvalid = 1 << 2,
	EmpPumpFaultBatteryLow = 1 << 3,
	EmpPumpFaultConfig = 1 << 4,
	EmpPumpFaultServiceRequired = 1 << 5,
	EmpPumpFaultDerated = 1 << 6,
	EmpPumpFaultNotOperable = 1 << 7,
	EmpPumpFaultCommandNotExternal = 1 << 8,
	EmpPumpFaultHvil = 1 << 9,
	EmpPumpFaultIatInvalid = 1 << 10,
	EmpPumpFaultCoolingLimited = 1 << 11,
	EmpPumpFaultThermalOverload = 1 << 12,
};

enum EmpPumpCapability : uint8_t {
	EmpPumpCapStatus1 = 1 << 0,
	EmpPumpCapStatus2 = 1 << 1,
	EmpPumpCapStatus3 = 1 << 2,
	EmpPumpCapExternalTemperature = 1 << 3,
	EmpPumpCapAddressClaim = 1 << 4,
};

enum class EmpPumpServiceCommand : uint8_t {
	Stop = 0,
	Start = 1,
	ClearFaults = 2,
};

/**
 * FOME-facing configuration.  Integration should map its persistent
 * empPump configuration block field-for-field to this structure.
 */
struct EmpPumpConfig {
	bool enabled = false;
	bool afterRunEnabled = false;
	bool powerHoldEnabled = false;
	bool runDuringCranking = false;
	bool hotBootRecovery = false;
	bool closedLoopEnabled = false;
	bool serviceAllowedWhileRunning = false;

	uint8_t canBus = 0;
	uint8_t controllerAddress = 0x96;
	uint8_t sourceAddress = 0xA3;
	uint8_t stopDebounce100ms = 0;
	uint8_t statusTimeoutSeconds = 2;
	uint8_t batteryCutoff10 = 105;
	uint8_t batteryResume10 = 115;
	uint8_t manualTestSeconds = 10;

	uint16_t minRpm = 1000;
	uint16_t maxRpm = 6000;
	uint16_t afterRunMinRpm = 1000;
	uint16_t failsafeRpm = 2500;
	uint16_t rampRpmPerSecond = 1000;
	uint16_t afterRunMaxSeconds = 120;
	uint16_t manualTestRpm = 2000;

	int16_t afterRunStartTemp = 100;
	int16_t afterRunStopTemp = 90;
	int16_t targetTemp = 95;
	int16_t iatReference = 20;
	uint8_t deadband = 1;
	uint16_t kp = 50;
	uint16_t ki = 5;
	uint16_t integralLimit = 1500;
	uint16_t derivative = 0;
	uint16_t loadFF = 0;
	uint16_t iatGain = 0;
	uint8_t airflowFull = 100;
	uint16_t airflowRelief = 0;
	uint8_t fanEq = 0;
	uint8_t coolingLimitedDelta = 5;
	uint8_t overloadDelta = 15;
	uint8_t overloadDelay = 10;

	int16_t temperatureBins[6] = {60, 70, 80, 90, 100, 110};
	uint16_t rpmBins[6] = {1000, 1200, 1600, 2400, 4000, 6000};
	uint16_t engineRpmBins[4] = {0, 2000, 5000, 10000};
	uint16_t minimumFlowRpmBins[4] = {1000, 1200, 1800, 2400};
};

struct EmpPumpStatus {
	EmpPumpState state = EmpPumpState::Disabled;
	EmpPumpThermalState thermalState = EmpPumpThermalState::Inactive;
	uint16_t faults = 0;
	uint8_t capabilities = 0;
	uint16_t targetRpm = 0;
	uint16_t actualRpm = 0;
	uint8_t actualPercentRaw = 0;
	uint8_t controllerStatus = 0;
	uint8_t statusSummary = 0;
	uint8_t hvilState = 0;
	uint16_t voltageRaw = 0;
	uint16_t currentRaw = 32000;
	uint16_t powerWatts = 0;
	uint16_t externalTemperatureRaw = 8736;
	uint16_t mainStatusAgeMs = UINT16_MAX;
	uint16_t afterRunRemainingSeconds = 0;
	uint8_t transmitFailureCount = 0;
	uint8_t lastCommandControl = 0xFC;
	uint8_t controlFlags = 0;
	int16_t filteredIat = 0;
	int8_t temperatureError = 0;
	int16_t coolantSlopePerMinute = 0;
	uint16_t minimumFlowRpm = 0;
	int16_t feedForwardRpm = 0;
	int16_t piCorrectionRpm = 0;
	uint8_t coolingDemandRaw = 0;
	uint16_t saturationSeconds = 0;
};

// A bounded seqlock-style mailbox for trivially-copyable snapshots. Writers
// never wait. Readers retry a few times and otherwise retain their previous
// value, so a higher-priority CAN RX thread can never starve the slow owner.
// Configuration has one serialized producer (the configuration/Burn path).
template <typename T>
class EmpPumpSnapshotMailbox {
public:
	EmpPumpSnapshotMailbox() {
		for (auto& byte : m_bytes) {
			byte.store(0, std::memory_order_relaxed);
		}
	}

	void publish(const T& value) {
		static_assert(std::is_trivially_copyable_v<T>);
		const auto* source = reinterpret_cast<const uint8_t*>(&value);
		m_sequence.fetch_add(1, std::memory_order_acq_rel);
		for (size_t i = 0; i < sizeof(T); i++) {
			m_bytes[i].store(source[i], std::memory_order_relaxed);
		}
		m_sequence.fetch_add(1, std::memory_order_release);
	}

	bool tryRead(T& value, uint32_t* sequence = nullptr) const {
		const uint32_t begin = m_sequence.load(std::memory_order_acquire);
		if (begin & 1) {
			return false;
		}
		auto* target = reinterpret_cast<uint8_t*>(&value);
		for (size_t i = 0; i < sizeof(T); i++) {
			target[i] = m_bytes[i].load(std::memory_order_relaxed);
		}
		const uint32_t end = m_sequence.load(std::memory_order_acquire);
		if (begin != end || (end & 1)) {
			return false;
		}
		if (sequence) {
			*sequence = end;
		}
		return true;
	}

private:
	std::array<std::atomic<uint8_t>, sizeof(T)> m_bytes;
	std::atomic<uint32_t> m_sequence{0};
};

/**
 * The EMP module owns the state machine and all CAN framing.  It deliberately
 * has no direct dependency on persistent configuration: the integration layer
 * calls setConfiguration() on boot and after each burn.
 */
class EmpPump final : public EngineModule, public CanListener {
public:
	EmpPump();

	void initNoConfiguration() override;
	void onConfigurationChange(engine_configuration_s const* previousConfig) override;
	void onSlowCallback() override;
	void pollTx(CanBusIndex bus);
	void onEngineStop() override;
	void onIgnitionStateChanged(bool ignitionOn) override;
	bool needsDelayedShutoff() override;

	bool acceptFrame(CanBusIndex busIndex, const CANRxFrame& frame) const override;
	void decodeFrame(CanBusIndex busIndex, const CANRxFrame& frame, efitick_t nowNt) override;
	void decodeFrame(const CANRxFrame& frame, efitick_t nowNt) override;

	void setConfiguration(const EmpPumpConfig& config);
	EmpPumpConfig getConfiguration() const;

	// These two signals are intentionally supplied by the integration layer:
	// FOME fan topology is board/configuration dependent.
	void setAirflowInputs(bool fanOn, bool airflowAtMaximumCapacity);

	bool handleServiceCommand(EmpPumpServiceCommand command, uint16_t rpm = 0, uint8_t durationSeconds = 0);
	void reportTransmitResult(bool success);
	EmpPumpStatus getStatus() const;

private:
	struct Inputs {
		int16_t coolant = 0;
		int16_t iat = 0;
		uint16_t rpm = 0;
		uint16_t map = 0;
		uint16_t vehicleSpeedKph = 0;
		uint8_t battery10 = 0;
		bool coolantValid = false;
		bool iatValid = false;
		bool vehicleSpeedValid = false;
		bool fanOn = false;
		bool airflowAtMaximumCapacity = false;
		bool engineRunning = false;
		bool engineCranking = false;
	};

	struct RxTelemetry {
		uint32_t generation = 0;
		uint16_t actualRpm = 0;
		uint8_t actualPercentRaw = 0;
		uint8_t controllerStatus = 0;
		uint8_t statusSummary = 0;
		uint8_t status1ControllerState = 0;
		uint8_t status2CommandSource = 0;
		uint8_t status2ServiceState = 0;
		uint8_t status2OperationState = 0;
		uint8_t hvilState = 0;
		uint16_t voltageRaw = 0;
		uint16_t currentRaw = 32000;
		uint16_t powerWatts = 0;
		uint16_t externalTemperatureRaw = 8736;
		uint8_t capabilities = 0;
		uint32_t lastMainStatusMs = 0;
		bool mainStatusSeen = false;
	};

	struct TxCommand {
		uint32_t id = 0;
		uint16_t rawSpeed = 0xffff;
		uint8_t control = 0;
	};

	static constexpr size_t TxQueueCapacity = 8;
	struct TxQueue {
		std::array<TxCommand, TxQueueCapacity> commands;
		std::atomic<uint32_t> write{0};
		std::atomic<uint32_t> read{0};
	};

	static constexpr uint32_t CommandBaseId = 0x18EF0000;
	static constexpr uint32_t Status1BaseId = 0x18FF0300;
	static constexpr uint32_t Status2BaseId = 0x18FF2300;
	static constexpr uint32_t Status3BaseId = 0x18FF2400;
	static constexpr uint32_t ExternalTemperatureBaseId = 0x18FF4300;
	static constexpr uint32_t AddressClaimBaseId = 0x18EEFF00;
	static constexpr uint8_t ControlOff = 0xFC;
	static constexpr uint8_t ControlOffPowerHoldOff = 0xF0;
	static constexpr uint8_t ControlForward = 0xFD;
	static constexpr uint8_t ControlForwardPowerHoldOff = 0xF1;
	static constexpr uint8_t ControlForwardPowerHoldOn = 0xF5;

	Inputs readInputs() const;
	void resetRuntime(uint32_t nowMs);
	void update(uint32_t nowMs, const Inputs& inputs);
	void updatePowerHoldRequest(const Inputs& inputs);
	void updateStatusFaults();
	void applyRamp(uint32_t nowMs);
	uint16_t closedLoopRpm(uint32_t nowMs, const Inputs& inputs);
	uint16_t curveRpm(int16_t coolant) const;
	uint16_t minimumFlowRpm(uint16_t engineRpm) const;
	void resetClosedLoopRuntime();
	void sendCommandIfDue(uint32_t nowMs);
	void transmitCommand(uint32_t nowMs);
	bool queueTx(CanBusIndex bus, const TxCommand& command);
	bool txQueueEmpty(CanBusIndex bus) const;
	void recordTxQueueOverflow();
	void setState(EmpPumpState state, uint32_t nowMs);
	void decodeStatus1(RxTelemetry& telemetry, const CANRxFrame& frame, uint32_t nowMs);
	void decodeStatus2(RxTelemetry& telemetry, const CANRxFrame& frame, uint32_t nowMs);
	void decodeStatus3(RxTelemetry& telemetry, const CANRxFrame& frame);
	bool isConfigSane() const;
	static bool isConfigSane(const EmpPumpConfig& config);
	bool isEnabled() const;
	bool releasePowerHold();
	void consumeServiceMailbox(uint32_t nowMs);
	bool consumeExternalMailboxes(uint32_t nowMs);
	bool applyConfiguration(const EmpPumpConfig& config, uint32_t sequence, uint32_t nowMs);
	void publishStatus(uint32_t nowMs);
	EmpPumpStatus readRxStatus(EmpPumpStatus status, uint32_t nowMs) const;
	static uint32_t encodeEndpoint(const EmpPumpConfig& config);
	static bool decodeEndpoint(uint32_t endpoint, CanBusIndex& bus, uint8_t& address);

	// Runtime state below belongs exclusively to onSlowCallback(). External
	// callers submit mailboxes; CAN RX only updates the atomics below.
	EmpPumpConfig m_config;
	// Request mailbox: single external configuration/Burn producer.
	EmpPumpSnapshotMailbox<EmpPumpConfig> m_configurationMailbox;
	// Active snapshot: single slow-owner producer, safe for console readers.
	EmpPumpSnapshotMailbox<EmpPumpConfig> m_activeConfigurationMailbox;
	EmpPumpSnapshotMailbox<EmpPumpStatus> m_statusMailbox;
	uint32_t m_lastConfigurationSequence = 0;
	std::atomic<bool> m_configurationLoaded{false};
	std::atomic<bool> m_activeConfigurationPublished{false};
	// The receiver endpoint is published only after old Power Hold is released.
	std::atomic<uint32_t> m_rxEndpoint{0};

	// RX telemetry has one bounded-seqlock mailbox per CAN bus. It is never
	// coupled to mutable configuration; readers select only the active epoch.
	std::atomic<uint32_t> m_activeRxGeneration{0};
	std::array<EmpPumpSnapshotMailbox<RxTelemetry>, 2> m_rxMailboxes;
	// The slow callback is the sole producer; each bus TX thread consumes its queue.
	std::array<TxQueue, 2> m_txQueues;

	EmpPumpState m_state = EmpPumpState::Disabled;
	EmpPumpState m_previousState = EmpPumpState::Disabled;
	EmpPumpThermalState m_thermalState = EmpPumpThermalState::Inactive;
	uint16_t m_targetRpm = 0;
	uint16_t m_requestedRpm = 0;
	uint16_t m_latchedFaults = 0;
	uint16_t m_diagnosticFaults = 0;
	uint16_t m_controlFaults = 0;
	uint32_t m_lastUpdateMs = 0;
	uint32_t m_lastTransmitMs = 0;
	uint32_t m_stateSinceMs = 0;
	uint32_t m_stopCandidateMs = 0;
	uint32_t m_afterRunEndMs = 0;
	uint32_t m_serviceEndMs = 0;
	uint32_t m_lastSlopeSampleMs = 0;
	uint32_t m_saturationSinceMs = 0;
	uint16_t m_serviceRpm = 0;
	uint8_t m_transmitFailureCount = 0;
	uint8_t m_lastCommandControl = ControlOff;
	uint8_t m_controlFlags = 0;
	uint8_t m_coolingDemandRaw = 0;
	bool m_transmitSeen = false;
	bool m_engineSeen = false;
	bool m_hotBootEvaluated = false;
	bool m_batteryInhibit = false;
	bool m_commandDirty = true;
	bool m_powerHoldWanted = false;
	bool m_powerHoldCommanded = false;
	bool m_releaseQueued = false;
	bool m_configurationTransitionPending = false;
	bool m_filteredIatSeen = false;
	bool m_slopeSampleSeen = false;
	bool m_engineStopNotified = false;
	bool m_controlInitialized = false;
	// Console/bench callers write a single atomic mailbox.  State-machine
	// ownership remains with onSlowCallback(), avoiding concurrent mutation.
	std::atomic<uint32_t> m_serviceMailbox{0};
	std::atomic<bool> m_engineStopMailbox{false};
	// 0 = no event, 1 = key off, 2 = key on.
	std::atomic<uint8_t> m_ignitionMailbox{0};
	std::atomic<int8_t> m_transmitResultMailbox{0};
	std::atomic<bool> m_delayedShutoffRequested{false};
	std::atomic<uint32_t> m_statusStateSinceMs{0};
	std::atomic<uint32_t> m_statusTimeoutMs{0};
	std::atomic<bool> m_fanOn{false};
	std::atomic<bool> m_airflowAtMaximumCapacity{false};
	int32_t m_filteredIatX8 = 0;
	int32_t m_integralMilliRpm = 0;
	int16_t m_coolantSlopePerMinute = 0;
	int16_t m_lastSlopeCoolant = 0;
	int16_t m_diagnosticFeedForwardRpm = 0;
	int16_t m_diagnosticPiCorrectionRpm = 0;
	int16_t m_diagnosticFilteredIat = 0;
	int8_t m_diagnosticTemperatureError = 0;
	uint16_t m_diagnosticMinimumFlowRpm = 0;
};
