/**
 * @file emp_pump.cpp
 * @see emp_pump.h for Speeduino provenance and FOME adaptation notes.
 */

#include "pch.h"

#include "emp_pump.h"

#include "can_msg_tx.h"
#include "fan_control.h"

#include <algorithm>
#include <climits>

namespace {
constexpr uint32_t HeartbeatMs = 500;
constexpr uint8_t ControlFlagClosedLoop = 1 << 0;
constexpr uint8_t ControlFlagAtMinimum = 1 << 1;
constexpr uint8_t ControlFlagAtMaximum = 1 << 2;
constexpr uint8_t ControlFlagIatValid = 1 << 3;
constexpr uint8_t ControlFlagVssValid = 1 << 4;
constexpr uint8_t ControlFlagFanOn = 1 << 5;
constexpr uint8_t ControlFlagPositiveSlope = 1 << 6;
constexpr uint8_t ControlFlagAirflowAtCapacity = 1 << 7;

uint32_t nowMs() {
	return static_cast<uint32_t>(getTimeNowUs() / 1000);
}

bool elapsed(uint32_t now, uint32_t then, uint32_t period) {
	return static_cast<uint32_t>(now - then) >= period;
}

bool reached(uint32_t now, uint32_t deadline) {
	return static_cast<int32_t>(now - deadline) >= 0;
}

uint16_t saturatedU16(uint32_t value) {
	return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

int16_t saturatedI16(int32_t value) {
	return std::clamp(value, static_cast<int32_t>(INT16_MIN), static_cast<int32_t>(INT16_MAX));
}

int8_t saturatedI8(int16_t value) {
	return static_cast<int8_t>(std::clamp<int16_t>(value, INT8_MIN, INT8_MAX));
}

uint16_t clampPositive(float value) {
	if (value <= 0) {
		return 0;
	}

	return value >= UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

bool isExtendedFrame(const CANRxFrame& frame) {
#ifdef STM32H7XX
	return frame.common.XTD;
#else
	return frame.IDE;
#endif
}

EmpPumpConfig mapFomeConfig(const engine_configuration_s& configuration) {
	const auto& source = configuration.empPump;
	EmpPumpConfig result;
	result.enabled = configuration.enableEmpPump;
	result.afterRunEnabled = source.afterRunEnabled;
	result.powerHoldEnabled = source.powerHoldEnabled;
	result.runDuringCranking = source.runDuringCranking;
	result.hotBootRecovery = source.hotBootRecovery;
	result.closedLoopEnabled = source.closedLoopEnabled;
	result.serviceAllowedWhileRunning = source.serviceAllowedWhileRunning;
	result.canBus = source.canBus;
	result.controllerAddress = source.controllerAddress;
	result.sourceAddress = source.sourceAddress;
	result.stopDebounce100ms = source.stopDebounce100ms;
	result.statusTimeoutSeconds = source.statusTimeoutSeconds;
	result.batteryCutoff10 = source.batteryCutoff10;
	result.batteryResume10 = source.batteryResume10;
	result.manualTestSeconds = source.manualTestSeconds;
	result.minRpm = source.minimumRunRpm;
	result.maxRpm = source.maximumRpm;
	result.afterRunMinRpm = source.afterRunMinimumRpm;
	result.failsafeRpm = source.failsafeRpm;
	result.rampRpmPerSecond = source.rampRpmPerSecond;
	result.afterRunMaxSeconds = source.afterRunMaximumSeconds;
	result.manualTestRpm = source.manualTestRpm;
	result.afterRunStartTemp = source.afterRunStartTemperature;
	result.afterRunStopTemp = source.afterRunStopTemperature;
	result.targetTemp = source.targetTemperature;
	result.iatReference = source.iatReferenceTemperature;
	result.deadband = source.temperatureDeadband;
	result.kp = source.proportionalGain;
	result.ki = source.integralGain;
	result.integralLimit = source.integralLimitRpm;
	result.derivative = source.derivativeGain;
	result.loadFF = source.loadFeedForwardGain;
	result.iatGain = source.iatCompensationGain;
	result.airflowFull = source.airflowFullSpeedKph;
	result.airflowRelief = source.airflowReliefRpm;
	result.fanEq = source.fanEquivalentSpeedKph;
	result.coolingLimitedDelta = source.coolingLimitedDelta;
	result.overloadDelta = source.overloadDelta;
	result.overloadDelay = source.overloadDelaySeconds;
	for (size_t i = 0; i < efi::size(result.temperatureBins); i++) {
		result.temperatureBins[i] = source.temperatureBins[i];
		result.rpmBins[i] = source.rpmBins[i];
	}
	for (size_t i = 0; i < efi::size(result.engineRpmBins); i++) {
		result.engineRpmBins[i] = source.engineRpmBins[i];
		result.minimumFlowRpmBins[i] = source.minimumFlowRpmBins[i];
	}
	return result;
}
} // namespace

EmpPump::EmpPump()
	: CanListener(0) {}

uint32_t EmpPump::encodeEndpoint(const EmpPumpConfig& configuration) {
	if (!configuration.enabled || !isConfigSane(configuration)) {
		return 0;
	}
	return 1 | (static_cast<uint32_t>(configuration.canBus) << 1) |
		   (static_cast<uint32_t>(configuration.controllerAddress) << 8);
}

bool EmpPump::decodeEndpoint(uint32_t endpoint, CanBusIndex& bus, uint8_t& address) {
	if ((endpoint & 1) == 0) {
		return false;
	}
	bus = static_cast<CanBusIndex>((endpoint >> 1) & 1);
	address = static_cast<uint8_t>(endpoint >> 8);
	return true;
}

void EmpPump::initNoConfiguration() {
#if EFI_CAN_SUPPORT
	registerCanListener(*this);
#endif

	resetRuntime(nowMs());
}

void EmpPump::onConfigurationChange(engine_configuration_s const*) {
	setConfiguration(mapFomeConfig(*engineConfiguration));
}

void EmpPump::setConfiguration(const EmpPumpConfig& newConfig) {
	// The Burn/configuration thread never touches the slow owner's m_config.
	// The slow callback releases F5 on the old endpoint before applying this
	// snapshot and publishing the new RX endpoint.
	m_configurationMailbox.publish(newConfig);
	if (!m_configurationLoaded.load(std::memory_order_acquire)) {
		// Initial boot has no old endpoint or Power Hold to release. Publishing
		// this lets CAN RX discover a controller before the first slow callback.
		m_rxEndpoint.store(encodeEndpoint(newConfig), std::memory_order_release);
	}
	m_serviceMailbox.store(0, std::memory_order_release);
}

EmpPumpConfig EmpPump::getConfiguration() const {
	EmpPumpConfig result;
	if (m_activeConfigurationPublished.load(std::memory_order_acquire)) {
		for (int attempt = 0; attempt < 3; attempt++) {
			if (m_activeConfigurationMailbox.tryRead(result)) {
				return result;
			}
		}
	}
	if (!m_activeConfigurationPublished.load(std::memory_order_acquire)) {
		for (int attempt = 0; attempt < 3; attempt++) {
			if (m_configurationMailbox.tryRead(result)) {
				return result;
			}
		}
	}
	return result;
}

void EmpPump::setAirflowInputs(bool fanOn, bool airflowAtMaximumCapacity) {
	m_fanOn.store(fanOn, std::memory_order_relaxed);
	m_airflowAtMaximumCapacity.store(airflowAtMaximumCapacity, std::memory_order_relaxed);
}

bool EmpPump::isEnabled() const {
	return m_config.enabled && isConfigSane();
}

bool EmpPump::isConfigSane() const {
	return isConfigSane(m_config);
}

bool EmpPump::isConfigSane(const EmpPumpConfig& configuration) {
	if (configuration.canBus > static_cast<uint8_t>(CanBusIndex::Bus1) || configuration.controllerAddress > 0xf0 ||
		configuration.sourceAddress > 0xf0 || configuration.controllerAddress == configuration.sourceAddress ||
		configuration.minRpm == 0 || configuration.minRpm > configuration.afterRunMinRpm ||
		configuration.afterRunMinRpm > configuration.maxRpm || configuration.failsafeRpm < configuration.minRpm ||
		configuration.failsafeRpm > configuration.maxRpm || configuration.maxRpm > 32767 ||
		configuration.afterRunStopTemp >= configuration.afterRunStartTemp ||
		configuration.batteryResume10 < configuration.batteryCutoff10 || configuration.statusTimeoutSeconds == 0 ||
		configuration.manualTestSeconds == 0 || configuration.manualTestRpm < configuration.minRpm ||
		configuration.manualTestRpm > configuration.maxRpm || configuration.integralLimit > 12000 ||
		configuration.kp > 2000 || configuration.ki > 1000 || configuration.derivative > 1000 ||
		configuration.loadFF > 2000 || configuration.iatGain > 500 || configuration.airflowFull == 0 ||
		configuration.airflowRelief > 12000 || configuration.coolingLimitedDelta > configuration.overloadDelta) {
		return false;
	}

	for (size_t i = 1; i < efi::size(configuration.temperatureBins); i++) {
		if (configuration.temperatureBins[i] <= configuration.temperatureBins[i - 1]) {
			return false;
		}
	}

	for (size_t i = 1; i < efi::size(configuration.engineRpmBins); i++) {
		if (configuration.engineRpmBins[i] <= configuration.engineRpmBins[i - 1]) {
			return false;
		}
	}

	return true;
}

bool EmpPump::acceptFrame(CanBusIndex busIndex, const CANRxFrame& frame) const {
	CanBusIndex expectedBus;
	uint8_t address;
	if (!decodeEndpoint(m_rxEndpoint.load(std::memory_order_acquire), expectedBus, address) ||
		busIndex != expectedBus || !isExtendedFrame(frame)) {
		return false;
	}

	const uint32_t id = CAN_ID(frame);
	if (id == (Status1BaseId | address) || id == (Status2BaseId | address)) {
		return frame.DLC >= 8;
	}
	if (id == (Status3BaseId | address)) {
		return frame.DLC >= 5;
	}
	if (id == (ExternalTemperatureBaseId | address)) {
		return frame.DLC >= 3;
	}
	return id == (AddressClaimBaseId | address);
}

void EmpPump::decodeFrame(const CANRxFrame& frame, efitick_t nowNt) {
	decodeFrame(CanBusIndex::Bus0, frame, nowNt);
}

void EmpPump::decodeFrame(CanBusIndex busIndex, const CANRxFrame& frame, efitick_t nowNt) {
	// Each CAN bus has its own single-writer mailbox. A frame accepted just
	// before a bus switch is discarded by the endpoint/generation recheck.
	const uint32_t endpoint = m_rxEndpoint.load(std::memory_order_acquire);
	const uint32_t rxGeneration = m_activeRxGeneration.load(std::memory_order_acquire);
	const uint32_t id = CAN_ID(frame);
	const uint32_t now = static_cast<uint32_t>(NT2US(nowNt.count) / 1000);
	CanBusIndex expectedBus;
	uint8_t address;
	if (!decodeEndpoint(endpoint, expectedBus, address) || busIndex != expectedBus) {
		return;
	}
	RxTelemetry telemetry;
	m_rxMailboxes[static_cast<size_t>(busIndex)].tryRead(telemetry);
	if (telemetry.generation != rxGeneration) {
		telemetry = {};
	}

	if (id == (Status1BaseId | address) && frame.DLC >= 8) {
		decodeStatus1(telemetry, frame, now);
	} else if (id == (Status2BaseId | address) && frame.DLC >= 8) {
		decodeStatus2(telemetry, frame, now);
	} else if (id == (Status3BaseId | address) && frame.DLC >= 5) {
		decodeStatus3(telemetry, frame);
	} else if (id == (ExternalTemperatureBaseId | address) && frame.DLC >= 3) {
		const uint16_t raw = static_cast<uint16_t>(frame.data8[1]) | (static_cast<uint16_t>(frame.data8[2]) << 8);
		telemetry.externalTemperatureRaw = (raw == 0xffff || raw == 0xfeff || raw == 0xfffe) ? 8736 : raw;
		telemetry.capabilities |= EmpPumpCapExternalTemperature;
	} else if (id == (AddressClaimBaseId | address)) {
		telemetry.capabilities |= EmpPumpCapAddressClaim;
	}
	// Do not let a pre-switch frame commit into the new generation.
	if (m_rxEndpoint.load(std::memory_order_acquire) == endpoint &&
		m_activeRxGeneration.load(std::memory_order_acquire) == rxGeneration) {
		telemetry.generation = rxGeneration;
		m_rxMailboxes[static_cast<size_t>(busIndex)].publish(telemetry);
	}
}

void EmpPump::decodeStatus1(RxTelemetry& telemetry, const CANRxFrame& frame, uint32_t now) {
	const uint8_t summary = frame.data8[0] & 0x03;
	const uint8_t state = (frame.data8[0] >> 2) & 0x1f;
	const uint16_t rawSpeed = static_cast<uint16_t>(frame.data8[1]) | (static_cast<uint16_t>(frame.data8[2]) << 8);
	const uint16_t rawPower = static_cast<uint16_t>(frame.data8[5]) | (static_cast<uint16_t>(frame.data8[6]) << 8);

	telemetry.statusSummary = (telemetry.statusSummary & 0xfc) | summary;
	telemetry.controllerStatus = state;
	telemetry.status1ControllerState = state;
	telemetry.actualRpm = rawSpeed == 0xffff ? 0 : rawSpeed / 2;
	telemetry.powerWatts = rawPower == 0xffff ? 0 : rawPower;
	telemetry.actualPercentRaw = frame.data8[7] == 0xff ? 0 : frame.data8[7];
	telemetry.lastMainStatusMs = now;
	telemetry.mainStatusSeen = true;
	telemetry.capabilities |= EmpPumpCapStatus1;
}

void EmpPump::decodeStatus2(RxTelemetry& telemetry, const CANRxFrame& frame, uint32_t now) {
	const uint8_t direction = frame.data8[0] & 0x03;
	const uint8_t controller = (frame.data8[0] >> 2) & 0x0f;
	const uint8_t commandSource = (frame.data8[0] >> 6) & 0x03;
	const uint8_t service = frame.data8[7] & 0x03;
	const uint8_t operation = (frame.data8[7] >> 2) & 0x03;
	const uint16_t rawSpeed = static_cast<uint16_t>(frame.data8[1]) | (static_cast<uint16_t>(frame.data8[2]) << 8);
	const uint16_t rawTemperature =
			static_cast<uint16_t>(frame.data8[3]) | (static_cast<uint16_t>(frame.data8[4]) << 8);
	const uint16_t rawPower = static_cast<uint16_t>(frame.data8[5]) | (static_cast<uint16_t>(frame.data8[6]) << 8);

	telemetry.statusSummary = direction | (commandSource << 2) | (service << 4) | (operation << 6);
	telemetry.controllerStatus = controller;
	telemetry.status2CommandSource = commandSource;
	telemetry.status2ServiceState = service;
	telemetry.status2OperationState = operation;
	telemetry.actualRpm = rawSpeed == 0xffff ? 0 : rawSpeed / 2;
	telemetry.externalTemperatureRaw =
			(rawTemperature == 0xffff || rawTemperature == 0xfeff || rawTemperature == 0xfffe) ? 8736 : rawTemperature;
	telemetry.powerWatts = rawPower == 0xffff ? 0 : rawPower / 2;
	telemetry.lastMainStatusMs = now;
	telemetry.mainStatusSeen = true;
	telemetry.capabilities |= EmpPumpCapStatus2;
}

void EmpPump::decodeStatus3(RxTelemetry& telemetry, const CANRxFrame& frame) {
	const uint16_t rawVoltage = static_cast<uint16_t>(frame.data8[0]) | (static_cast<uint16_t>(frame.data8[1]) << 8);
	const uint16_t rawCurrent = static_cast<uint16_t>(frame.data8[2]) | (static_cast<uint16_t>(frame.data8[3]) << 8);
	telemetry.voltageRaw = rawVoltage == 0xffff ? 0 : rawVoltage;
	telemetry.currentRaw = rawCurrent == 0xffff ? 32000 : rawCurrent;
	// Status2 owns statusSummary bits 6-7 (operation state).  Keep HVIL in a
	// dedicated telemetry field so Status2 cannot manufacture HVIL faults.
	telemetry.hvilState = frame.data8[4] & 0x03;
	telemetry.capabilities |= EmpPumpCapStatus3;
}

EmpPump::Inputs EmpPump::readInputs() const {
	Inputs result;
	auto clt = Sensor::get(SensorType::Clt);
	if (clt) {
		result.coolant = static_cast<int16_t>(clt.Value);
		result.coolantValid = true;
	}
	auto iat = Sensor::get(SensorType::Iat);
	if (iat) {
		result.iat = static_cast<int16_t>(iat.Value);
		result.iatValid = true;
	}
	auto vss = Sensor::get(SensorType::VehicleSpeed);
	if (vss) {
		result.vehicleSpeedKph = clampPositive(vss.Value);
		result.vehicleSpeedValid = true;
	}

	result.rpm = clampPositive(Sensor::getOrZero(SensorType::Rpm));
	result.map = clampPositive(Sensor::getOrZero(SensorType::Map));
	result.battery10 =
			static_cast<uint8_t>(std::clamp(Sensor::getOrZero(SensorType::BatteryVoltage) * 10.0f, 0.0f, 255.0f));
	result.fanOn = m_fanOn.load(std::memory_order_relaxed);
	result.airflowAtMaximumCapacity = m_airflowAtMaximumCapacity.load(std::memory_order_relaxed);
#if EFI_SHAFT_POSITION_INPUT
	result.engineRunning = engine->rpmCalculator.isRunning();
	result.engineCranking = engine->rpmCalculator.isCranking();
#endif
	return result;
}

void EmpPump::onSlowCallback() {
	const uint32_t now = nowMs();
	// Engine modules are registered before loadConfiguration().  Configuration
	// change callbacks intentionally are not emitted during boot, so take the
	// final flash/default snapshot on the first running callback.
	if (!m_configurationLoaded.load(std::memory_order_acquire)) {
		EmpPumpConfig initial;
		if (!m_configurationMailbox.tryRead(initial, &m_lastConfigurationSequence) ||
			m_lastConfigurationSequence == 0) {
			initial = mapFomeConfig(*engineConfiguration);
			// The configuration mailbox has one external producer. Bootstrap is
			// local to the slow owner, so it must not publish into that mailbox.
			m_lastConfigurationSequence = 1;
		}
		m_config = initial;
		m_activeConfigurationMailbox.publish(m_config);
		m_activeConfigurationPublished.store(true, std::memory_order_release);
		m_configurationLoaded.store(true, std::memory_order_release);
		m_activeRxGeneration.store(m_lastConfigurationSequence, std::memory_order_release);
		m_rxEndpoint.store(encodeEndpoint(m_config), std::memory_order_release);
		resetRuntime(now);
	}
	if (!consumeExternalMailboxes(now)) {
		publishStatus(now);
		return;
	}
	consumeServiceMailbox(now);

	// EngineModule slow callbacks run at approximately 20Hz: run the regulator
	// at exactly 10Hz, while still emitting a due CAN heartbeat every callback.
	if (!m_controlInitialized || elapsed(now, m_lastUpdateMs, 100)) {
		// A PWM fan reports m_state only when its resulting duty is non-zero.
		// Check both fields so configured PWM and conventional relay fans provide
		// the same ram-air relief input.
		auto& fan1 = engine->module<FanControl1>();
		auto& fan2 = engine->module<FanControl2>();
		const float fan1Duty = static_cast<float>(fan1->fanDuty);
		const float fan2Duty = static_cast<float>(fan2->fanDuty);
		const bool fanOn = fan1->m_state || fan2->m_state || fan1Duty > 0 || fan2Duty > 0;
		// A relay fan is binary and therefore at full cooling when on.  For PWM
		// fans only a commanded 100% duty asserts the capacity limit.
		const bool airflowAtMaximumCapacity = fan1->m_state || fan2->m_state || fan1Duty >= 100 || fan2Duty >= 100;
		setAirflowInputs(fanOn, airflowAtMaximumCapacity);
		update(now, readInputs());
		m_controlInitialized = true;
	}
	sendCommandIfDue(now);
	publishStatus(now);
}

bool EmpPump::consumeExternalMailboxes(uint32_t now) {
	EmpPumpConfig nextConfig;
	uint32_t sequence = 0;
	bool configurationReady = !m_configurationTransitionPending;
	if (m_configurationMailbox.tryRead(nextConfig, &sequence) && sequence != 0 &&
		sequence != m_lastConfigurationSequence) {
		m_configurationTransitionPending = true;
		configurationReady = applyConfiguration(nextConfig, sequence, now);
		if (configurationReady) {
			m_configurationTransitionPending = false;
		}
	}

	if (m_engineStopMailbox.exchange(false, std::memory_order_acq_rel)) {
		m_engineStopNotified = true;
	}

	switch (m_ignitionMailbox.exchange(0, std::memory_order_acq_rel)) {
		case 2: // key on
			m_serviceEndMs = 0;
			m_afterRunEndMs = 0;
			m_powerHoldWanted = false;
			m_engineStopNotified = false;
			m_commandDirty = true;
			break;
		case 1: // key off: service must not survive, normal after-run remains eligible
			m_serviceEndMs = 0;
			m_commandDirty = true;
			break;
		default:
			break;
	}

	const int8_t txResult = m_transmitResultMailbox.exchange(0, std::memory_order_acq_rel);
	if (txResult > 0) {
		m_latchedFaults &= static_cast<uint16_t>(~EmpPumpFaultTx);
	} else if (txResult < 0) {
		m_latchedFaults |= EmpPumpFaultTx;
		if (m_transmitFailureCount < UINT8_MAX) {
			m_transmitFailureCount++;
		}
		m_commandDirty = true;
	}
	return configurationReady;
}

bool EmpPump::applyConfiguration(const EmpPumpConfig& configuration, uint32_t sequence, uint32_t now) {
	// Drain commands on the old endpoint, including F0 when Power Hold was
	// requested, before accepting traffic or commands on the new endpoint.
	// The slow callback skips control while this nonblocking transition waits.
	if (!releasePowerHold() || !txQueueEmpty(static_cast<CanBusIndex>(m_config.canBus))) {
		return false;
	}
	m_config = configuration;
	m_activeConfigurationMailbox.publish(m_config);
	m_activeConfigurationPublished.store(true, std::memory_order_release);
	m_lastConfigurationSequence = sequence;
	resetRuntime(now);
	m_activeRxGeneration.store(sequence, std::memory_order_release);
	m_rxEndpoint.store(encodeEndpoint(m_config), std::memory_order_release);
	return true;
}

void EmpPump::onEngineStop() {
	// This callback may run outside the slow owner. Defer the transition.
	m_engineStopMailbox.store(true, std::memory_order_release);
}

void EmpPump::onIgnitionStateChanged(bool ignitionOn) {
	m_ignitionMailbox.store(ignitionOn ? 2 : 1, std::memory_order_release);
}

bool EmpPump::needsDelayedShutoff() {
	return m_delayedShutoffRequested.load(std::memory_order_acquire);
}

void EmpPump::resetRuntime(uint32_t now) {
	m_previousState = m_state;
	m_state = EmpPumpState::Disabled;
	m_thermalState = EmpPumpThermalState::Inactive;
	m_targetRpm = 0;
	m_requestedRpm = 0;
	m_latchedFaults = 0;
	m_diagnosticFaults = 0;
	m_lastUpdateMs = now;
	m_lastTransmitMs = 0;
	m_stateSinceMs = now;
	m_stopCandidateMs = 0;
	m_afterRunEndMs = 0;
	m_serviceEndMs = 0;
	m_serviceRpm = 0;
	m_transmitFailureCount = 0;
	m_lastCommandControl = ControlOff;
	m_transmitSeen = false;
	m_engineSeen = false;
	m_hotBootEvaluated = false;
	m_batteryInhibit = false;
	m_commandDirty = true;
	m_powerHoldWanted = false;
	m_powerHoldCommanded = false;
	m_releaseQueued = false;
	m_filteredIatSeen = false;
	m_engineStopNotified = false;
	m_controlInitialized = false;
	m_delayedShutoffRequested.store(false, std::memory_order_release);
	resetClosedLoopRuntime();
}

void EmpPump::setState(EmpPumpState state, uint32_t now) {
	if (m_state != state) {
		m_previousState = m_state;
		m_state = state;
		m_stateSinceMs = now;
		m_commandDirty = true;
	}
}

void EmpPump::update(uint32_t now, const Inputs& inputs) {
	updateStatusFaults();
	const bool valid = isConfigSane();
	const bool enabled = m_config.enabled && valid;
	if (m_engineStopNotified && inputs.engineRunning) {
		// A restart can occur before the following slow callback.
		m_engineStopNotified = false;
	}
	const bool engineActive =
			!m_engineStopNotified && (inputs.engineRunning || (inputs.engineCranking && m_config.runDuringCranking));

	if (!valid) {
		m_latchedFaults |= EmpPumpFaultConfig;
	}
	if (enabled && !inputs.coolantValid &&
		(engineActive || m_state == EmpPumpState::AfterRun || m_state == EmpPumpState::ServiceTest)) {
		m_latchedFaults |= EmpPumpFaultCltInvalid;
	}

	if (!enabled) {
		m_requestedRpm = 0;
		m_serviceEndMs = 0;
		m_afterRunEndMs = 0;
		setState(EmpPumpState::Disabled, now);
		m_thermalState = EmpPumpThermalState::Inactive;
		resetClosedLoopRuntime();
		updatePowerHoldRequest(inputs);
		applyRamp(now);
		m_lastUpdateMs = now;
		return;
	}

	if (m_serviceEndMs != 0 && !reached(now, m_serviceEndMs)) {
		setState(EmpPumpState::ServiceTest, now);
		m_requestedRpm = m_serviceRpm;
		m_thermalState = EmpPumpThermalState::Service;
		resetClosedLoopRuntime();
	} else if (engineActive) {
		m_engineStopNotified = false;
		m_engineSeen = true;
		m_stopCandidateMs = 0;
		m_afterRunEndMs = 0;
		m_batteryInhibit = false;
		setState(EmpPumpState::EngineActive, now);
		if (!inputs.coolantValid) {
			m_requestedRpm = m_config.failsafeRpm;
			m_thermalState = EmpPumpThermalState::Failsafe;
			resetClosedLoopRuntime();
		} else if (m_config.closedLoopEnabled) {
			m_requestedRpm = closedLoopRpm(now, inputs);
		} else {
			m_requestedRpm = curveRpm(inputs.coolant);
			m_thermalState = EmpPumpThermalState::ClosedLoop;
			resetClosedLoopRuntime();
		}
	} else {
		m_serviceEndMs = 0;
		if (!m_hotBootEvaluated) {
			m_hotBootEvaluated = true;
			if (m_config.hotBootRecovery && inputs.coolantValid && inputs.coolant >= m_config.afterRunStartTemp) {
				m_engineSeen = true;
				m_afterRunEndMs = now + static_cast<uint32_t>(m_config.afterRunMaxSeconds) * 1000;
			}
		}

		if (m_state == EmpPumpState::EngineActive) {
			if (m_stopCandidateMs == 0) {
				m_stopCandidateMs = now;
			}
			const uint32_t debounce =
					m_engineStopNotified ? 0 : static_cast<uint32_t>(m_config.stopDebounce100ms) * 100;
			if (!elapsed(now, m_stopCandidateMs, debounce)) {
				m_requestedRpm = inputs.coolantValid ? (m_config.closedLoopEnabled ? closedLoopRpm(now, inputs)
																				   : curveRpm(inputs.coolant))
													 : m_config.failsafeRpm;
				m_thermalState = inputs.coolantValid ? EmpPumpThermalState::ClosedLoop : EmpPumpThermalState::Failsafe;
				updatePowerHoldRequest(inputs);
				applyRamp(now);
				m_lastUpdateMs = now;
				return;
			}
			if (m_config.afterRunEnabled) {
				m_afterRunEndMs = now + static_cast<uint32_t>(m_config.afterRunMaxSeconds) * 1000;
			}
		}

		const bool mayAfterRun = m_engineSeen && m_config.afterRunEnabled && m_afterRunEndMs != 0 &&
								 !reached(now, m_afterRunEndMs) && inputs.coolantValid &&
								 inputs.coolant >= m_config.afterRunStartTemp;
		if (m_state != EmpPumpState::AfterRun && mayAfterRun) {
			setState(EmpPumpState::AfterRun, now);
			m_batteryInhibit = inputs.battery10 < m_config.batteryCutoff10;
		}

		if (m_state == EmpPumpState::AfterRun) {
			m_thermalState = EmpPumpThermalState::AfterRun;
			resetClosedLoopRuntime();
			if (!inputs.coolantValid || inputs.coolant <= m_config.afterRunStopTemp || reached(now, m_afterRunEndMs)) {
				m_afterRunEndMs = 0;
				m_engineSeen = false;
				setState(EmpPumpState::Stopped, now);
				m_requestedRpm = 0;
				m_thermalState = EmpPumpThermalState::Inactive;
			} else {
				if (inputs.battery10 < m_config.batteryCutoff10) {
					m_batteryInhibit = true;
					m_latchedFaults |= EmpPumpFaultBatteryLow;
				} else if (m_batteryInhibit && inputs.battery10 >= m_config.batteryResume10) {
					m_batteryInhibit = false;
				}
				m_requestedRpm = m_batteryInhibit ? 0 : std::max(curveRpm(inputs.coolant), m_config.afterRunMinRpm);
			}
		} else {
			if (m_afterRunEndMs != 0 && reached(now, m_afterRunEndMs)) {
				m_afterRunEndMs = 0;
				m_engineSeen = false;
			}
			setState(EmpPumpState::Stopped, now);
			m_requestedRpm = 0;
			m_thermalState = EmpPumpThermalState::Inactive;
			resetClosedLoopRuntime();
		}
	}

	updatePowerHoldRequest(inputs);
	applyRamp(now);
	m_lastUpdateMs = now;
}

void EmpPump::resetClosedLoopRuntime() {
	m_integralMilliRpm = 0;
	m_coolantSlopePerMinute = 0;
	m_diagnosticFeedForwardRpm = 0;
	m_diagnosticPiCorrectionRpm = 0;
	m_diagnosticTemperatureError = 0;
	m_diagnosticMinimumFlowRpm = 0;
	m_coolingDemandRaw = 0;
	m_slopeSampleSeen = false;
	m_saturationSinceMs = 0;
	m_controlFlags = 0;
	m_controlFaults = 0;
}

uint16_t EmpPump::curveRpm(int16_t coolant) const {
	if (coolant <= m_config.temperatureBins[0]) {
		return m_config.rpmBins[0];
	}
	for (size_t i = 1; i < efi::size(m_config.temperatureBins); i++) {
		if (coolant <= m_config.temperatureBins[i]) {
			const int16_t x0 = m_config.temperatureBins[i - 1];
			const int16_t x1 = m_config.temperatureBins[i];
			const int32_t y0 = m_config.rpmBins[i - 1];
			const int32_t y1 = m_config.rpmBins[i];
			return static_cast<uint16_t>(
					std::max<int32_t>(0, y0 + (static_cast<int32_t>(coolant - x0) * (y1 - y0)) / (x1 - x0)));
		}
	}
	return m_config.rpmBins[efi::size(m_config.rpmBins) - 1];
}

uint16_t EmpPump::minimumFlowRpm(uint16_t rpm) const {
	uint16_t result = m_config.minimumFlowRpmBins[efi::size(m_config.minimumFlowRpmBins) - 1];
	if (rpm <= m_config.engineRpmBins[0]) {
		result = m_config.minimumFlowRpmBins[0];
	} else {
		for (size_t i = 1; i < efi::size(m_config.engineRpmBins); i++) {
			if (rpm <= m_config.engineRpmBins[i]) {
				const uint16_t x0 = m_config.engineRpmBins[i - 1];
				const uint16_t x1 = m_config.engineRpmBins[i];
				const uint16_t y0 = m_config.minimumFlowRpmBins[i - 1];
				const uint16_t y1 = m_config.minimumFlowRpmBins[i];
				result = static_cast<uint16_t>(y0 + (static_cast<uint32_t>(rpm - x0) * (y1 - y0)) / (x1 - x0));
				break;
			}
		}
	}
	return std::clamp(result, m_config.minRpm, m_config.maxRpm);
}

uint16_t EmpPump::closedLoopRpm(uint32_t now, const Inputs& inputs) {
	if (inputs.iatValid) {
		const int32_t sample = static_cast<int32_t>(inputs.iat) * 8;
		if (!m_filteredIatSeen) {
			m_filteredIatX8 = sample;
			m_filteredIatSeen = true;
		} else {
			m_filteredIatX8 += (sample - m_filteredIatX8) / 8;
		}
		m_diagnosticFilteredIat = static_cast<int16_t>(m_filteredIatX8 / 8);
	}
	if (!m_slopeSampleSeen) {
		m_slopeSampleSeen = true;
		m_lastSlopeSampleMs = now;
		m_lastSlopeCoolant = inputs.coolant;
	} else if (elapsed(now, m_lastSlopeSampleMs, 1000)) {
		const uint32_t dt = now - m_lastSlopeSampleMs;
		const int32_t slopePerMinute = static_cast<int32_t>(
				(static_cast<int32_t>(inputs.coolant - m_lastSlopeCoolant) * 60000) / static_cast<int32_t>(dt));
		const int32_t rawSlope = std::clamp<int32_t>(slopePerMinute, -600, 600);
		m_coolantSlopePerMinute = static_cast<int16_t>((m_coolantSlopePerMinute * 3 + rawSlope) / 4);
		m_lastSlopeSampleMs = now;
		m_lastSlopeCoolant = inputs.coolant;
	}

	m_controlFaults &=
			static_cast<uint16_t>(~(EmpPumpFaultIatInvalid | EmpPumpFaultCoolingLimited | EmpPumpFaultThermalOverload));
	m_controlFlags = ControlFlagClosedLoop;
	if (inputs.iatValid) {
		m_controlFlags |= ControlFlagIatValid;
	} else {
		m_controlFaults |= EmpPumpFaultIatInvalid;
	}
	if (inputs.vehicleSpeedValid) {
		m_controlFlags |= ControlFlagVssValid;
	}
	if (inputs.fanOn) {
		m_controlFlags |= ControlFlagFanOn;
	}
	if (m_coolantSlopePerMinute > 0) {
		m_controlFlags |= ControlFlagPositiveSlope;
	}
	if (inputs.airflowAtMaximumCapacity) {
		m_controlFlags |= ControlFlagAirflowAtCapacity;
	}

	const uint16_t minimum = minimumFlowRpm(inputs.rpm);
	m_diagnosticMinimumFlowRpm = minimum;
	m_diagnosticTemperatureError = saturatedI8(inputs.coolant - m_config.targetTemp);
	const uint16_t boundedMap = std::min<uint16_t>(inputs.map, 250);
	const uint16_t boundedRpm = std::min<uint16_t>(inputs.rpm, 20000);
	int32_t ff = (static_cast<uint32_t>(boundedRpm) * boundedMap / 100) * m_config.loadFF / 1000;
	if (inputs.iatValid) {
		ff += static_cast<int32_t>(m_diagnosticFilteredIat - m_config.iatReference) * m_config.iatGain;
	}
	uint16_t airflow = inputs.vehicleSpeedValid ? inputs.vehicleSpeedKph : 0;
	if (inputs.fanOn) {
		airflow = saturatedU16(static_cast<uint32_t>(airflow) + m_config.fanEq);
	}
	if (m_config.airflowFull > 0 && airflow > 0) {
		airflow = std::min<uint16_t>(airflow, m_config.airflowFull);
		ff -= static_cast<uint32_t>(airflow) * m_config.airflowRelief / m_config.airflowFull;
	}
	if (m_coolantSlopePerMinute > 0) {
		ff += static_cast<int32_t>(m_coolantSlopePerMinute) * m_config.derivative;
	}
	m_diagnosticFeedForwardRpm = saturatedI16(ff);

	int16_t error = inputs.coolant - m_config.targetTemp;
	if (error > m_config.deadband) {
		error -= m_config.deadband;
	} else if (error < -static_cast<int16_t>(m_config.deadband)) {
		error += m_config.deadband;
	} else {
		error = 0;
	}
	const uint32_t dt = std::min<uint32_t>(now - m_lastUpdateMs, 1000);
	const int32_t limit = static_cast<int32_t>(m_config.integralLimit) * 1000;
	const int32_t candidate = std::clamp(
			m_integralMilliRpm + static_cast<int32_t>(error) * m_config.ki * static_cast<int32_t>(dt), -limit, limit);
	const int32_t proportional = static_cast<int32_t>(error) * m_config.kp;
	int32_t pi = proportional + candidate / 1000;
	int32_t raw = static_cast<int32_t>(minimum) + ff + pi;
	if (!((raw > m_config.maxRpm && error > 0) || (raw < minimum && error < 0))) {
		m_integralMilliRpm = candidate;
	} else {
		pi = proportional + m_integralMilliRpm / 1000;
		raw = static_cast<int32_t>(minimum) + ff + pi;
	}
	m_diagnosticPiCorrectionRpm = saturatedI16(pi);
	uint16_t result = minimum;
	if (raw <= minimum) {
		m_controlFlags |= ControlFlagAtMinimum;
	} else if (raw >= m_config.maxRpm) {
		result = m_config.maxRpm;
		m_controlFlags |= ControlFlagAtMaximum;
	} else {
		result = static_cast<uint16_t>(raw);
	}
	m_coolingDemandRaw =
			m_config.maxRpm > minimum
					? static_cast<uint8_t>(std::min<uint32_t>(
							  200, static_cast<uint32_t>(result - minimum) * 200 / (m_config.maxRpm - minimum)))
					: 200;

	const int16_t actualError = inputs.coolant - m_config.targetTemp;
	const bool ramAirAtCapacity =
			m_config.airflowFull > 0 && inputs.vehicleSpeedValid && inputs.vehicleSpeedKph >= m_config.airflowFull;
	if (result == m_config.maxRpm && (inputs.airflowAtMaximumCapacity || ramAirAtCapacity) &&
		actualError >= m_config.coolingLimitedDelta) {
		m_controlFaults |= EmpPumpFaultCoolingLimited;
		if (m_saturationSinceMs == 0) {
			m_saturationSinceMs = now;
		}
		if (actualError >= m_config.overloadDelta &&
			elapsed(now, m_saturationSinceMs, static_cast<uint32_t>(m_config.overloadDelay) * 1000)) {
			m_thermalState = EmpPumpThermalState::Overload;
			m_controlFaults |= EmpPumpFaultThermalOverload;
		} else {
			m_thermalState = EmpPumpThermalState::CapacityLimited;
		}
	} else {
		m_saturationSinceMs = 0;
		m_thermalState = inputs.coolant < (m_config.targetTemp - m_config.deadband) && result == minimum
							   ? EmpPumpThermalState::Warmup
							   : EmpPumpThermalState::ClosedLoop;
	}
	return result;
}

void EmpPump::updatePowerHoldRequest(const Inputs& inputs) {
	const bool armWhileRunning = m_state == EmpPumpState::EngineActive && inputs.coolantValid &&
								 inputs.coolant >= m_config.afterRunStartTemp;
	const bool keepAfterRun = m_state == EmpPumpState::AfterRun && !m_batteryInhibit && m_requestedRpm > 0;
	const bool wanted = m_config.afterRunEnabled && m_config.powerHoldEnabled && (armWhileRunning || keepAfterRun);
	if (wanted != m_powerHoldWanted) {
		m_powerHoldWanted = wanted;
		m_commandDirty = true;
	}
	m_delayedShutoffRequested.store(
			isEnabled() && m_config.powerHoldEnabled && (wanted || m_powerHoldCommanded), std::memory_order_release);
}

void EmpPump::applyRamp(uint32_t now) {
	if (m_requestedRpm == 0) {
		if (m_targetRpm != 0) {
			m_targetRpm = 0;
			m_commandDirty = true;
		}
		return;
	}
	const uint16_t desired = std::clamp(m_requestedRpm, m_config.minRpm, m_config.maxRpm);
	if (m_targetRpm == 0 || m_config.rampRpmPerSecond == 0) {
		if (m_targetRpm != desired) {
			m_targetRpm = desired;
			m_commandDirty = true;
		}
		return;
	}
	uint32_t step = static_cast<uint32_t>(m_config.rampRpmPerSecond) * (now - m_lastUpdateMs) / 1000;
	if (step == 0 && now != m_lastUpdateMs) {
		step = 1;
	}
	uint16_t next = m_targetRpm;
	if (next < desired) {
		next = static_cast<uint16_t>(std::min<uint32_t>(desired, next + step));
	} else if (next > desired) {
		next = static_cast<uint16_t>(step >= static_cast<uint32_t>(next - desired) ? desired : next - step);
	}
	if (next != m_targetRpm) {
		m_targetRpm = next;
		m_commandDirty = true;
	}
}

void EmpPump::updateStatusFaults() {
	m_diagnosticFaults = 0;
	CanBusIndex activeBus;
	uint8_t ignoredAddress;
	if (!decodeEndpoint(m_rxEndpoint.load(std::memory_order_acquire), activeBus, ignoredAddress)) {
		return;
	}
	RxTelemetry telemetry;
	if (!m_rxMailboxes[static_cast<size_t>(activeBus)].tryRead(telemetry) ||
		telemetry.generation != m_activeRxGeneration.load(std::memory_order_acquire)) {
		return;
	}
	const uint8_t caps = telemetry.capabilities;
	if (caps & EmpPumpCapStatus1) {
		const uint8_t state = telemetry.status1ControllerState;
		if (state == 20) {
			m_diagnosticFaults |= EmpPumpFaultServiceRequired | EmpPumpFaultNotOperable;
		} else if (state == 22 || state == 23 || state == 25) {
			m_diagnosticFaults |= EmpPumpFaultNotOperable;
		} else if (state == 6 || state == 27 || state == 28 || state == 29 || state == 30) {
			m_diagnosticFaults |= EmpPumpFaultDerated;
		}
	}
	if (caps & EmpPumpCapStatus2) {
		if (telemetry.status2ServiceState == 1) {
			m_diagnosticFaults |= EmpPumpFaultServiceRequired;
		}
		if (telemetry.status2OperationState == 1) {
			m_diagnosticFaults |= EmpPumpFaultDerated;
		}
		if (telemetry.status2OperationState == 2) {
			m_diagnosticFaults |= EmpPumpFaultNotOperable;
		}
		if (telemetry.status2CommandSource != 1) {
			m_diagnosticFaults |= EmpPumpFaultCommandNotExternal;
		}
	}
	if (caps & EmpPumpCapStatus3) {
		if (telemetry.hvilState != 0) {
			m_diagnosticFaults |= EmpPumpFaultHvil;
		}
	}
}

void EmpPump::sendCommandIfDue(uint32_t now) {
	if (!isEnabled() && m_state == EmpPumpState::Disabled && m_previousState == EmpPumpState::Disabled &&
		!m_transmitSeen) {
		return;
	}
	if (!m_commandDirty && m_transmitSeen && !elapsed(now, m_lastTransmitMs, HeartbeatMs)) {
		return;
	}
	transmitCommand(now);
}

bool EmpPump::queueTx(CanBusIndex bus, const TxCommand& command) {
	const size_t busIndex = static_cast<size_t>(bus);
	if (busIndex >= m_txQueues.size()) {
		return false;
	}
	auto& queue = m_txQueues[busIndex];
	const uint32_t write = queue.write.load(std::memory_order_relaxed);
	const uint32_t read = queue.read.load(std::memory_order_acquire);
	if (write - read >= TxQueueCapacity) {
		return false;
	}
	queue.commands[write % TxQueueCapacity] = command;
	queue.write.store(write + 1, std::memory_order_release);
	return true;
}

bool EmpPump::txQueueEmpty(CanBusIndex bus) const {
	const size_t busIndex = static_cast<size_t>(bus);
	if (busIndex >= m_txQueues.size()) {
		return true;
	}
	const auto& queue = m_txQueues[busIndex];
	return queue.read.load(std::memory_order_acquire) == queue.write.load(std::memory_order_acquire);
}

void EmpPump::recordTxQueueOverflow() {
	m_latchedFaults |= EmpPumpFaultTx;
	if (m_transmitFailureCount < UINT8_MAX) {
		m_transmitFailureCount++;
	}
}

void EmpPump::pollTx(CanBusIndex bus) {
	const size_t busIndex = static_cast<size_t>(bus);
	if (busIndex >= m_txQueues.size()) {
		return;
	}
	auto& queue = m_txQueues[busIndex];
	const uint32_t read = queue.read.load(std::memory_order_relaxed);
	if (read == queue.write.load(std::memory_order_acquire)) {
		return;
	}
	const TxCommand command = queue.commands[read % TxQueueCapacity];
#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
	{
		CanTxMessage msg(command.id, 8, bus, true);
		msg[0] = command.control;
		msg[1] = command.rawSpeed & 0xff;
		msg[2] = command.rawSpeed >> 8;
		for (size_t i = 3; i < 8; i++) {
			msg[i] = 0xff;
		}
	}
#else
	(void)command;
#endif
	// Keep the slot occupied until CanTxMessage has completed its send attempt.
	// A configuration change can then use an empty queue as its ordering barrier.
	queue.read.store(read + 1, std::memory_order_release);
}

void EmpPump::transmitCommand(uint32_t now) {
	uint8_t control = ControlOff;
	if (m_targetRpm > 0) {
		control = m_powerHoldWanted && m_config.powerHoldEnabled
						? ControlForwardPowerHoldOn
						: (m_powerHoldCommanded ? ControlForwardPowerHoldOff : ControlForward);
	} else if (m_powerHoldCommanded) {
		control = ControlOffPowerHoldOff;
	}
	const uint32_t id =
			CommandBaseId | (static_cast<uint32_t>(m_config.controllerAddress) << 8) | m_config.sourceAddress;
	const uint16_t rawSpeed = m_targetRpm > 0 ? m_targetRpm * 2 : 0xffff;
	if (!queueTx(static_cast<CanBusIndex>(m_config.canBus), {id, rawSpeed, control})) {
		recordTxQueueOverflow();
		return;
	}
	m_lastCommandControl = control;
	if (control == ControlForwardPowerHoldOn) {
		m_powerHoldCommanded = true;
	} else if (control == ControlOffPowerHoldOff || control == ControlForwardPowerHoldOff) {
		m_powerHoldCommanded = false;
	}
	m_delayedShutoffRequested.store(
			isEnabled() && m_config.powerHoldEnabled && (m_powerHoldWanted || m_powerHoldCommanded),
			std::memory_order_release);
	m_lastTransmitMs = now;
	m_transmitSeen = true;
	m_commandDirty = false;
	if (m_targetRpm == 0) {
		m_previousState = m_state;
	}
}

bool EmpPump::releasePowerHold() {
	if (!m_powerHoldCommanded || m_releaseQueued) {
		return true;
	}
	const uint32_t oldId =
			CommandBaseId | (static_cast<uint32_t>(m_config.controllerAddress) << 8) | m_config.sourceAddress;
	if (!queueTx(static_cast<CanBusIndex>(m_config.canBus), {oldId, 0xffff, ControlOffPowerHoldOff})) {
		recordTxQueueOverflow();
		return false;
	}
	m_releaseQueued = true;
	return true;
}

void EmpPump::consumeServiceMailbox(uint32_t now) {
	const uint32_t payload = m_serviceMailbox.exchange(0, std::memory_order_acq_rel);
	if (payload == 0) {
		return;
	}

	const auto command = static_cast<EmpPumpServiceCommand>((payload & 0xff) - 1);
	const uint16_t rpm = static_cast<uint16_t>((payload >> 8) & 0xffff);
	const uint8_t durationSeconds = static_cast<uint8_t>(payload >> 24);
	switch (command) {
		case EmpPumpServiceCommand::Stop:
			m_serviceEndMs = 0;
			m_serviceRpm = 0;
			m_commandDirty = true;
			break;
		case EmpPumpServiceCommand::Start:
			// Validation is repeated here because a Burn can happen between mailbox
			// submission and the slow-control owner consuming it.
			if (isEnabled() && rpm >= m_config.minRpm && rpm <= m_config.maxRpm && durationSeconds != 0) {
				m_serviceRpm = rpm;
				m_serviceEndMs = now + static_cast<uint32_t>(durationSeconds) * 1000;
				m_commandDirty = true;
			}
			break;
		case EmpPumpServiceCommand::ClearFaults:
			m_latchedFaults = 0;
			m_transmitFailureCount = 0;
			break;
	}
}

bool EmpPump::handleServiceCommand(EmpPumpServiceCommand command, uint16_t rpm, uint8_t durationSeconds) {
	EmpPumpConfig configuration;
	if (m_activeConfigurationPublished.load(std::memory_order_acquire)) {
		if (!m_activeConfigurationMailbox.tryRead(configuration)) {
			return false;
		}
	} else if (!m_configurationMailbox.tryRead(configuration)) {
		return false;
	}
	switch (command) {
		case EmpPumpServiceCommand::Stop:
			m_serviceMailbox.store(static_cast<uint32_t>(command) + 1, std::memory_order_release);
			return true;
		case EmpPumpServiceCommand::Start: {
			if (rpm == 0) {
				rpm = configuration.manualTestRpm;
			}
			if (durationSeconds == 0) {
				durationSeconds = configuration.manualTestSeconds;
			}
			if (!configuration.enabled || !isConfigSane(configuration) || rpm < configuration.minRpm ||
				rpm > configuration.maxRpm || durationSeconds == 0) {
				return false;
			}
			const Inputs inputs = readInputs();
			if (!configuration.serviceAllowedWhileRunning && (inputs.engineRunning || inputs.engineCranking)) {
				return false;
			}
			m_serviceMailbox.store(
					(static_cast<uint32_t>(command) + 1) | (static_cast<uint32_t>(rpm) << 8) |
							(static_cast<uint32_t>(durationSeconds) << 24),
					std::memory_order_release);
			return true;
		}
		case EmpPumpServiceCommand::ClearFaults:
			m_serviceMailbox.store(static_cast<uint32_t>(command) + 1, std::memory_order_release);
			return true;
	}
	return false;
}

void EmpPump::reportTransmitResult(bool success) {
	// CanTxMessage has no production per-frame result. This is an explicit
	// test/future-driver hook consumed by the slow owner.
	m_transmitResultMailbox.store(success ? 1 : -1, std::memory_order_release);
}

EmpPumpStatus EmpPump::getStatus() const {
	EmpPumpStatus status;
	for (int attempt = 0; attempt < 3; attempt++) {
		if (m_statusMailbox.tryRead(status)) {
			break;
		}
	}
	const uint32_t now = nowMs();
	status = readRxStatus(status, now);
	const uint32_t timeout = m_statusTimeoutMs.load(std::memory_order_acquire);
	const uint32_t stateSince = m_statusStateSinceMs.load(std::memory_order_acquire);
	if (timeout != 0 && (status.state == EmpPumpState::EngineActive || status.state == EmpPumpState::AfterRun ||
						 status.state == EmpPumpState::ServiceTest)) {
		const uint32_t reference = status.mainStatusAgeMs == UINT16_MAX ? stateSince : now - status.mainStatusAgeMs;
		if (elapsed(now, reference, timeout)) {
			status.faults |= EmpPumpFaultStatusTimeout;
		}
	}
	return status;
}

EmpPumpStatus EmpPump::readRxStatus(EmpPumpStatus status, uint32_t now) const {
	CanBusIndex activeBus;
	uint8_t ignoredAddress;
	if (!decodeEndpoint(m_rxEndpoint.load(std::memory_order_acquire), activeBus, ignoredAddress)) {
		return status;
	}
	RxTelemetry telemetry;
	if (!m_rxMailboxes[static_cast<size_t>(activeBus)].tryRead(telemetry) ||
		telemetry.generation != m_activeRxGeneration.load(std::memory_order_acquire)) {
		return status;
	}
	status.capabilities = telemetry.capabilities;
	status.actualRpm = telemetry.actualRpm;
	status.actualPercentRaw = telemetry.actualPercentRaw;
	status.controllerStatus = telemetry.controllerStatus;
	status.statusSummary = telemetry.statusSummary;
	status.hvilState = telemetry.hvilState;
	status.voltageRaw = telemetry.voltageRaw;
	status.currentRaw = telemetry.currentRaw;
	status.powerWatts = telemetry.powerWatts;
	status.externalTemperatureRaw = telemetry.externalTemperatureRaw;
	status.mainStatusAgeMs = telemetry.mainStatusSeen ? saturatedU16(now - telemetry.lastMainStatusMs) : UINT16_MAX;
	return status;
}

void EmpPump::publishStatus(uint32_t now) {
	EmpPumpStatus status;
	status.state = m_state;
	status.thermalState = m_thermalState;
	status.targetRpm = m_targetRpm;
	status.transmitFailureCount = m_transmitFailureCount;
	status.lastCommandControl = m_lastCommandControl;
	status.controlFlags = m_controlFlags;
	status.filteredIat = m_diagnosticFilteredIat;
	status.temperatureError = m_diagnosticTemperatureError;
	status.coolantSlopePerMinute = m_coolantSlopePerMinute;
	status.minimumFlowRpm = m_diagnosticMinimumFlowRpm;
	status.feedForwardRpm = m_diagnosticFeedForwardRpm;
	status.piCorrectionRpm = m_diagnosticPiCorrectionRpm;
	status.coolingDemandRaw = m_coolingDemandRaw;
	if (m_state == EmpPumpState::AfterRun && !reached(now, m_afterRunEndMs)) {
		status.afterRunRemainingSeconds = saturatedU16((m_afterRunEndMs - now + 999) / 1000);
	}
	if (m_saturationSinceMs) {
		status.saturationSeconds = saturatedU16((now - m_saturationSinceMs) / 1000);
	}
	status.faults = m_latchedFaults | m_diagnosticFaults | m_controlFaults;
	if (m_targetRpm == 0) {
		status.faults &= static_cast<uint16_t>(~EmpPumpFaultCommandNotExternal);
	}
	const uint32_t timeout = static_cast<uint32_t>(m_config.statusTimeoutSeconds) * 1000;
	m_statusStateSinceMs.store(m_stateSinceMs, std::memory_order_release);
	m_statusTimeoutMs.store(timeout, std::memory_order_release);
	status = readRxStatus(status, now);
	if ((m_state == EmpPumpState::EngineActive || m_state == EmpPumpState::AfterRun ||
		 m_state == EmpPumpState::ServiceTest) &&
		(status.mainStatusAgeMs == UINT16_MAX ? elapsed(now, m_stateSinceMs, timeout)
											  : status.mainStatusAgeMs >= timeout)) {
		status.faults |= EmpPumpFaultStatusTimeout;
	}
	m_statusMailbox.publish(status);
}
