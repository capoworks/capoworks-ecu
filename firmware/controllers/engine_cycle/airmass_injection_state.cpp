#include "pch.h"
#include "airmass_injection_state.h"
#include "airmass_loads.h"

static bool needsStandalonePublication(engine_load_mode_e mode) {
	return mode == LM_SPEED_DENSITY || mode == LM_ALPHA_N || mode == LM_REAL_MAF;
}

void AirmassInjectionState::setStatus(AirmassInjectionStatus status, AirmassInjectionFault fault) {
	m_status = status;
	m_fault = fault;
}

void AirmassInjectionState::failCalculation(AirmassInjectionFault fault) {
	setStatus(AirmassInjectionStatus::Faulted, fault);
	m_calculationAccepted = false;
	m_standaloneReady = false;
	++m_epoch;
}

void AirmassInjectionState::observeMode(engine_load_mode_e mode) {
	if (!m_observedMode || m_mode != mode) {
		if (m_observedMode) {
			++m_epoch;
		}
		m_mode = mode;
		m_observedMode = true;
		m_calculationAccepted = false;
		m_standaloneReady = false;
		m_calculationFallback = AirmassInjectionFault::None;
		setStatus(AirmassInjectionStatus::Legacy, AirmassInjectionFault::None);
	}
}

AirmassInjectionState::CalculationToken
AirmassInjectionState::beginCalculation(engine_load_mode_e mode, float rpm, int configurationVersion) {
	chibios_rt::CriticalSectionLocker csl;
	observeMode(mode);
	++m_epoch;
	m_positiveRpmCalculation = std::isfinite(rpm) && rpm > 0;
	// Keep the last completed publication available while its replacement is
	// calculated. Stops, tune changes, and actual failures close it immediately.
	const bool invalidatesPublication = !m_positiveRpmCalculation || m_configurationVersion != configurationVersion;
	if (invalidatesPublication) {
		m_standaloneReady = false;
	}
	m_configurationVersion = configurationVersion;
	m_calculationAccepted = false;
	m_calculationFallback = AirmassInjectionFault::None;
	if (invalidatesPublication &&
		(m_status == AirmassInjectionStatus::Ready || m_status == AirmassInjectionStatus::Degraded)) {
		setStatus(AirmassInjectionStatus::NotReady, m_fault);
	}
	return m_epoch;
}

void AirmassInjectionState::acceptCalculation(AirmassInjectionFault fallback) {
	chibios_rt::CriticalSectionLocker csl;
	m_calculationAccepted = true;
	m_calculationFallback = fallback;
}

void AirmassInjectionState::rejectCalculation(AirmassInjectionFault fault) {
	chibios_rt::CriticalSectionLocker csl;
	rejectCalculationLocked(fault);
}

void AirmassInjectionState::rejectCalculationLocked(AirmassInjectionFault fault) {
	observeMode(engineConfiguration->fuelAlgorithm);
	if (needsStandalonePublication(m_mode) && m_positiveRpmCalculation) {
		failCalculation(fault);
	} else if (needsStandalonePublication(m_mode)) {
		m_standaloneReady = false;
		m_calculationAccepted = false;
		++m_epoch;
	}
}

bool AirmassInjectionState::isCalculationCurrent(CalculationToken token) {
	chibios_rt::CriticalSectionLocker csl;
	return isCalculationCurrentLocked(token);
}

bool AirmassInjectionState::isCalculationCurrentLocked(CalculationToken token) {
	observeMode(engineConfiguration->fuelAlgorithm);
	return token == m_epoch && m_configurationVersion == engine->getGlobalConfigurationVersion();
}

AirmassInjectionState::CalculationToken AirmassInjectionState::publicationEpoch() const {
	chibios_rt::CriticalSectionLocker csl;
	return m_epoch;
}

void AirmassInjectionState::completeCalculation(CalculationToken token, bool publicationValid) {
	chibios_rt::CriticalSectionLocker csl;
	completeCalculationLocked(token, publicationValid);
}

void AirmassInjectionState::completeCalculationLocked(CalculationToken token, bool publicationValid) {
	observeMode(engineConfiguration->fuelAlgorithm);
	// A stale completion cannot close or reopen a newer publication.
	if (token != m_epoch || m_configurationVersion != engine->getGlobalConfigurationVersion()) {
		return;
	}
	if (!needsStandalonePublication(m_mode)) {
		return;
	}
	auto rpm = Sensor::get(SensorType::Rpm);
	if (!m_positiveRpmCalculation || !rpm || !std::isfinite(rpm.Value) || rpm.Value <= 0) {
		m_standaloneReady = false;
		setStatus(AirmassInjectionStatus::NotReady, m_fault);
		return;
	}
	if (!publicationValid || !engine->engineState.airmassCalculationValid) {
		failCalculation(AirmassInjectionFault::Result);
		return;
	}
	m_standaloneReady = true;
	auto status = m_calculationFallback != AirmassInjectionFault::None ? AirmassInjectionStatus::Degraded
																	   : AirmassInjectionStatus::Legacy;
	setStatus(status, m_calculationFallback);
}

void AirmassInjectionState::onEngineStop() {
	chibios_rt::CriticalSectionLocker csl;
	++m_epoch;
	m_calculationAccepted = false;
	m_positiveRpmCalculation = false;
	m_standaloneReady = false;
	setStatus(AirmassInjectionStatus::Legacy, AirmassInjectionFault::None);
}

void AirmassInjectionState::onConfigurationWrite(engine_load_mode_e proposedMode, bool strategyChanged) {
	chibios_rt::CriticalSectionLocker csl;
	// This hook is called before every persistent/calibration mutation while
	// the global Engine is valid. onEngineStop intentionally does not do this:
	// it also runs from the Engine constructor before the global is assigned.
	invalidateAirmassConsumerLoads();
	invalidateAirmassCalibration();
	observeMode(engineConfiguration->fuelAlgorithm);
	++m_epoch;
	m_calculationAccepted = false;
	m_standaloneReady = false;
	if (strategyChanged) {
		observeMode(proposedMode);
	}
	setStatus(AirmassInjectionStatus::Legacy, AirmassInjectionFault::None);
}

bool AirmassInjectionState::allowInjection() {
	chibios_rt::CriticalSectionLocker csl;
	return allowInjectionLocked();
}

bool AirmassInjectionState::allowInjectionLocked() {
	observeMode(engineConfiguration->fuelAlgorithm);
	if (needsStandalonePublication(m_mode)) {
		// airmassCalculationValid describes the in-progress calculation. Only
		// completeCalculation may admit its final per-cylinder publication.
		return m_standaloneReady && m_configurationVersion == engine->getGlobalConfigurationVersion();
	}
	return m_status == AirmassInjectionStatus::Legacy ||
		   ((m_status == AirmassInjectionStatus::Ready || m_status == AirmassInjectionStatus::Degraded) &&
			m_configurationVersion == engine->getGlobalConfigurationVersion());
}

bool AirmassInjectionState::allowPrime() {
	chibios_rt::CriticalSectionLocker csl;
	return allowPrimeLocked();
}

bool AirmassInjectionState::allowPrimeLocked() {
	// Priming uses its own upstream CLT, key-cycle, RPM and flood-clear checks.
	return true;
}

AirmassInjectionStatus AirmassInjectionState::status() const {
	chibios_rt::CriticalSectionLocker csl;
	return m_status;
}

AirmassInjectionFault AirmassInjectionState::fault() const {
	chibios_rt::CriticalSectionLocker csl;
	return m_fault;
}

uint16_t AirmassInjectionState::pendingCallbacks() const {
	chibios_rt::CriticalSectionLocker csl;
	return m_pendingCallbacks;
}

bool AirmassInjectionState::callbacksAccepted(size_t count) {
	if (count > static_cast<size_t>(UINT16_MAX - m_pendingCallbacks)) {
		return false;
	}
	m_pendingCallbacks += count;
	return true;
}

void AirmassInjectionState::callbacksRejected(size_t count) {
	m_pendingCallbacks -= count;
}

void AirmassInjectionState::callbackCompleted() {
	chibios_rt::CriticalSectionLocker csl;
	callbackCompletedLocked();
}

void AirmassInjectionState::callbackCompletedLocked() {
	efiAssertVoid(ObdCode::CUSTOM_ERR_ASSERT, m_pendingCallbacks > 0, "untracked injection callback");
	--m_pendingCallbacks;
}

void AirmassInjectionState::LockedAccess::rejectCalculation(AirmassInjectionFault fault) {
	m_state.rejectCalculationLocked(fault);
}

bool AirmassInjectionState::LockedAccess::isCalculationCurrent(CalculationToken token) {
	return m_state.isCalculationCurrentLocked(token);
}

void AirmassInjectionState::LockedAccess::completeCalculation(CalculationToken token, bool publicationValid) {
	m_state.completeCalculationLocked(token, publicationValid);
}

bool AirmassInjectionState::LockedAccess::allowInjection() {
	return m_state.allowInjectionLocked();
}

bool AirmassInjectionState::LockedAccess::allowPrime() {
	return m_state.allowPrimeLocked();
}

bool AirmassInjectionState::LockedAccess::callbacksAccepted(size_t count) {
	return m_state.callbacksAccepted(count);
}

void AirmassInjectionState::LockedAccess::callbacksRejected(size_t count) {
	m_state.callbacksRejected(count);
}

AirmassInjectionState::CalculationToken AirmassInjectionState::LockedAccess::publicationEpoch() const {
	return m_state.m_epoch;
}

void AirmassInjectionState::LockedAccess::callbackCompleted() {
	m_state.callbackCompletedLocked();
}

bool scheduleFuelCallbacks(const ScheduledAction* events, size_t count, bool prime) {
	chibios_rt::CriticalSectionLocker csl;
	auto state = engine->airmassInjectionState.locked(csl);
	if (!(prime ? state.allowPrime() : state.allowInjection())) {
		return false;
	}
	// Executors validate once, before reserving/exposing any callback.
	if (!state.callbacksAccepted(count)) {
		if (!prime) {
			state.rejectCalculation(AirmassInjectionFault::Scheduling);
		}
		return false;
	}
	if (!getScheduler()->scheduleBatch(events, count)) {
		state.callbacksRejected(count);
		if (!prime) {
			state.rejectCalculation(AirmassInjectionFault::Scheduling);
		}
		return false;
	}
	return true;
}
