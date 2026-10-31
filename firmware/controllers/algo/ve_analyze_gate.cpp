/**
 * @file ve_analyze_gate.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "ve_analyze_gate.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr uint8_t TimestampFaultNone = 0;
constexpr uint8_t TimestampFaultDuplicate = 1;
constexpr uint8_t TimestampFaultRegressed = 2;

constexpr uint8_t HistoryFaultNone = 0;
constexpr uint8_t HistoryFaultGap = 1;
constexpr uint8_t HistoryFaultInvalid = 2;

constexpr uint8_t TransientUnknown = 0;
constexpr uint8_t TransientInactive = 1;
constexpr uint8_t TransientActive = 2;
constexpr uint8_t InactiveTransientStates = 0x55;

int64_t secondsToNt(float seconds) {
	return static_cast<int64_t>(USF2NT(seconds * US_PER_SECOND_F));
}

int64_t saturatingAdd(efitick_t nowNt, int64_t durationNt) {
	constexpr int64_t maximum = std::numeric_limits<int64_t>::max();
	if (durationNt > 0 && nowNt.count > maximum - durationNt) {
		return maximum;
	}
	return nowNt.count + durationNt;
}

bool validPositivePolicy(float value) {
	return std::isfinite(value) && value > 0 && value <= ve_analyze_gate::MaximumPolicySeconds;
}

bool validLambdaDelay(float value) {
	if (value == ve_analyze_gate::NoLambdaDelay) {
		return true;
	}
	return std::isfinite(value) && value >= 0 && value <= ve_analyze_gate::MaximumPolicySeconds;
}

uint32_t reasonBit(VeAnalyzeGateReason reason) {
	return static_cast<uint32_t>(reason);
}

VeAnalyzeGateReason transientReason(VeAnalyzeTransient transient) {
	if (transient == VeAnalyzeTransient::FuelCut) {
		return VeAnalyzeGateReason::FuelCutRecovery;
	}
	return transient == VeAnalyzeTransient::FuelCorrection ? VeAnalyzeGateReason::FuelCorrectionRecovery
														   : VeAnalyzeGateReason::CalibrationRecovery;
}

uint32_t saturatedMilliseconds(uint64_t microseconds) {
	const uint64_t milliseconds = microseconds / 1000;
	const uint64_t maximum = std::numeric_limits<uint32_t>::max();
	return milliseconds > maximum ? static_cast<uint32_t>(maximum) : static_cast<uint32_t>(milliseconds);
}

uint32_t elapsedMilliseconds(efitick_t nowNt, efitick_t sinceNt) {
	if (nowNt.count <= sinceNt.count) {
		return 0;
	}

	const uint64_t elapsedNt = static_cast<uint64_t>(nowNt.count) - static_cast<uint64_t>(sinceNt.count);
	return saturatedMilliseconds(elapsedNt / static_cast<uint64_t>(US_TO_NT_MULTIPLIER));
}

uint32_t remainingMilliseconds(efitick_t nowNt, int64_t untilNt) {
	if (nowNt.count >= untilNt) {
		return 0;
	}

	const uint64_t remainingNt = static_cast<uint64_t>(untilNt) - static_cast<uint64_t>(nowNt.count);
	const uint64_t remainingUs = remainingNt / static_cast<uint64_t>(US_TO_NT_MULTIPLIER);
	const uint64_t milliseconds = (remainingUs + 999) / 1000;
	const uint64_t maximum = std::numeric_limits<uint32_t>::max();
	return milliseconds > maximum ? static_cast<uint32_t>(maximum) : static_cast<uint32_t>(milliseconds);
}
} // namespace

VeAnalyzeGate::VeAnalyzeGate() {
	configure(VeAnalyzeGateConfig{});
}

VeAnalyzeGate::VeAnalyzeGate(const VeAnalyzeGateConfig& config) {
	configure(config);
}

bool VeAnalyzeGate::configure(const VeAnalyzeGateConfig& config) {
	const bool validRate = validPositivePolicy(config.RateThresholdPercentPerSecond);
	const bool validQuickMovement = validPositivePolicy(config.QuickMovementThresholdPercent);
	const bool validSampleTolerance = validPositivePolicy(config.SampleToleranceSeconds);
	const bool validQuietHold = validPositivePolicy(config.QuietHoldSeconds);
	const bool validCutRecovery = validPositivePolicy(config.CutRecoverySeconds);
	const bool validDelay = validLambdaDelay(config.MaxLambdaDelaySeconds);
	const bool valid =
			validRate && validQuickMovement && validSampleTolerance && validQuietHold && validCutRecovery && validDelay;

	if (!valid) {
		m_config = config;
		m_configValid = false;
		m_normalHoldNt = 0;
		m_cutHoldNt = 0;
		m_sampleToleranceNt = 0;
		clearSamples();
		m_hasTpsUpdate = false;
		m_hasMovement = false;
		m_needStartupHold = true;
		return false;
	}

	const float lambdaWindowSeconds = config.MaxLambdaDelaySeconds == ve_analyze_gate::NoLambdaDelay
											? 0
											: config.MaxLambdaDelaySeconds + ve_analyze_gate::WindowSeconds;
	const float normalHoldSeconds = std::max(config.QuietHoldSeconds, lambdaWindowSeconds);
	const float cutHoldSeconds = std::max(config.CutRecoverySeconds, normalHoldSeconds);

	m_config = config;
	m_configValid = true;
	m_normalHoldNt = secondsToNt(normalHoldSeconds);
	m_cutHoldNt = secondsToNt(cutHoldSeconds);
	m_sampleToleranceNt = secondsToNt(config.SampleToleranceSeconds);
	clearSamples();
	m_hasTpsUpdate = false;
	m_hasMovement = false;
	m_needStartupHold = true;
	return true;
}

void VeAnalyzeGate::clearSamples() {
	m_head = 0;
	m_count = 0;
	m_tpsValid = false;
	m_rateValid = false;
	m_tpsMovementActive = false;
	m_absRatePercentPerSecond = 0;
	m_windowSeconds = 0;
	m_timestampFault = TimestampFaultNone;
	m_historyFault = HistoryFaultNone;
}

void VeAnalyzeGate::reset(efitick_t nowNt) {
	clearSamples();
	m_hasTpsUpdate = false;
	m_hasMovement = false;
	m_hasCorrection = false;
	m_hasExternalUpdate = false;
	m_transientStates = InactiveTransientStates;

	if (!m_configValid) {
		m_holdUntilNt = std::numeric_limits<int64_t>::min();
		m_holdReasons = reasonBit(VeAnalyzeGateReason::InvalidConfiguration);
		m_needStartupHold = true;
		return;
	}

	m_holdUntilNt = std::numeric_limits<int64_t>::min();
	m_holdReasons = 0;
	m_needStartupHold = false;
	startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::ResetRecovery);
}

int64_t VeAnalyzeGate::normalHoldNt() const {
	return m_normalHoldNt;
}

int64_t VeAnalyzeGate::cutHoldNt() const {
	return m_cutHoldNt;
}

void VeAnalyzeGate::startHold(efitick_t nowNt, int64_t durationNt, VeAnalyzeGateReason reason) {
	const int64_t candidateUntil = saturatingAdd(nowNt, durationNt);
	if (candidateUntil > m_holdUntilNt) {
		m_holdUntilNt = candidateUntil;
		m_holdReasons = reasonBit(reason);
	} else {
		m_holdReasons |= reasonBit(reason);
	}
}

void VeAnalyzeGate::reseedForFault(efitick_t nowNt, VeAnalyzeGateReason reason) {
	clearSamples();
	m_needStartupHold = false;
	startHold(nowNt, normalHoldNt(), reason);
}

bool VeAnalyzeGate::updateRate() {
	if (m_count < 2) {
		m_rateValid = false;
		m_absRatePercentPerSecond = 0;
		m_windowSeconds = 0;
		return false;
	}

	const Sample& latest = m_samples[(m_head + m_count - 1) % m_samples.size()];
	const int64_t windowNt = static_cast<int64_t>(USF2NT(ve_analyze_gate::WindowSeconds * US_PER_SECOND_F));
	const int64_t targetNt = latest.Timestamp.count - windowNt;

	size_t baseIndex = 0;
	bool foundBase = false;
	for (size_t index = 0; index < m_count; index++) {
		const Sample& sample = m_samples[(m_head + index) % m_samples.size()];
		if (sample.Timestamp.count > targetNt) {
			break;
		}
		baseIndex = index;
		foundBase = true;
	}

	if (!foundBase) {
		m_rateValid = false;
		m_absRatePercentPerSecond = 0;
		m_windowSeconds = 0;
		return false;
	}

	const Sample& base = m_samples[(m_head + baseIndex) % m_samples.size()];
	const int64_t spanNt = latest.Timestamp.count - base.Timestamp.count;
	if (spanNt < windowNt || spanNt <= 0) {
		m_rateValid = false;
		m_absRatePercentPerSecond = 0;
		m_windowSeconds = 0;
		return false;
	}

	const float spanSeconds = NT2USF(spanNt) * 0.000001f;
	const float rate = (latest.TpsPercent - base.TpsPercent) / spanSeconds;
	if (!std::isfinite(rate)) {
		m_rateValid = false;
		m_absRatePercentPerSecond = 0;
		m_windowSeconds = 0;
		return false;
	}

	m_absRatePercentPerSecond = std::fabs(rate);
	m_windowSeconds = spanSeconds;
	m_rateValid = true;
	if (m_historyFault == HistoryFaultGap) {
		m_historyFault = HistoryFaultNone;
	}
	return true;
}

void VeAnalyzeGate::appendSample(efitick_t nowNt, float tpsPercent) {
	if (m_needStartupHold) {
		startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::StartupRecovery);
		m_needStartupHold = false;
	}

	const bool hasPrevious = m_count > 0;
	const Sample previous = hasPrevious ? m_samples[(m_head + m_count - 1) % m_samples.size()] : Sample{};

	if (m_count < m_samples.size()) {
		m_samples[(m_head + m_count) % m_samples.size()] = {nowNt, tpsPercent};
		m_count++;
	} else {
		m_samples[m_head] = {nowNt, tpsPercent};
		m_head = (m_head + 1) % m_samples.size();
	}

	m_tpsValid = true;
	updateRate();

	const bool quickMovement =
			hasPrevious && std::fabs(tpsPercent - previous.TpsPercent) > m_config.QuickMovementThresholdPercent;
	const bool windowMovement = m_rateValid && m_absRatePercentPerSecond > m_config.RateThresholdPercentPerSecond;
	const bool tpsMovementActive = quickMovement || windowMovement;
	if (tpsMovementActive) {
		m_hasMovement = true;
		m_lastMovement = nowNt;
		startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::TpsMovementRecovery);
	} else if (m_tpsMovementActive) {
		// Start the complete recovery hold when the movement window closes.
		startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::TpsMovementRecovery);
	}
	m_tpsMovementActive = tpsMovementActive;
}

bool VeAnalyzeGate::updateTps(efitick_t nowNt, float tpsPercent, bool sensorValid, bool fresh) {
	if (!m_configValid) {
		return false;
	}

	const bool timestampRegressed = m_hasTpsUpdate && nowNt.count < m_lastTpsUpdate.count;
	if (!timestampRegressed && m_hasTpsUpdate && nowNt.count == m_lastTpsUpdate.count) {
		m_timestampFault = TimestampFaultDuplicate;
		if (!sensorValid || !std::isfinite(tpsPercent)) {
			clearSamples();
			m_timestampFault = TimestampFaultDuplicate;
			m_tpsValid = false;
			m_historyFault = HistoryFaultInvalid;
			startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::InvalidTps);
		}
		return false;
	}

	m_lastTpsUpdate = nowNt;
	m_hasTpsUpdate = true;
	if (timestampRegressed) {
		reseedForFault(nowNt, VeAnalyzeGateReason::RegressedTimestamp);
		m_timestampFault = TimestampFaultRegressed;
	} else {
		m_timestampFault = TimestampFaultNone;
	}

	if (!sensorValid || !std::isfinite(tpsPercent)) {
		clearSamples();
		m_tpsValid = false;
		m_historyFault = HistoryFaultInvalid;
		startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::InvalidTps);
		return false;
	}

	if (!fresh) {
		return false;
	}

	if (m_count > 0) {
		const Sample& previous = m_samples[(m_head + m_count - 1) % m_samples.size()];
		const int64_t gapNt = nowNt.count - previous.Timestamp.count;
		if (gapNt > m_sampleToleranceNt) {
			clearSamples();
			m_historyFault = HistoryFaultGap;
			m_needStartupHold = false;
			startHold(nowNt, normalHoldNt(), VeAnalyzeGateReason::SampleGap);
		}
	}

	appendSample(nowNt, tpsPercent);
	return true;
}

size_t VeAnalyzeGate::transientIndex(VeAnalyzeTransient transient) {
	return static_cast<size_t>(transient);
}

uint8_t VeAnalyzeGate::transientState(VeAnalyzeTransient transient) const {
	return (m_transientStates >> (transientIndex(transient) * 2)) & 0x3;
}

void VeAnalyzeGate::setTransientState(VeAnalyzeTransient transient, uint8_t state) {
	const size_t shift = transientIndex(transient) * 2;
	m_transientStates = (m_transientStates & ~(0x3 << shift)) | (state << shift);
}

void VeAnalyzeGate::observeTransient(efitick_t nowNt, VeAnalyzeTransient transient, bool active) {
	if (!m_configValid || transientIndex(transient) >= static_cast<size_t>(VeAnalyzeTransient::Count)) {
		return;
	}

	if (m_hasExternalUpdate && nowNt.count < m_lastExternalUpdate.count) {
		m_hasExternalUpdate = false;
		reseedForFault(nowNt, VeAnalyzeGateReason::RegressedTimestamp);
		m_timestampFault = TimestampFaultRegressed;
	}

	const uint8_t previousState = transientState(transient);
	if (active) {
		setTransientState(transient, TransientActive);
		m_hasCorrection = true;
		m_lastCorrection = nowNt;
		startHold(
				nowNt,
				transient == VeAnalyzeTransient::FuelCut ? cutHoldNt() : normalHoldNt(),
				transientReason(transient));
	} else if (previousState == TransientActive) {
		setTransientState(transient, TransientInactive);
		m_hasCorrection = true;
		m_lastCorrection = nowNt;
		startHold(
				nowNt,
				transient == VeAnalyzeTransient::FuelCut ? cutHoldNt() : normalHoldNt(),
				transientReason(transient));
	} else if (previousState == TransientUnknown) {
		setTransientState(transient, TransientInactive);
	}

	m_lastExternalUpdate = nowNt;
	m_hasExternalUpdate = true;
}

VeAnalyzeGateSnapshot VeAnalyzeGate::snapshot(efitick_t nowNt) const {
	VeAnalyzeGateSnapshot result;
	if (!m_configValid) {
		result.Reasons = reasonBit(VeAnalyzeGateReason::InvalidConfiguration);
		return result;
	}

	result.HasObservedMovement = m_hasMovement;
	result.HasObservedTransient = m_hasCorrection;
	result.MillisecondsSinceMovement =
			m_hasMovement ? elapsedMilliseconds(nowNt, m_lastMovement) : std::numeric_limits<uint32_t>::max();
	result.MillisecondsSinceCorrection =
			m_hasCorrection ? elapsedMilliseconds(nowNt, m_lastCorrection) : std::numeric_limits<uint32_t>::max();

	bool timestampValid = m_timestampFault == TimestampFaultNone;
	if (m_timestampFault == TimestampFaultDuplicate) {
		result.Reasons |= reasonBit(VeAnalyzeGateReason::DuplicateTimestamp);
	} else if (m_timestampFault == TimestampFaultRegressed) {
		result.Reasons |= reasonBit(VeAnalyzeGateReason::RegressedTimestamp);
	}

	if (m_count > 0 && m_tpsValid) {
		const Sample& latest = m_samples[(m_head + m_count - 1) % m_samples.size()];
		result.TpsPercent = latest.TpsPercent;
		result.AbsTpsRatePercentPerSecond = m_absRatePercentPerSecond;
		result.WindowSeconds = m_windowSeconds;

		if (nowNt.count < latest.Timestamp.count) {
			result.Reasons |= reasonBit(VeAnalyzeGateReason::RegressedTimestamp);
			timestampValid = false;
		} else {
			const int64_t ageNt = nowNt.count - latest.Timestamp.count;
			if (ageNt > m_sampleToleranceNt) {
				result.Reasons |= reasonBit(VeAnalyzeGateReason::StaleTps);
				result.TpsValid = false;
			} else {
				result.TpsValid = true;
			}
		}
	} else {
		result.Reasons |= reasonBit(VeAnalyzeGateReason::InvalidTps);
	}

	if (!m_rateValid) {
		result.Reasons |= reasonBit(VeAnalyzeGateReason::InsufficientHistory);
		if (m_historyFault == HistoryFaultGap) {
			result.Reasons |= reasonBit(VeAnalyzeGateReason::SampleGap);
		}
	}

	result.RateValid = result.TpsValid && m_rateValid && timestampValid;

	bool hasActiveExternalCondition = false;
	for (size_t index = 0; index < static_cast<size_t>(VeAnalyzeTransient::Count); index++) {
		const auto transient = static_cast<VeAnalyzeTransient>(index);
		if (transientState(transient) == TransientActive) {
			result.Reasons |= reasonBit(transientReason(transient));
			hasActiveExternalCondition = true;
		}
	}

	if (m_holdUntilNt > nowNt.count) {
		result.HoldRemainingMilliseconds = remainingMilliseconds(nowNt, m_holdUntilNt);
		result.Reasons |= m_holdReasons;
	}

	result.Ready = result.TpsValid && result.RateValid && result.HoldRemainingMilliseconds == 0 && timestampValid &&
				   !hasActiveExternalCondition;
	return result;
}
