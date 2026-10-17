#include "pch.h"
#include "ignition_cycle_profile.h"

void IgnitionCycleProfile::reset() {
	m_count = 0;
	m_head = 0;
	m_valid = false;
}

void IgnitionCycleProfile::configure(const TriggerWaveform& shape, const TriggerFormDetails& details) {
	reset();
	m_slots = 0;
	m_details = &details;
	const auto length = shape.getLength();
	// Initial support is deliberately narrow: a single rising-only crank wheel,
	// repeated twice over 720 degrees. Validate sparse paired slots before using stride 2.
	if (shape.shapeDefinitionError || !shape.useOnlyRisingEdges || shape.needSecondTriggerInput ||
		shape.getExpectedEventCount(TriggerWheel::T_SECONDARY) != 0 ||
		shape.getWheelOperationMode() != FOUR_STROKE_CRANK_SENSOR || length < 16 || length > PWM_PHASE_MAX_COUNT ||
		(length & 3)) {
		return;
	}
	float normalSpan = 720.0f;
	for (size_t i = 0; i < length; i += 2) {
		const float angle = details.eventAngles[i];
		const float next = i + 2 < length ? details.eventAngles[i + 2] : 720.0f;
		if (!std::isfinite(angle) || angle != details.eventAngles[i + 1] || (i == 0 && angle != 0) || next <= angle ||
			next > 720.0f) {
			return;
		}
		normalSpan = std::min(normalSpan, next - angle);
	}
	unsigned gaps = 0;
	float gapSpan = 0;
	for (size_t i = 0; i < length; i += 2) {
		const float next = i + 2 < length ? details.eventAngles[i + 2] : 720.0f;
		const float span = next - details.eventAngles[i];
		if (std::abs(span - normalSpan) > 0.01f) {
			if (span < 1.5f * normalSpan || (gaps && std::abs(span - gapSpan) > 0.01f)) {
				return;
			}
			gapSpan = span;
			gaps++;
		}
		if (i < length / 2 && std::abs(details.eventAngles[i + length / 2] - details.eventAngles[i] - 360.0f) > 0.01f) {
			return;
		}
	}
	if (gaps == 2) {
		m_slots = length;
	}
}

void IgnitionCycleProfile::update(const uint32_t* timestamps, uint16_t index, const EnginePhaseInfo& phase) {
	m_valid = false;
	if (!m_slots || index >= m_slots || (index & 1)) {
		reset();
		return;
	}
	// Full-width timestamp comparison rejects stale history even across more than one
	// uint32 wrap; all arithmetic used by the prediction itself stays in uint32/float32.
	if (m_count && (index != m_expectedIndex || phase.timestamp <= m_phase.timestamp ||
					phase.timestamp - m_phase.timestamp > US2NT(100000))) {
		reset();
	}
	const uint32_t now = phase.timestamp;
	const auto next = index + 2 == m_slots ? 0 : index + 2;
	if (phase.currentTrgPhase.angle != m_details->eventAngles[index] ||
		phase.nextTrgPhase.angle != m_details->eventAngles[next]) {
		reset();
		return;
	}
	const auto past3 = index >= 6 ? index - 6 : index + m_slots - 6;
	const uint32_t oldCurrent = timestamps[index];
	const uint32_t oldNext = timestamps[next];
	// At j=N+3, the ring and FIFO first contain all N+4 trusted timestamps.
	if (m_count >= toothCount() + 3) {
		const uint32_t recentNow = now - timestamps[past3];
		const uint32_t recentOld = oldCurrent - m_old[m_head];
		const uint32_t durationOld = oldNext - oldCurrent;
		const uint32_t cycleTime = now - oldCurrent;
		constexpr uint32_t maxCycle = 4800000U * US_TO_NT_MULTIPLIER; // 720 degrees at the stop threshold (25 RPM)
		constexpr uint32_t maxGap = 100000U * US_TO_NT_MULTIPLIER;	  // matches watchdog floor for these dense wheels
		float span = phase.nextTrgPhase - phase.currentTrgPhase;
		if (span < 0) {
			span += 720.0f;
		}
		if (recentNow && recentOld && durationOld && cycleTime && cycleTime <= maxCycle && recentNow <= 3 * maxGap &&
			recentOld <= 3 * maxGap && durationOld <= maxGap && recentNow < cycleTime && recentOld < cycleTime &&
			span > 0 && span < 360.0f) {
			const float nowWindow = static_cast<float>(recentNow);
			const float oldWindow = static_cast<float>(recentOld);
			// Experimental rejection, rather than clamping an untrusted profile into a timer.
			if (nowWindow >= 0.5f * oldWindow && nowWindow <= 2.0f * oldWindow) {
				// One float division per tooth; each ignition event only needs a multiply.
				m_ticksPerDegree = (static_cast<float>(durationOld) * nowWindow) / (oldWindow * span);
				m_span = span;
				m_valid = std::isfinite(m_ticksPerDegree) && m_ticksPerDegree > 0 && m_ticksPerDegree * span <= maxGap;
			}
		}
	}
	m_old[m_head] = oldCurrent;
	if (++m_head == 3) {
		m_head = 0;
	}
	m_expectedIndex = next;
	if (m_count < toothCount() + 3) {
		m_count++;
	}
	m_phase = phase;
}

expected<float> IgnitionCycleProfile::getDelayNt(const EnginePhaseInfo& phase, float angleOffset) const {
	if (!m_valid || phase.timestamp != m_phase.timestamp || !(phase.currentTrgPhase == m_phase.currentTrgPhase) ||
		!(phase.nextTrgPhase == m_phase.nextTrgPhase) || !(phase.currentEngPhase == m_phase.currentEngPhase) ||
		!(phase.nextEngPhase == m_phase.nextEngPhase) || !std::isfinite(angleOffset) || angleOffset < 0 ||
		angleOffset >= m_span) {
		return unexpected;
	}
	return m_ticksPerDegree * angleOffset;
}
