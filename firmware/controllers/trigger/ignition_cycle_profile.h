#pragma once

#include "trigger_structure.h"

// Reuses InstantRpmCalculator's phase-indexed timestamps. No independently advancing ring.
class IgnitionCycleProfile {
public:
	void configure(const TriggerWaveform& shape, const TriggerFormDetails& details);
	void reset();
	// Must run BEFORE the shared timestamp at index is overwritten.
	void update(const uint32_t* timestamps, uint16_t index, const EnginePhaseInfo& phase);
	expected<float> getDelayNt(const EnginePhaseInfo& phase, float angleOffset) const;
	uint16_t toothCount() const {
		return m_slots / 2;
	}

private:
	const TriggerFormDetails* m_details = nullptr;
	uint32_t m_old[3] = {};
	EnginePhaseInfo m_phase{};
	float m_ticksPerDegree = 0;
	float m_span = 0;
	uint16_t m_slots = 0;
	uint16_t m_expectedIndex = 0;
	uint16_t m_count = 0;
	uint8_t m_head = 0;
	bool m_valid = false;
};
