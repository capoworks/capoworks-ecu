#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

enum class IgnitionRetargetStatus : uint8_t {
	Unchanged,
	Full,
	Limited,
	Rejected,
	Recovered
};

struct IgnitionTargetDecision {
	float distance;
	IgnitionRetargetStatus status;
};

// All distances refer to the SAME TDC occurrence, without wrapping a passed
// request into the next cycle. Bounds are derived from the causal time model.
inline IgnitionTargetDecision
selectIgnitionTarget(float previous, float requested, float earliest, float latest, bool alreadyArmed = false) {
	if (!std::isfinite(previous) || !std::isfinite(requested) || !std::isfinite(earliest) || !std::isfinite(latest) ||
		earliest > latest) {
		return {previous, IgnitionRetargetStatus::Rejected};
	}
	const float applied = std::clamp(requested, earliest, latest);
	// An already registered timer needs no new service reserve. A new target
	// must lie on the segment from that timer to the requested target.
	if (alreadyArmed && (applied < std::min(previous, requested) || applied > std::max(previous, requested))) {
		return {previous, IgnitionRetargetStatus::Rejected};
	}
	// Speed/arrival changes may invalidate even the previous plan. Explicitly
	// report safety recovery, not acceptance in the driver's requested direction.
	if (!alreadyArmed && (previous < earliest || previous > latest)) {
		return {applied, IgnitionRetargetStatus::Recovered};
	}
	if (applied == previous) {
		return {previous, requested == previous ? IgnitionRetargetStatus::Unchanged : IgnitionRetargetStatus::Rejected};
	}
	return {applied, applied == requested ? IgnitionRetargetStatus::Full : IgnitionRetargetStatus::Limited};
}
