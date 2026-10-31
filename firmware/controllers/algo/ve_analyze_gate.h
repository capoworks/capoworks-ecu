/**
 * @file ve_analyze_gate.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "efitime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ve_analyze_gate {
inline constexpr uint16_t PolicyVersion = 1;

inline constexpr float SamplingFrequencyHz = 250.0f;
inline constexpr float WindowSeconds = 0.20f;
inline constexpr float DefaultRateThresholdPercentPerSecond = 5.0f;
inline constexpr float DefaultQuickMovementThresholdPercent = 1.0f;
inline constexpr float DefaultSampleToleranceSeconds = 0.05f;
inline constexpr float DefaultQuietHoldSeconds = 1.5f;
inline constexpr float DefaultCutRecoverySeconds = 2.0f;
inline constexpr float NoLambdaDelay = -1.0f;
inline constexpr float MaximumPolicySeconds = 86400.0f;
inline constexpr size_t SampleCapacity = 64;

static_assert(SampleCapacity >= static_cast<size_t>(SamplingFrequencyHz * WindowSeconds) + 1);
} // namespace ve_analyze_gate

struct VeAnalyzeGateConfig {
	// Passing these values explicitly keeps future TunerStudio policy distinct
	// from the component defaults instead of silently changing the sample gate.
	float RateThresholdPercentPerSecond = ve_analyze_gate::DefaultRateThresholdPercentPerSecond;
	float QuickMovementThresholdPercent = ve_analyze_gate::DefaultQuickMovementThresholdPercent;
	float SampleToleranceSeconds = ve_analyze_gate::DefaultSampleToleranceSeconds;
	float QuietHoldSeconds = ve_analyze_gate::DefaultQuietHoldSeconds;
	float CutRecoverySeconds = ve_analyze_gate::DefaultCutRecoverySeconds;
	// Negative means that the integration did not supply a lambda delay.
	float MaxLambdaDelaySeconds = ve_analyze_gate::NoLambdaDelay;
};

enum class VeAnalyzeTransient : uint8_t {
	// Normal fuel correction or another ordinary calibration-visible change.
	FuelCorrection,
	// Strong fuel cut, including actual zero-delivered-mass operation.
	FuelCut,
	// A calibration mutation that invalidates samples already in flight.
	Calibration,
	Count
};

enum class VeAnalyzeGateReason : uint32_t {
	None = 0,
	InvalidConfiguration = 1u << 0,
	InvalidTps = 1u << 1,
	StaleTps = 1u << 2,
	InsufficientHistory = 1u << 3,
	SampleGap = 1u << 4,
	DuplicateTimestamp = 1u << 5,
	RegressedTimestamp = 1u << 6,
	TpsMovementRecovery = 1u << 7,
	FuelCorrectionRecovery = 1u << 8,
	FuelCutRecovery = 1u << 9,
	CalibrationRecovery = 1u << 10,
	StartupRecovery = 1u << 11,
	ResetRecovery = 1u << 12
};

constexpr uint32_t operator|(VeAnalyzeGateReason lhs, VeAnalyzeGateReason rhs) {
	return static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs);
}

struct VeAnalyzeGateSnapshot {
	// TpsValid describes the current TPS reading and its sample age.
	bool TpsValid = false;
	// RateValid additionally requires a complete, timestamp-valid window.
	bool RateValid = false;
	// Ready means the sample gate is currently open. It includes all holds.
	bool Ready = false;

	float TpsPercent = 0;
	float AbsTpsRatePercentPerSecond = 0;
	float WindowSeconds = 0;

	bool HasObservedMovement = false;
	bool HasObservedTransient = false;
	uint32_t MillisecondsSinceMovement = std::numeric_limits<uint32_t>::max();
	uint32_t MillisecondsSinceCorrection = std::numeric_limits<uint32_t>::max();
	uint32_t HoldRemainingMilliseconds = 0;
	uint32_t Reasons = 0;

	bool hasReason(VeAnalyzeGateReason reason) const {
		return (Reasons & static_cast<uint32_t>(reason)) != 0;
	}
};

/**
 * Bounded, engine-independent VE Analyze sample gate.
 *
 * TPS updates must use the fast-loop timestamp. Set sensorValid from
 * Sensor::get() preserving invalidity. Set fresh only when that call supplied
 * a newly published sample; cached valid samples remain usable until the
 * configured sample tolerance expires.
 *
 * External transients are edge-observed booleans. Call with active=true for
 * every update while a condition persists and active=false when it ends.
 * Recurring active observations renew the hold. An inactive observation is
 * only significant after an active one.
 *
 * This class owns no global state and performs no allocation. Concurrent
 * callers must provide synchronization for the owning instance.
 */
class VeAnalyzeGate final {
public:
	VeAnalyzeGate();
	explicit VeAnalyzeGate(const VeAnalyzeGateConfig& config);

	bool configure(const VeAnalyzeGateConfig& config);
	bool isValidConfiguration() const {
		return m_configValid;
	}

	const VeAnalyzeGateConfig& config() const {
		return m_config;
	}

	void reset(efitick_t nowNt);

	// Returns true only when a new, finite, non-duplicate sample was accepted.
	bool updateTps(efitick_t nowNt, float tpsPercent, bool sensorValid, bool fresh);

	void observeTransient(efitick_t nowNt, VeAnalyzeTransient transient, bool active);

	// Read-only and safe to poll slowly. Polling does not clear event history.
	VeAnalyzeGateSnapshot snapshot(efitick_t nowNt) const;

private:
	struct Sample {
		efitick_t Timestamp;
		float TpsPercent = 0;
	};

	bool updateRate();
	void appendSample(efitick_t nowNt, float tpsPercent);
	void clearSamples();
	void startHold(efitick_t nowNt, int64_t durationNt, VeAnalyzeGateReason reason);
	void reseedForFault(efitick_t nowNt, VeAnalyzeGateReason reason);
	int64_t normalHoldNt() const;
	int64_t cutHoldNt() const;

	static size_t transientIndex(VeAnalyzeTransient transient);
	uint8_t transientState(VeAnalyzeTransient transient) const;
	void setTransientState(VeAnalyzeTransient transient, uint8_t state);

	VeAnalyzeGateConfig m_config;
	bool m_configValid = false;
	int64_t m_normalHoldNt = 0;
	int64_t m_cutHoldNt = 0;
	int64_t m_sampleToleranceNt = 0;

	std::array<Sample, ve_analyze_gate::SampleCapacity> m_samples;
	size_t m_head = 0;
	size_t m_count = 0;

	bool m_hasTpsUpdate = false;
	efitick_t m_lastTpsUpdate;
	bool m_tpsValid = false;
	bool m_rateValid = false;
	float m_absRatePercentPerSecond = 0;
	float m_windowSeconds = 0;
	bool m_needStartupHold = true;

	uint8_t m_timestampFault = 0;
	uint8_t m_historyFault = 0;

	int64_t m_holdUntilNt = std::numeric_limits<int64_t>::min();
	uint32_t m_holdReasons = 0;

	bool m_hasMovement = false;
	efitick_t m_lastMovement;
	bool m_hasCorrection = false;
	efitick_t m_lastCorrection;

	bool m_hasExternalUpdate = false;
	efitick_t m_lastExternalUpdate;
	// Two bits per transient: 0 = unknown, 1 = inactive, 2 = active.
	uint8_t m_transientStates = 0;
};
