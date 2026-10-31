/**
 * @file ve_analyze_fuel_events.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

#include "efitime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ve_analyze_fuel_events {
inline constexpr uint16_t PolicyVersion = 1;
inline constexpr size_t MaximumCylinders = static_cast<size_t>(MAX_CYLINDER_COUNT);
inline constexpr size_t MaximumBanks = static_cast<size_t>(STFT_BANK_COUNT);

static_assert(MaximumCylinders <= 16, "affected cylinder mask is 16 bits");
static_assert(MaximumBanks <= 8, "affected bank mask is 8 bits");

inline constexpr float DefaultAeLimitPercent = 1.0f;
inline constexpr float DefaultFilmLimitPercent = 2.0f;
inline constexpr float DefaultCombinedLimitPercent = 2.0f;
inline constexpr float DefaultNormalRecoverySeconds = 1.5f;
inline constexpr float DefaultCutRecoverySeconds = 2.0f;

// A cylinder normally receives an event every 75 ms at 1600 RPM. This finite
// tolerance also supports slower observations without requiring 4 ms updates.
inline constexpr float DefaultEventFreshnessSeconds = 0.25f;
inline constexpr float DefaultMassAbsoluteToleranceGrams = 0.00001f;
inline constexpr float DefaultMassRelativeTolerance = 0.00001f;
inline constexpr float MaximumPolicySeconds = 86400.0f;
inline constexpr float MaximumPolicyPercent = 10000.0f;

inline constexpr float gramsToMilligrams(float grams) {
	return grams * 1000.0f;
}

inline constexpr float milligramsToGrams(float milligrams) {
	return milligrams * 0.001f;
}

enum class FuelEventReason : uint32_t {
	None = 0,
	InvalidConfiguration = 1u << 0,
	InvalidOwnerCylinder = 1u << 1,
	InvalidCylinderMask = 1u << 2,
	InvalidBankMask = 1u << 3,
	InvalidGeneration = 1u << 4,
	RegressedGeneration = 1u << 5,
	DuplicateTimestamp = 1u << 6,
	RegressedTimestamp = 1u << 7,
	NonFiniteMass = 1u << 8,
	NegativeMass = 1u << 9,
	MassIdentityMismatch = 1u << 10,
	StageMassMismatch = 1u << 11,
	InvalidDenominator = 1u << 12,
	ExcessAeRatio = 1u << 13,
	ExcessFilmRatio = 1u << 14,
	ExcessCombinedRatio = 1u << 15,
	NormalRecovery = 1u << 16,
	WallModelClampRecovery = 1u << 17,
	ZeroCommandedRecovery = 1u << 18,
	SchedulingRejectedRecovery = 1u << 19,
	ResetRecovery = 1u << 20,
	NoFreshEvent = 1u << 21,
	StaleEvent = 1u << 22,
	FutureTimestamp = 1u << 23
};

inline constexpr size_t FuelEventReasonCount = 24;

constexpr uint32_t operator|(FuelEventReason lhs, FuelEventReason rhs) {
	return static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs);
}

constexpr size_t reasonIndex(FuelEventReason reason) {
	const uint32_t value = static_cast<uint32_t>(reason);
	if (value == 0 || (value & (value - 1)) != 0) {
		return FuelEventReasonCount;
	}

	size_t index = 0;
	while ((value & (1u << index)) == 0) {
		index++;
	}
	return index;
}
} // namespace ve_analyze_fuel_events

/**
 * Experimental initial guard. These values are component policy, not client
 * filter values.
 */
struct VeAnalyzeFuelEventPolicy {
	float AeLimitPercent = ve_analyze_fuel_events::DefaultAeLimitPercent;
	float FilmLimitPercent = ve_analyze_fuel_events::DefaultFilmLimitPercent;
	float CombinedLimitPercent = ve_analyze_fuel_events::DefaultCombinedLimitPercent;
	float NormalRecoverySeconds = ve_analyze_fuel_events::DefaultNormalRecoverySeconds;
	float CutRecoverySeconds = ve_analyze_fuel_events::DefaultCutRecoverySeconds;
	float EventFreshnessSeconds = ve_analyze_fuel_events::DefaultEventFreshnessSeconds;
	float MassAbsoluteToleranceGrams = ve_analyze_fuel_events::DefaultMassAbsoluteToleranceGrams;
	float MassRelativeTolerance = ve_analyze_fuel_events::DefaultMassRelativeTolerance;
};

/**
 * One calculated injection attempt supplied by the caller.
 *
 * All masses are already scaled for the active injection mode, split, and
 * staging. This collector never applies an injection-mode multiplier again.
 * The caller also supplies the applied correction masses and final command.
 */
struct VeAnalyzeFuelEventInput {
	uint8_t OwnerCylinder = 0;
	uint16_t AffectedCylinderMask = 0;
	uint8_t AffectedBankMask = 0;
	// Zero is the reserved "unknown generation" value.
	uint32_t PublicationGeneration = 0;
	uint64_t Serial = 0;
	efitick_t Timestamp;

	float BasePreAeMassGrams = 0;
	float AppliedAeMassGrams = 0;
	float FilmCorrectionMassGrams = 0;
	float FinalCommandedMassGrams = 0;

	bool HasStage1Mass = false;
	float Stage1MassGrams = 0;
	bool HasStage2Mass = false;
	float Stage2MassGrams = 0;

	bool WallModelZeroClamp = false;
	bool SchedulerAccepted = false;
};

/**
 * Snapshot of one event record. Masses are explicitly exported in milligrams.
 * "Calculated" and "attempted" values are not claims of physically delivered
 * fuel; scheduled command is only the command presented to scheduling.
 */
struct VeAnalyzeFuelEventRecord {
	bool HasEvent = false;
	bool Fresh = false;
	bool SchedulerAccepted = false;
	bool RatiosValid = false;
	bool Stage1Supplied = false;
	bool Stage2Supplied = false;
	bool WallModelZeroClamp = false;

	uint8_t OwnerCylinder = 0;
	uint16_t AffectedCylinderMask = 0;
	uint8_t AffectedBankMask = 0;
	uint32_t PublicationGeneration = 0;
	uint64_t Serial = 0;
	uint32_t AgeMilliseconds = std::numeric_limits<uint32_t>::max();

	float CalculatedBaseMassMilligrams = 0;
	float CalculatedAppliedAeMassMilligrams = 0;
	float CalculatedFilmCorrectionMassMilligrams = 0;
	float AttemptedFinalCommandedMassMilligrams = 0;
	// This is zero when the scheduler rejected the attempt.
	float ScheduledCommandMassMilligrams = 0;
	float Stage1MassMilligrams = 0;
	float Stage2MassMilligrams = 0;

	float AePercentOfBase = 0;
	float FilmPercentOfBase = 0;
	float CombinedPercentOfBase = 0;
};

struct VeAnalyzeFuelEventBankSnapshot {
	bool Observed = false;
	bool EvidenceFresh = false;
	uint16_t ObservedCylinderMask = 0;
	uint16_t ActiveNormalCylinderMask = 0;
	uint16_t ActiveCutCylinderMask = 0;

	uint32_t Reasons = 0;
	uint32_t AttemptedEventCount = 0;
	uint32_t ScheduledEventCount = 0;
	uint32_t SchedulerRejectedEventCount = 0;

	float PeakAePercentOfBase = 0;
	float PeakFilmPercentOfBase = 0;
	float PeakCombinedPercentOfBase = 0;

	VeAnalyzeFuelEventRecord LatestEvent;

	bool hasReason(ve_analyze_fuel_events::FuelEventReason reason) const {
		return (Reasons & static_cast<uint32_t>(reason)) != 0;
	}
};

struct VeAnalyzeFuelEventSnapshot {
	// Ready describes collector evidence only. It deliberately excludes model
	// qualification and lambda validity.
	bool Ready = false;
	bool HasFreshEvent = false;
	uint32_t Reasons = 0;

	uint32_t AttemptedEventCount = 0;
	// Counts scheduler acceptance of coherent attempts. It is not a delivered-
	// fuel count and explicitly excludes scheduler-rejected attempts.
	uint32_t AcceptedEventCount = 0;
	uint32_t RejectedEventCount = 0;
	uint32_t GuardRejectedEventCount = 0;
	uint32_t InvalidEvidenceEventCount = 0;

	uint16_t ObservedCylinderMask = 0;
	uint8_t ObservedBankMask = 0;
	uint32_t PublicationGeneration = 0;
	uint32_t LastEventAgeMilliseconds = std::numeric_limits<uint32_t>::max();

	VeAnalyzeFuelEventRecord LastEvent;
	std::array<VeAnalyzeFuelEventBankSnapshot, ve_analyze_fuel_events::MaximumBanks> Banks;
	std::array<uint32_t, ve_analyze_fuel_events::FuelEventReasonCount> RejectedEventCountsByReason = {};

	bool hasReason(ve_analyze_fuel_events::FuelEventReason reason) const {
		return (Reasons & static_cast<uint32_t>(reason)) != 0;
	}

	uint32_t rejectedEventCount(ve_analyze_fuel_events::FuelEventReason reason) const {
		const size_t index = ve_analyze_fuel_events::reasonIndex(reason);
		return index < RejectedEventCountsByReason.size() ? RejectedEventCountsByReason[index] : 0;
	}
};

/**
 * Bounded, engine-independent per-event fuel observability collector.
 *
 * This class owns no global state and performs no allocation. It preserves the
 * latest coherent event per owner cylinder and per affected bank, peak absolute
 * ratios, and recovery timers across snapshot polling. A condition on one
 * cylinder can only be ended by a later observation affecting that cylinder.
 *
 * The caller must provide already mode-scaled and staged masses. Concurrent
 * callers must synchronize access to the owning instance.
 */
class VeAnalyzeFuelEvents final {
public:
	VeAnalyzeFuelEvents();
	explicit VeAnalyzeFuelEvents(const VeAnalyzeFuelEventPolicy& policy);

	bool isValidConfiguration() const {
		return m_configValid;
	}

	const VeAnalyzeFuelEventPolicy& policy() const {
		return m_policy;
	}

	/**
	 * Clears all evidence and counters, then requires a fresh event and starts
	 * normal reset recovery.
	 */
	void reset(efitick_t nowNt);
	void syncReset(efitick_t nowNt) {
		reset(nowNt);
	}

	/**
	 * Records one event attempt.
	 *
	 * Returns true when coherent evidence was recorded, regardless of scheduler
	 * acceptance. Use the snapshot to distinguish scheduled from rejected.
	 */
	bool observeEvent(const VeAnalyzeFuelEventInput& input);
	bool observe(const VeAnalyzeFuelEventInput& input) {
		return observeEvent(input);
	}

	// Read-only and safe to poll slowly. Polling does not clear peaks or timers.
	VeAnalyzeFuelEventSnapshot snapshot(efitick_t nowNt) const;
	VeAnalyzeFuelEventRecord cylinderSnapshot(size_t ownerCylinder, efitick_t nowNt) const;

private:
	struct StoredEvent {
		bool HasEvent = false;
		bool SchedulerAccepted = false;
		bool Stage1Supplied = false;
		bool Stage2Supplied = false;
		bool WallModelZeroClamp = false;
		bool RatiosValid = false;

		uint8_t OwnerCylinder = 0;
		uint16_t AffectedCylinderMask = 0;
		uint8_t AffectedBankMask = 0;
		uint32_t PublicationGeneration = 0;
		uint64_t Serial = 0;
		efitick_t Timestamp;

		float BasePreAeMassGrams = 0;
		float AppliedAeMassGrams = 0;
		float FilmCorrectionMassGrams = 0;
		float FinalCommandedMassGrams = 0;
		float Stage1MassGrams = 0;
		float Stage2MassGrams = 0;
	};

	struct BankState {
		StoredEvent Latest;
		uint16_t ActiveNormalCylinders = 0;
		uint16_t ActiveCutCylinders = 0;
		uint16_t ObservedCylinderMask = 0;

		uint32_t NormalReasons = 0;
		uint32_t CutReasons = 0;
		uint32_t FaultReasons = 0;

		int64_t NormalUntilNt = std::numeric_limits<int64_t>::min();
		int64_t CutUntilNt = std::numeric_limits<int64_t>::min();
		int64_t FaultUntilNt = std::numeric_limits<int64_t>::min();
		int64_t PeakUntilNt = std::numeric_limits<int64_t>::min();

		float PeakAePercent = 0;
		float PeakFilmPercent = 0;
		float PeakCombinedPercent = 0;

		uint32_t AttemptedEvents = 0;
		uint32_t ScheduledEvents = 0;
		uint32_t SchedulerRejectedEvents = 0;
	};

	void setPolicy(const VeAnalyzeFuelEventPolicy& policy);
	void clearHistory();
	void rejectEvent(
			const VeAnalyzeFuelEventInput& input, uint32_t reasons, uint16_t validCylinderMask, uint8_t validBankMask);
	void recordRejectedReasons(uint32_t reasons);
	void markBankFault(uint8_t bankMask, efitick_t nowNt, uint32_t reasons, int64_t durationNt);
	void markGlobalFault(efitick_t nowNt, uint32_t reasons, int64_t durationNt);
	void updateBankConditions(
			BankState& bank,
			uint16_t cylinderMask,
			efitick_t nowNt,
			bool normalActive,
			uint32_t normalReasons,
			bool cutActive,
			uint32_t cutReasons);
	void updatePeak(BankState& bank, efitick_t nowNt, const StoredEvent& event);
	bool timestampIsFresh(efitick_t nowNt, efitick_t timestamp) const;
	VeAnalyzeFuelEventRecord makeRecord(const StoredEvent& event, efitick_t nowNt) const;
	VeAnalyzeFuelEventBankSnapshot makeBankSnapshot(size_t bankIndex, efitick_t nowNt) const;

	VeAnalyzeFuelEventPolicy m_policy;
	bool m_configValid = false;
	int64_t m_normalRecoveryNt = 0;
	int64_t m_cutRecoveryNt = 0;
	int64_t m_eventFreshnessNt = 0;

	bool m_hasGeneration = false;
	uint32_t m_generation = 0;

	std::array<StoredEvent, ve_analyze_fuel_events::MaximumCylinders> m_cylinders;
	std::array<BankState, ve_analyze_fuel_events::MaximumBanks> m_banks;

	int64_t m_resetUntilNt = std::numeric_limits<int64_t>::min();
	uint32_t m_resetReasons = 0;
	int64_t m_globalFaultUntilNt = std::numeric_limits<int64_t>::min();
	uint32_t m_globalFaultReasons = 0;

	uint32_t m_attemptedEvents = 0;
	uint32_t m_acceptedEvents = 0;
	uint32_t m_rejectedEvents = 0;
	uint32_t m_guardRejectedEvents = 0;
	uint32_t m_invalidEvidenceEvents = 0;
	std::array<uint32_t, ve_analyze_fuel_events::FuelEventReasonCount> m_rejectedByReason = {};
};

static_assert(sizeof(VeAnalyzeFuelEvents) < 1536, "fuel-event collector exceeds its RAM budget");
