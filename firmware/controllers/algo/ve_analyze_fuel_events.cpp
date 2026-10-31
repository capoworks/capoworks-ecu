/**
 * @file ve_analyze_fuel_events.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "ve_analyze_fuel_events.h"

#include <algorithm>
#include <cmath>

namespace {
using namespace ve_analyze_fuel_events;

constexpr uint32_t CylinderMaskAll = (1u << MaximumCylinders) - 1u;
constexpr uint32_t BankMaskAll = (1u << MaximumBanks) - 1u;

uint32_t saturatingIncrement(uint32_t value) {
	return value == std::numeric_limits<uint32_t>::max() ? value : value + 1;
}

int64_t secondsToNt(float seconds) {
	return static_cast<int64_t>(std::ceil(static_cast<double>(seconds) * US_PER_SECOND_F * US_TO_NT_MULTIPLIER));
}

int64_t saturatingAddNt(efitick_t nowNt, int64_t durationNt) {
	constexpr int64_t maximum = std::numeric_limits<int64_t>::max();
	if (durationNt > 0 && nowNt.count > maximum - durationNt) {
		return maximum;
	}
	return nowNt.count + durationNt;
}

uint32_t elapsedMilliseconds(efitick_t nowNt, efitick_t sinceNt) {
	if (nowNt.count <= sinceNt.count) {
		return 0;
	}

	const uint64_t elapsedNt = static_cast<uint64_t>(nowNt.count) - static_cast<uint64_t>(sinceNt.count);
	const uint64_t elapsedUs = elapsedNt / static_cast<uint64_t>(US_TO_NT_MULTIPLIER);
	const uint64_t milliseconds = elapsedUs / 1000;
	const uint64_t maximum = std::numeric_limits<uint32_t>::max();
	return milliseconds > maximum ? static_cast<uint32_t>(maximum) : static_cast<uint32_t>(milliseconds);
}

bool validPositiveSeconds(float value) {
	return std::isfinite(value) && value > 0 && value <= MaximumPolicySeconds;
}

bool validThreshold(float value) {
	return std::isfinite(value) && value >= 0 && value <= MaximumPolicyPercent;
}

bool validTolerance(float value) {
	return std::isfinite(value) && value >= 0 && value <= 1;
}

bool closeEnough(float actual, float expected, const VeAnalyzeFuelEventPolicy& policy) {
	const float scale = std::max({std::fabs(actual), std::fabs(expected), 1.0f});
	const float tolerance = policy.MassAbsoluteToleranceGrams + policy.MassRelativeTolerance * scale;
	return std::fabs(actual - expected) <= tolerance;
}

bool isBitSet(uint8_t mask, size_t index) {
	return (mask & (1u << index)) != 0;
}

bool isBitSet(uint16_t mask, size_t index) {
	return (mask & (1u << index)) != 0;
}

float ratioPercent(float mass, float baseMass) {
	return 100.0f * std::fabs(mass) / baseMass;
}

uint16_t cylinderBit(uint8_t cylinder) {
	return static_cast<uint16_t>(1u << cylinder);
}

bool isGenerationNewer(uint32_t candidate, uint32_t current) {
	const uint32_t delta = candidate - current;
	return delta != 0 && delta < 0x80000000u;
}

bool hasAnyReason(uint32_t reasons) {
	return reasons != 0;
}
} // namespace

VeAnalyzeFuelEvents::VeAnalyzeFuelEvents() {
	setPolicy(VeAnalyzeFuelEventPolicy{});
}

VeAnalyzeFuelEvents::VeAnalyzeFuelEvents(const VeAnalyzeFuelEventPolicy& policy) {
	setPolicy(policy);
}

void VeAnalyzeFuelEvents::setPolicy(const VeAnalyzeFuelEventPolicy& policy) {
	m_policy = policy;
	const bool valid =
			validThreshold(policy.AeLimitPercent) && validThreshold(policy.FilmLimitPercent) &&
			validThreshold(policy.CombinedLimitPercent) && validPositiveSeconds(policy.NormalRecoverySeconds) &&
			validPositiveSeconds(policy.CutRecoverySeconds) && validPositiveSeconds(policy.EventFreshnessSeconds) &&
			validTolerance(policy.MassAbsoluteToleranceGrams) && validTolerance(policy.MassRelativeTolerance);

	m_configValid = valid;
	if (!valid) {
		m_normalRecoveryNt = 0;
		m_cutRecoveryNt = 0;
		m_eventFreshnessNt = 0;
		clearHistory();
		return;
	}

	m_normalRecoveryNt = secondsToNt(policy.NormalRecoverySeconds);
	m_cutRecoveryNt = secondsToNt(policy.CutRecoverySeconds);
	m_eventFreshnessNt = secondsToNt(policy.EventFreshnessSeconds);
	clearHistory();
}

void VeAnalyzeFuelEvents::clearHistory() {
	m_cylinders.fill(StoredEvent{});
	m_banks.fill(BankState{});
	m_hasGeneration = false;
	m_generation = 0;
	m_expectedCylinderMask = 0;
	m_expectedBankMask = 0;
	m_rejectionSinceLatestEvent = false;
	m_cylinderRejectionAfterEvent.fill(false);
	m_resetUntilNt = std::numeric_limits<int64_t>::min();
	m_resetReasons = 0;
	m_globalFaultUntilNt = std::numeric_limits<int64_t>::min();
	m_globalFaultReasons = 0;
	m_attemptedEvents = 0;
	m_acceptedEvents = 0;
	m_rejectedEvents = 0;
	m_guardRejectedEvents = 0;
	m_invalidEvidenceEvents = 0;
	m_rejectedByReason.fill(0);
}

void VeAnalyzeFuelEvents::reset(efitick_t nowNt) {
	clearHistory();
	m_resetReasons = static_cast<uint32_t>(FuelEventReason::ResetRecovery);
	if (m_configValid) {
		m_resetUntilNt = saturatingAddNt(nowNt, m_normalRecoveryNt);
	}
}

void VeAnalyzeFuelEvents::recordRejectedReasons(uint32_t reasons) {
	for (size_t index = 0; index < FuelEventReasonCount; index++) {
		if ((reasons & (1u << index)) != 0) {
			m_rejectedByReason[index] = saturatingIncrement(m_rejectedByReason[index]);
		}
	}
}

void VeAnalyzeFuelEvents::markGlobalFault(efitick_t nowNt, uint32_t reasons, int64_t durationNt) {
	const int64_t candidateUntil = saturatingAddNt(nowNt, durationNt);
	if (m_globalFaultUntilNt <= nowNt.count) {
		m_globalFaultReasons = reasons;
		m_globalFaultUntilNt = candidateUntil;
		return;
	}

	m_globalFaultReasons |= reasons;
	m_globalFaultUntilNt = std::max(m_globalFaultUntilNt, candidateUntil);
}

void VeAnalyzeFuelEvents::markBankFault(uint8_t bankMask, efitick_t nowNt, uint32_t reasons, int64_t durationNt) {
	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		if (!isBitSet(bankMask, bankIndex)) {
			continue;
		}

		BankState& bank = m_banks[bankIndex];
		const int64_t candidateUntil = saturatingAddNt(nowNt, durationNt);
		if (bank.FaultUntilNt <= nowNt.count) {
			bank.FaultReasons = reasons;
			bank.FaultUntilNt = candidateUntil;
		} else {
			bank.FaultReasons |= reasons;
			bank.FaultUntilNt = std::max(bank.FaultUntilNt, candidateUntil);
		}
	}
}

void VeAnalyzeFuelEvents::rejectEvent(
		const VeAnalyzeFuelEventInput& input, uint32_t reasons, uint16_t validCylinderMask, uint8_t validBankMask) {
	m_invalidEvidenceEvents = saturatingIncrement(m_invalidEvidenceEvents);
	recordRejectedReasons(reasons);
	m_rejectionSinceLatestEvent = true;

	const bool zeroCommanded = std::isfinite(input.FinalCommandedMassGrams) && input.FinalCommandedMassGrams == 0;
	const bool cutLike =
			input.WallModelZeroClamp || zeroCommanded || !input.SchedulerAccepted || input.IntegrationUnsupported;
	const int64_t duration = cutLike ? m_cutRecoveryNt : m_normalRecoveryNt;

	uint32_t faultReasons = reasons;
	if (cutLike) {
		if (input.WallModelZeroClamp) {
			faultReasons |= static_cast<uint32_t>(FuelEventReason::WallModelClampRecovery);
		}
		if (zeroCommanded) {
			faultReasons |= static_cast<uint32_t>(FuelEventReason::ZeroCommandedRecovery);
		}
		if (!input.SchedulerAccepted) {
			faultReasons |= static_cast<uint32_t>(FuelEventReason::SchedulingRejectedRecovery);
		}
		if (input.IntegrationUnsupported) {
			faultReasons |= static_cast<uint32_t>(FuelEventReason::IntegrationUnsupported);
		}
	} else {
		faultReasons |= static_cast<uint32_t>(FuelEventReason::NormalRecovery);
	}

	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		if (validBankMask == 0 || isBitSet(validBankMask, bankIndex)) {
			m_banks[bankIndex].ObservedCylinderMask |= validCylinderMask;
			m_banks[bankIndex].LatestRejectedAfterEvent = true;
		}
	}
	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		if (validCylinderMask == 0 || isBitSet(validCylinderMask, cylinderIndex)) {
			m_cylinderRejectionAfterEvent[cylinderIndex] = true;
		}
	}

	if (validBankMask != 0) {
		markBankFault(validBankMask, input.Timestamp, faultReasons, duration);
	} else {
		markGlobalFault(input.Timestamp, faultReasons, duration);
	}
}

bool VeAnalyzeFuelEvents::timestampIsFresh(efitick_t nowNt, efitick_t timestamp) const {
	if (nowNt.count < timestamp.count) {
		return false;
	}
	return nowNt.count - timestamp.count <= m_eventFreshnessNt;
}

VeAnalyzeFuelEventRecord VeAnalyzeFuelEvents::makeRecord(const StoredEvent& event, efitick_t nowNt) const {
	VeAnalyzeFuelEventRecord result;
	result.HasEvent = event.HasEvent;
	if (!event.HasEvent) {
		return result;
	}

	result.Fresh = timestampIsFresh(nowNt, event.Timestamp);
	result.SchedulerAccepted = event.SchedulerAccepted;
	result.RatiosValid = event.RatiosValid;
	result.Stage1Supplied = event.Stage1Supplied;
	result.Stage2Supplied = event.Stage2Supplied;
	result.WallModelZeroClamp = event.WallModelZeroClamp;
	result.OwnerCylinder = event.OwnerCylinder;
	result.AffectedCylinderMask = event.AffectedCylinderMask;
	result.AffectedBankMask = event.AffectedBankMask;
	result.PublicationGeneration = event.PublicationGeneration;
	result.Serial = event.Serial;
	result.AgeMilliseconds = nowNt.count > event.Timestamp.count ? elapsedMilliseconds(nowNt, event.Timestamp) : 0;

	result.CalculatedBaseMassMilligrams = gramsToMilligrams(event.BasePreAeMassGrams);
	result.CalculatedAppliedAeMassMilligrams = gramsToMilligrams(event.AppliedAeMassGrams);
	result.CalculatedFilmCorrectionMassMilligrams = gramsToMilligrams(event.FilmCorrectionMassGrams);
	result.AttemptedFinalCommandedMassMilligrams = gramsToMilligrams(event.FinalCommandedMassGrams);
	result.ScheduledCommandMassMilligrams = event.SchedulerAccepted ? result.AttemptedFinalCommandedMassMilligrams : 0;
	result.Stage1MassMilligrams = gramsToMilligrams(event.Stage1MassGrams);
	result.Stage2MassMilligrams = gramsToMilligrams(event.Stage2MassGrams);

	if (event.RatiosValid) {
		result.AePercentOfBase = ratioPercent(event.AppliedAeMassGrams, event.BasePreAeMassGrams);
		result.FilmPercentOfBase = ratioPercent(event.FilmCorrectionMassGrams, event.BasePreAeMassGrams);
		result.CombinedPercentOfBase = result.AePercentOfBase + result.FilmPercentOfBase;
	}
	return result;
}

VeAnalyzeFuelEventRecord VeAnalyzeFuelEvents::cylinderSnapshot(size_t ownerCylinder, efitick_t nowNt) const {
	if (ownerCylinder >= MaximumCylinders) {
		return {};
	}
	return makeRecord(m_cylinders[ownerCylinder], nowNt);
}

void VeAnalyzeFuelEvents::updateBankConditions(
		BankState& bank,
		uint16_t cylinderMask,
		efitick_t nowNt,
		bool normalActive,
		uint32_t normalReasons,
		bool cutActive,
		uint32_t cutReasons) {
	const uint16_t priorNormal = bank.ActiveNormalCylinders & cylinderMask;
	const uint16_t priorCut = bank.ActiveCutCylinders & cylinderMask;

	if (normalActive) {
		bank.ActiveNormalCylinders |= cylinderMask;
		if (bank.NormalUntilNt <= nowNt.count) {
			bank.NormalReasons = 0;
		}
		bank.NormalReasons |= normalReasons | static_cast<uint32_t>(FuelEventReason::NormalRecovery);
		bank.NormalUntilNt = std::max(bank.NormalUntilNt, saturatingAddNt(nowNt, m_normalRecoveryNt));
	} else if (priorNormal != 0) {
		bank.ActiveNormalCylinders &= ~cylinderMask;
		bank.NormalReasons |= static_cast<uint32_t>(FuelEventReason::NormalRecovery);
		bank.NormalUntilNt = std::max(bank.NormalUntilNt, saturatingAddNt(nowNt, m_normalRecoveryNt));
	}

	if (cutActive) {
		bank.ActiveCutCylinders |= cylinderMask;
		if (bank.CutUntilNt <= nowNt.count) {
			bank.CutReasons = 0;
		}
		bank.CutReasons |= cutReasons | static_cast<uint32_t>(FuelEventReason::NormalRecovery);
		bank.CutUntilNt = std::max(bank.CutUntilNt, saturatingAddNt(nowNt, m_cutRecoveryNt));
	} else if (priorCut != 0) {
		bank.ActiveCutCylinders &= ~cylinderMask;
		bank.CutReasons |= static_cast<uint32_t>(FuelEventReason::NormalRecovery);
		bank.CutUntilNt = std::max(bank.CutUntilNt, saturatingAddNt(nowNt, m_cutRecoveryNt));
	}
}

void VeAnalyzeFuelEvents::updatePeak(BankState& bank, efitick_t nowNt, const StoredEvent& event) {
	if (bank.PeakUntilNt <= nowNt.count) {
		bank.PeakAePercent = 0;
		bank.PeakFilmPercent = 0;
		bank.PeakCombinedPercent = 0;
	}
	bank.PeakUntilNt = std::max({bank.PeakUntilNt, bank.NormalUntilNt, bank.CutUntilNt});
	if (!event.RatiosValid) {
		return;
	}

	const float ae = ratioPercent(event.AppliedAeMassGrams, event.BasePreAeMassGrams);
	const float film = ratioPercent(event.FilmCorrectionMassGrams, event.BasePreAeMassGrams);
	const float combined = ae + film;
	if (bank.PeakUntilNt <= nowNt.count) {
		bank.PeakAePercent = ae;
		bank.PeakFilmPercent = film;
		bank.PeakCombinedPercent = combined;
		bank.PeakUntilNt = saturatingAddNt(nowNt, m_eventFreshnessNt);
		return;
	}

	if (ae > bank.PeakAePercent) {
		bank.PeakAePercent = ae;
	}
	if (film > bank.PeakFilmPercent) {
		bank.PeakFilmPercent = film;
	}
	if (combined > bank.PeakCombinedPercent) {
		bank.PeakCombinedPercent = combined;
	}
}

bool VeAnalyzeFuelEvents::observeEvent(const VeAnalyzeFuelEventInput& input) {
	if (!m_configValid) {
		return false;
	}

	m_attemptedEvents = saturatingIncrement(m_attemptedEvents);

	uint32_t structuralReasons = 0;
	if (input.OwnerCylinder >= MaximumCylinders) {
		structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidOwnerCylinder);
	}

	const uint32_t cylinderMask = input.AffectedCylinderMask;
	const bool validCylinderMask = cylinderMask != 0 && (cylinderMask & ~CylinderMaskAll) == 0 &&
								   input.OwnerCylinder < MaximumCylinders &&
								   (cylinderMask & cylinderBit(input.OwnerCylinder)) != 0;
	if (!validCylinderMask) {
		structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidCylinderMask);
	}

	const uint32_t bankMask = input.AffectedBankMask;
	const bool validBankMask = bankMask != 0 && (bankMask & ~BankMaskAll) == 0;
	if (!validBankMask) {
		structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidBankMask);
	}

	if (input.PublicationGeneration == 0) {
		structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidGeneration);
	}
	if (input.IntegrationUnsupported) {
		structuralReasons |= static_cast<uint32_t>(FuelEventReason::IntegrationUnsupported);
	}

	if (input.ExpectedCylinderMask != 0) {
		if ((input.ExpectedCylinderMask & ~CylinderMaskAll) != 0 ||
			(input.AffectedCylinderMask & ~input.ExpectedCylinderMask) != 0) {
			structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidCylinderMask);
		} else {
			m_expectedCylinderMask = input.ExpectedCylinderMask;
		}
	}
	if (input.ExpectedBankMask != 0) {
		if ((input.ExpectedBankMask & ~BankMaskAll) != 0 || (input.AffectedBankMask & ~input.ExpectedBankMask) != 0) {
			structuralReasons |= static_cast<uint32_t>(FuelEventReason::InvalidBankMask);
		} else {
			m_expectedBankMask = input.ExpectedBankMask;
		}
	}

	const uint16_t acceptedCylinderMask = validCylinderMask ? input.AffectedCylinderMask : 0;
	const uint8_t acceptedBankMask = validBankMask ? input.AffectedBankMask : 0;
	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		if (isBitSet(acceptedBankMask, bankIndex)) {
			m_banks[bankIndex].AttemptedEvents = saturatingIncrement(m_banks[bankIndex].AttemptedEvents);
		}
	}

	const uint32_t schedulingReasons =
			input.SchedulerAccepted ? 0 : static_cast<uint32_t>(FuelEventReason::SchedulingRejectedRecovery);
	if (!input.SchedulerAccepted) {
		m_rejectedEvents = saturatingIncrement(m_rejectedEvents);
		for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
			if (isBitSet(acceptedBankMask, bankIndex)) {
				m_banks[bankIndex].SchedulerRejectedEvents =
						saturatingIncrement(m_banks[bankIndex].SchedulerRejectedEvents);
			}
		}
	}

	if (hasAnyReason(structuralReasons)) {
		rejectEvent(input, structuralReasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
		return false;
	}

	if (!m_hasGeneration) {
		m_hasGeneration = true;
		m_generation = input.PublicationGeneration;
	} else if (input.PublicationGeneration != m_generation) {
		if (!isGenerationNewer(input.PublicationGeneration, m_generation)) {
			const uint32_t reasons = static_cast<uint32_t>(FuelEventReason::RegressedGeneration);
			rejectEvent(input, reasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
			return false;
		}

		clearHistory();
		m_hasGeneration = true;
		m_generation = input.PublicationGeneration;
		m_expectedCylinderMask = input.ExpectedCylinderMask;
		m_expectedBankMask = input.ExpectedBankMask;
		m_attemptedEvents = 1;
		for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
			if (isBitSet(acceptedBankMask, bankIndex)) {
				m_banks[bankIndex].AttemptedEvents = 1;
				if (!input.SchedulerAccepted) {
					m_banks[bankIndex].SchedulerRejectedEvents = 1;
				}
			}
		}
		if (!input.SchedulerAccepted) {
			m_rejectedEvents = 1;
		}
		m_resetReasons = static_cast<uint32_t>(FuelEventReason::ResetRecovery);
		m_resetUntilNt = saturatingAddNt(input.Timestamp, m_normalRecoveryNt);
	}

	efitick_t latestTimestamp;
	bool hasLatestTimestamp = false;
	for (const StoredEvent& cylinder : m_cylinders) {
		if (!cylinder.HasEvent) {
			continue;
		}
		if (!hasLatestTimestamp || cylinder.Timestamp.count > latestTimestamp.count) {
			latestTimestamp = cylinder.Timestamp;
			hasLatestTimestamp = true;
		}
	}

	if (hasLatestTimestamp && input.Timestamp.count < latestTimestamp.count) {
		const uint32_t reasons = static_cast<uint32_t>(FuelEventReason::RegressedTimestamp);
		rejectEvent(input, reasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
		return false;
	}

	bool timestampConflict = false;
	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		if (!isBitSet(acceptedCylinderMask, cylinderIndex) || !m_cylinders[cylinderIndex].HasEvent) {
			continue;
		}
		if (input.Timestamp.count == m_cylinders[cylinderIndex].Timestamp.count) {
			timestampConflict = true;
			break;
		}
	}
	if (timestampConflict) {
		const uint32_t reasons = static_cast<uint32_t>(FuelEventReason::DuplicateTimestamp);
		rejectEvent(input, reasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
		return false;
	}

	uint32_t coherenceReasons = 0;
	const float base = input.BasePreAeMassGrams;
	const float ae = input.AppliedAeMassGrams;
	const float film = input.FilmCorrectionMassGrams;
	const float finalMass = input.FinalCommandedMassGrams;
	const float stage1 = input.Stage1MassGrams;
	const float stage2 = input.Stage2MassGrams;
	const bool coreFinite = std::isfinite(base) && std::isfinite(ae) && std::isfinite(film) && std::isfinite(finalMass);
	if (!coreFinite) {
		coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NonFiniteMass);
	}
	if (std::isfinite(base) && base < 0) {
		coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NegativeMass);
	}
	if (std::isfinite(finalMass) && finalMass < 0) {
		coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NegativeMass);
	}
	if (input.HasStage1Mass) {
		if (!std::isfinite(stage1)) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NonFiniteMass);
		} else if (stage1 < 0) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NegativeMass);
		}
	}
	if (input.HasStage2Mass) {
		if (!std::isfinite(stage2)) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NonFiniteMass);
		} else if (stage2 < 0) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NegativeMass);
		}
	}

	if (coreFinite && !closeEnough(finalMass, base + ae + film, m_policy)) {
		coherenceReasons |= static_cast<uint32_t>(FuelEventReason::MassIdentityMismatch);
	}

	if (coreFinite && (input.HasStage1Mass || input.HasStage2Mass)) {
		const float suppliedStageMass = (input.HasStage1Mass ? stage1 : 0) + (input.HasStage2Mass ? stage2 : 0);
		if (!closeEnough(suppliedStageMass, finalMass, m_policy)) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::StageMassMismatch);
		}
	}

	if (hasAnyReason(coherenceReasons)) {
		rejectEvent(input, coherenceReasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
		return false;
	}

	uint32_t guardReasons = 0;
	const bool ratiosValid = base > 0;
	float aePercent = 0;
	float filmPercent = 0;
	float combinedPercent = 0;
	if (ratiosValid) {
		aePercent = ratioPercent(ae, base);
		filmPercent = ratioPercent(film, base);
		combinedPercent = aePercent + filmPercent;
		if (!std::isfinite(aePercent) || !std::isfinite(filmPercent) || !std::isfinite(combinedPercent)) {
			coherenceReasons |= static_cast<uint32_t>(FuelEventReason::NonFiniteMass);
		} else {
			if (aePercent > m_policy.AeLimitPercent) {
				guardReasons |= static_cast<uint32_t>(FuelEventReason::ExcessAeRatio);
			}
			if (filmPercent > m_policy.FilmLimitPercent) {
				guardReasons |= static_cast<uint32_t>(FuelEventReason::ExcessFilmRatio);
			}
			if (combinedPercent > m_policy.CombinedLimitPercent) {
				guardReasons |= static_cast<uint32_t>(FuelEventReason::ExcessCombinedRatio);
			}
		}
	} else {
		guardReasons |= static_cast<uint32_t>(FuelEventReason::InvalidDenominator);
	}

	if (hasAnyReason(coherenceReasons)) {
		rejectEvent(input, coherenceReasons | schedulingReasons, acceptedCylinderMask, acceptedBankMask);
		return false;
	}

	StoredEvent event;
	event.HasEvent = true;
	event.SchedulerAccepted = input.SchedulerAccepted;
	event.Stage1Supplied = input.HasStage1Mass;
	event.Stage2Supplied = input.HasStage2Mass;
	event.WallModelZeroClamp = input.WallModelZeroClamp;
	event.RatiosValid = ratiosValid;
	event.OwnerCylinder = input.OwnerCylinder;
	event.AffectedCylinderMask = input.AffectedCylinderMask;
	event.AffectedBankMask = input.AffectedBankMask;
	event.PublicationGeneration = input.PublicationGeneration;
	event.Serial = input.Serial;
	event.Timestamp = input.Timestamp;
	event.BasePreAeMassGrams = base;
	event.AppliedAeMassGrams = ae;
	event.FilmCorrectionMassGrams = film;
	event.FinalCommandedMassGrams = finalMass;
	event.Stage1MassGrams = stage1;
	event.Stage2MassGrams = stage2;

	bool historyGap = false;
	m_rejectionSinceLatestEvent = false;
	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		if (!isBitSet(acceptedCylinderMask, cylinderIndex)) {
			continue;
		}
		const StoredEvent& previous = m_cylinders[cylinderIndex];
		if (previous.HasEvent && input.Timestamp.count - previous.Timestamp.count > m_eventFreshnessNt) {
			historyGap = true;
		}
		m_cylinders[cylinderIndex] = event;
		m_cylinderRejectionAfterEvent[cylinderIndex] = false;
	}

	const float zeroCommanded = std::isfinite(finalMass) && finalMass == 0;
	const bool cutActive = input.WallModelZeroClamp || zeroCommanded || !input.SchedulerAccepted;
	uint32_t cutReasons = 0;
	if (input.WallModelZeroClamp) {
		cutReasons |= static_cast<uint32_t>(FuelEventReason::WallModelClampRecovery);
	}
	if (zeroCommanded) {
		cutReasons |= static_cast<uint32_t>(FuelEventReason::ZeroCommandedRecovery);
	}
	if (!input.SchedulerAccepted) {
		cutReasons |= static_cast<uint32_t>(FuelEventReason::SchedulingRejectedRecovery);
	}

	const bool normalActive = guardReasons != 0;
	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		if (!isBitSet(acceptedBankMask, bankIndex)) {
			continue;
		}

		BankState& bank = m_banks[bankIndex];
		bank.Latest = event;
		bank.LatestRejectedAfterEvent = false;
		bank.ObservedCylinderMask |= input.AffectedCylinderMask;
		if (input.SchedulerAccepted) {
			bank.ScheduledEvents = saturatingIncrement(bank.ScheduledEvents);
		}
		updateBankConditions(
				bank, input.AffectedCylinderMask, input.Timestamp, normalActive, guardReasons, cutActive, cutReasons);
		updatePeak(bank, input.Timestamp, event);
	}

	if (input.SchedulerAccepted) {
		m_acceptedEvents = saturatingIncrement(m_acceptedEvents);
	}

	const uint32_t eventReasons = guardReasons | schedulingReasons;
	if (hasAnyReason(eventReasons)) {
		m_guardRejectedEvents = saturatingIncrement(m_guardRejectedEvents);
		recordRejectedReasons(eventReasons);
	}
	if (historyGap) {
		const uint32_t gapReasons = static_cast<uint32_t>(FuelEventReason::StaleEvent);
		m_guardRejectedEvents = saturatingIncrement(m_guardRejectedEvents);
		recordRejectedReasons(gapReasons);
		markBankFault(acceptedBankMask, input.Timestamp, gapReasons, m_normalRecoveryNt);
	}
	return true;
}

VeAnalyzeFuelEventBankSnapshot VeAnalyzeFuelEvents::makeBankSnapshot(size_t bankIndex, efitick_t nowNt) const {
	VeAnalyzeFuelEventBankSnapshot result;
	const BankState& bank = m_banks[bankIndex];

	result.Observed = bank.Latest.HasEvent || bank.ObservedCylinderMask != 0;
	result.ObservedCylinderMask = bank.ObservedCylinderMask;
	result.ActiveNormalCylinderMask = bank.ActiveNormalCylinders;
	result.ActiveCutCylinderMask = bank.ActiveCutCylinders;
	result.AttemptedEventCount = bank.AttemptedEvents;
	result.ScheduledEventCount = bank.ScheduledEvents;
	result.SchedulerRejectedEventCount = bank.SchedulerRejectedEvents;
	result.LatestEvent = makeRecord(bank.Latest, nowNt);
	result.LatestEvent.SupersededByRejection = bank.Latest.HasEvent && bank.LatestRejectedAfterEvent;
	if (bank.PeakUntilNt > nowNt.count) {
		result.PeakAePercentOfBase = bank.PeakAePercent;
		result.PeakFilmPercentOfBase = bank.PeakFilmPercent;
		result.PeakCombinedPercentOfBase = bank.PeakCombinedPercent;
	}

	bool hasCylinderEvidence = false;
	bool allEvidenceFresh = true;
	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		const StoredEvent& cylinder = m_cylinders[cylinderIndex];
		if (!cylinder.HasEvent || !isBitSet(cylinder.AffectedBankMask, bankIndex)) {
			continue;
		}

		hasCylinderEvidence = true;
		if (nowNt.count < cylinder.Timestamp.count) {
			result.Reasons |= static_cast<uint32_t>(FuelEventReason::FutureTimestamp);
			allEvidenceFresh = false;
		} else if (!timestampIsFresh(nowNt, cylinder.Timestamp)) {
			result.Reasons |= static_cast<uint32_t>(FuelEventReason::StaleEvent);
			allEvidenceFresh = false;
		}
	}

	result.EvidenceFresh = result.Observed && hasCylinderEvidence && allEvidenceFresh;
	if (result.Observed && !hasCylinderEvidence) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::NoFreshEvent);
	}

	if (bank.ActiveNormalCylinders != 0) {
		result.Reasons |= bank.NormalReasons;
	}
	if (bank.ActiveCutCylinders != 0) {
		result.Reasons |= bank.CutReasons;
	}
	if (bank.NormalUntilNt > nowNt.count) {
		result.Reasons |= bank.NormalReasons;
	}
	if (bank.CutUntilNt > nowNt.count) {
		result.Reasons |= bank.CutReasons;
	}
	if (bank.FaultUntilNt > nowNt.count) {
		result.Reasons |= bank.FaultReasons;
	}
	if ((bank.NormalUntilNt > nowNt.count || bank.CutUntilNt > nowNt.count) &&
		!result.hasReason(FuelEventReason::NormalRecovery)) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::NormalRecovery);
	}

	if (m_resetUntilNt > nowNt.count) {
		result.Reasons |= m_resetReasons;
	}
	if (m_globalFaultUntilNt > nowNt.count) {
		result.Reasons |= m_globalFaultReasons;
	}
	return result;
}

VeAnalyzeFuelEventCylinderSnapshot VeAnalyzeFuelEvents::makeCylinderSnapshot(
		size_t cylinderIndex,
		efitick_t nowNt,
		const std::array<VeAnalyzeFuelEventBankSnapshot, ve_analyze_fuel_events::MaximumBanks>& banks) const {
	VeAnalyzeFuelEventCylinderSnapshot result;
	const StoredEvent& event = m_cylinders[cylinderIndex];
	const VeAnalyzeFuelEventRecord record = makeRecord(event, nowNt);
	result.HasEvent = record.HasEvent;
	result.Fresh = record.Fresh;
	result.SupersededByRejection = event.HasEvent && m_cylinderRejectionAfterEvent[cylinderIndex];
	result.Serial = record.Serial;
	result.AgeMilliseconds = record.AgeMilliseconds;
	result.CalculatedBaseMassMilligrams = record.CalculatedBaseMassMilligrams;
	result.CalculatedAppliedAeMassMilligrams = record.CalculatedAppliedAeMassMilligrams;
	result.CalculatedFilmCorrectionMassMilligrams = record.CalculatedFilmCorrectionMassMilligrams;
	result.AttemptedFinalCommandedMassMilligrams = record.AttemptedFinalCommandedMassMilligrams;
	result.ScheduledCommandMassMilligrams = record.ScheduledCommandMassMilligrams;

	if (!event.HasEvent) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::NoFreshEvent);
	} else if (nowNt.count < event.Timestamp.count) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::FutureTimestamp);
	} else if (!timestampIsFresh(nowNt, event.Timestamp)) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::StaleEvent);
	}

	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		if (isBitSet(event.AffectedBankMask, bankIndex)) {
			result.Reasons |= banks[bankIndex].Reasons;
		}
	}
	if (m_resetUntilNt > nowNt.count) {
		result.Reasons |= m_resetReasons;
	}
	return result;
}

VeAnalyzeFuelEventSnapshot VeAnalyzeFuelEvents::snapshot(efitick_t nowNt) const {
	VeAnalyzeFuelEventSnapshot result;
	if (!m_configValid) {
		result.Reasons = static_cast<uint32_t>(FuelEventReason::InvalidConfiguration);
		return result;
	}

	result.AttemptedEventCount = m_attemptedEvents;
	result.AcceptedEventCount = m_acceptedEvents;
	result.RejectedEventCount = m_rejectedEvents;
	result.GuardRejectedEventCount = m_guardRejectedEvents;
	result.InvalidEvidenceEventCount = m_invalidEvidenceEvents;
	result.RejectedEventCountsByReason = m_rejectedByReason;
	result.PublicationGeneration = m_generation;
	result.ExpectedCylinderMask = m_expectedCylinderMask;
	result.ExpectedBankMask = m_expectedBankMask;

	efitick_t latestTimestamp;
	bool hasLatestTimestamp = false;
	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		const StoredEvent& cylinder = m_cylinders[cylinderIndex];
		if (!cylinder.HasEvent) {
			continue;
		}

		result.ObservedCylinderMask |= static_cast<uint16_t>(1u << cylinderIndex);
		result.ObservedBankMask |= cylinder.AffectedBankMask;
		if (timestampIsFresh(nowNt, cylinder.Timestamp)) {
			result.FreshCylinderMask |= static_cast<uint16_t>(1u << cylinderIndex);
		}
		if (!hasLatestTimestamp || cylinder.Timestamp.count > latestTimestamp.count) {
			latestTimestamp = cylinder.Timestamp;
			result.LastEvent = makeRecord(cylinder, nowNt);
			result.LastEvent.SupersededByRejection = m_rejectionSinceLatestEvent;
			hasLatestTimestamp = true;
		}
	}

	result.HasFreshEvent = false;
	for (size_t bankIndex = 0; bankIndex < MaximumBanks; bankIndex++) {
		result.Banks[bankIndex] = makeBankSnapshot(bankIndex, nowNt);
		if (result.Banks[bankIndex].EvidenceFresh) {
			result.FreshBankMask |= static_cast<uint8_t>(1u << bankIndex);
		}
		if (isBitSet(m_expectedBankMask, bankIndex) && !result.Banks[bankIndex].EvidenceFresh) {
			result.Banks[bankIndex].Reasons |= static_cast<uint32_t>(FuelEventReason::NoFreshEvent);
		}
		result.Reasons |= result.Banks[bankIndex].Reasons;
		result.HasFreshEvent |= result.Banks[bankIndex].EvidenceFresh;
	}

	for (size_t cylinderIndex = 0; cylinderIndex < MaximumCylinders; cylinderIndex++) {
		result.Cylinders[cylinderIndex] = makeCylinderSnapshot(cylinderIndex, nowNt, result.Banks);
		if (isBitSet(m_expectedCylinderMask, cylinderIndex) && !isBitSet(result.FreshCylinderMask, cylinderIndex)) {
			result.Cylinders[cylinderIndex].Reasons |= static_cast<uint32_t>(FuelEventReason::NoFreshEvent);
		}
	}

	result.CoverageComplete =
			(m_expectedCylinderMask == 0 ||
			 (result.FreshCylinderMask & m_expectedCylinderMask) == m_expectedCylinderMask) &&
			(m_expectedBankMask == 0 || (result.FreshBankMask & m_expectedBankMask) == m_expectedBankMask);

	if (result.ObservedBankMask == 0 || !result.HasFreshEvent || !result.CoverageComplete) {
		result.Reasons |= static_cast<uint32_t>(FuelEventReason::NoFreshEvent);
	}

	if (hasLatestTimestamp) {
		if (nowNt.count < latestTimestamp.count) {
			result.Reasons |= static_cast<uint32_t>(FuelEventReason::FutureTimestamp);
		}
		result.LastEventAgeMilliseconds = elapsedMilliseconds(nowNt, latestTimestamp);
	}

	if (m_resetUntilNt > nowNt.count) {
		result.Reasons |= m_resetReasons;
	}
	if (m_globalFaultUntilNt > nowNt.count) {
		result.Reasons |= m_globalFaultReasons;
	}

	result.Ready = result.HasFreshEvent && result.CoverageComplete && result.Reasons == 0;
	return result;
}
