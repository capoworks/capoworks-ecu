/**
 * @file test_ve_analyze_fuel_events.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "ve_analyze_fuel_events.h"

#include <array>
#include <cmath>
#include <limits>

namespace {
using ve_analyze_fuel_events::FuelEventReason;

constexpr int64_t Millisecond = 1000;
constexpr uint32_t Generation = 7;

efitick_t timeUs(int64_t microseconds) {
	return efitick_t{US2NT(microseconds).count()};
}

class FuelEventsFixture {
public:
	VeAnalyzeFuelEventInput
	make(int64_t nowUs,
		 float base,
		 float ae = 0,
		 float film = 0,
		 uint8_t owner = 0,
		 uint16_t cylinderMask = 1,
		 uint8_t bankMask = 1,
		 bool schedulerAccepted = true,
		 bool wallClamp = false,
		 uint32_t generation = Generation) {
		VeAnalyzeFuelEventInput event;
		event.OwnerCylinder = owner;
		event.AffectedCylinderMask = cylinderMask;
		event.AffectedBankMask = bankMask;
		event.PublicationGeneration = generation;
		event.Serial = nextSerial++;
		event.Timestamp = timeUs(nowUs);
		event.BasePreAeMassGrams = base;
		event.AppliedAeMassGrams = ae;
		event.FilmCorrectionMassGrams = film;
		event.FinalCommandedMassGrams = base + ae + film;
		event.SchedulerAccepted = schedulerAccepted;
		event.WallModelZeroClamp = wallClamp;
		return event;
	}

	VeAnalyzeFuelEventInput
	normal(int64_t nowUs,
		   uint8_t owner = 0,
		   uint16_t cylinderMask = 1,
		   uint8_t bankMask = 1,
		   float base = 1.0f,
		   uint32_t generation = Generation) {
		return make(nowUs, base, 0, 0, owner, cylinderMask, bankMask, true, false, generation);
	}

	VeAnalyzeFuelEvents Collector;
	uint64_t nextSerial = 1;
};

void feed(FuelEventsFixture& fixture, const VeAnalyzeFuelEventInput& event) {
	EXPECT_TRUE(fixture.Collector.observeEvent(event));
}

void feedNormalUntil(FuelEventsFixture& fixture, int64_t& nowUs, int64_t endUs, uint32_t generation = Generation) {
	while (nowUs < endUs) {
		nowUs += 200 * Millisecond;
		if (nowUs > endUs) {
			nowUs = endUs;
		}
		feed(fixture, fixture.normal(nowUs, 0, 1, 1, 1.0f, generation));
	}
}
} // namespace

TEST(VeAnalyzeFuelEvents, ExportsExplicitMilligramsAndAttemptedVersusScheduledMass) {
	FuelEventsFixture fixture;
	auto event = fixture.make(0, 1.234f, 0.005f, 0.006f);
	feed(fixture, event);

	auto snapshot = fixture.Collector.snapshot(timeUs(0));
	ASSERT_TRUE(snapshot.LastEvent.HasEvent);
	EXPECT_FLOAT_EQ(1234.0f, snapshot.LastEvent.CalculatedBaseMassMilligrams);
	EXPECT_FLOAT_EQ(5.0f, snapshot.LastEvent.CalculatedAppliedAeMassMilligrams);
	EXPECT_FLOAT_EQ(6.0f, snapshot.LastEvent.CalculatedFilmCorrectionMassMilligrams);
	EXPECT_FLOAT_EQ(1245.0f, snapshot.LastEvent.AttemptedFinalCommandedMassMilligrams);
	EXPECT_FLOAT_EQ(1245.0f, snapshot.LastEvent.ScheduledCommandMassMilligrams);
	EXPECT_TRUE(snapshot.LastEvent.SchedulerAccepted);

	EXPECT_FLOAT_EQ(1234.0f, ve_analyze_fuel_events::gramsToMilligrams(1.234f));
	EXPECT_FLOAT_EQ(1.234f, ve_analyze_fuel_events::milligramsToGrams(1234.0f));
}

TEST(VeAnalyzeFuelEvents, KeepsAlreadyModeScaledInputsWithoutApplyingModeAgain) {
	const std::array<float, 4> modeScaledBases = {10.0f, 5.0f, 10.0f, 2.5f};
	for (const float base : modeScaledBases) {
		FuelEventsFixture fixture;
		const float ae = base * 0.001f;
		const float film = base * 0.002f;
		feed(fixture, fixture.make(0, base, ae, film));

		auto snapshot = fixture.Collector.snapshot(timeUs(0));
		EXPECT_FLOAT_EQ(base * 1000.0f, snapshot.LastEvent.CalculatedBaseMassMilligrams);
		EXPECT_NEAR(0.1f, snapshot.LastEvent.AePercentOfBase, 0.00001f);
		EXPECT_NEAR(0.2f, snapshot.LastEvent.FilmPercentOfBase, 0.00001f);
		EXPECT_TRUE(snapshot.Ready);
	}
}

TEST(VeAnalyzeFuelEvents, OpposingAeAndFilmAreKeptAsAbsoluteModules) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f, -0.03f));

	auto snapshot = fixture.Collector.snapshot(timeUs(0));
	ASSERT_TRUE(snapshot.LastEvent.HasEvent);
	EXPECT_NEAR(2.0f, snapshot.LastEvent.AePercentOfBase, 0.00001f);
	EXPECT_NEAR(3.0f, snapshot.LastEvent.FilmPercentOfBase, 0.00001f);
	EXPECT_NEAR(5.0f, snapshot.LastEvent.CombinedPercentOfBase, 0.00001f);
	EXPECT_FALSE(snapshot.Ready);
	EXPECT_TRUE(snapshot.hasReason(FuelEventReason::ExcessFilmRatio));
	EXPECT_TRUE(snapshot.hasReason(FuelEventReason::ExcessCombinedRatio));
}

TEST(VeAnalyzeFuelEvents, QuietSecondBankCannotHideFirstBankViolation) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f, 0));
	feed(fixture, fixture.make(100 * Millisecond, 1.0f, 0, 0, 1, 2, 2));

	auto snapshot = fixture.Collector.snapshot(timeUs(200 * Millisecond));
	EXPECT_EQ(3, snapshot.ObservedBankMask);
	EXPECT_FALSE(snapshot.Ready);
	EXPECT_TRUE(snapshot.Banks[0].hasReason(FuelEventReason::ExcessAeRatio));
	EXPECT_FALSE(snapshot.Banks[1].hasReason(FuelEventReason::ExcessAeRatio));
}

TEST(VeAnalyzeFuelEvents, OneEventAppliesItsEvidenceToEveryAffectedBank) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f, 0, 0, 1, 3));

	auto snapshot = fixture.Collector.snapshot(timeUs(0));
	EXPECT_EQ(3, snapshot.ObservedBankMask);
	EXPECT_TRUE(snapshot.Banks[0].hasReason(FuelEventReason::ExcessAeRatio));
	EXPECT_TRUE(snapshot.Banks[1].hasReason(FuelEventReason::ExcessAeRatio));
	EXPECT_EQ(1, snapshot.Banks[0].ActiveNormalCylinderMask);
	EXPECT_EQ(1, snapshot.Banks[1].ActiveNormalCylinderMask);
	EXPECT_EQ(1u, snapshot.Banks[0].ScheduledEventCount);
	EXPECT_EQ(1u, snapshot.Banks[1].ScheduledEventCount);
}

TEST(VeAnalyzeFuelEvents, MultipleCylindersInOneBankRemainIndividuallyActive) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f, 0, 0, 1, 1));
	feed(fixture, fixture.make(100 * Millisecond, 1.0f, 0, 0, 1, 2, 1));

	auto snapshot = fixture.Collector.snapshot(timeUs(200 * Millisecond));
	EXPECT_FALSE(snapshot.Ready);
	EXPECT_EQ(1, snapshot.Banks[0].ActiveNormalCylinderMask);
	EXPECT_EQ(3, snapshot.Banks[0].ObservedCylinderMask);
	EXPECT_TRUE(snapshot.Banks[0].hasReason(FuelEventReason::ExcessAeRatio));
	EXPECT_EQ(2u, snapshot.Banks[0].ScheduledEventCount);
}

TEST(VeAnalyzeFuelEvents, ShortEventSurvivesSubTwentyHertzPolling) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f, 0));

	for (int64_t nowUs = 50 * Millisecond; nowUs <= 200 * Millisecond; nowUs += 50 * Millisecond) {
		auto snapshot = fixture.Collector.snapshot(timeUs(nowUs));
		EXPECT_TRUE(snapshot.LastEvent.Fresh);
		EXPECT_FALSE(snapshot.Ready);
		EXPECT_TRUE(snapshot.Banks[0].hasReason(FuelEventReason::ExcessAeRatio));
		EXPECT_NEAR(2.0f, snapshot.Banks[0].PeakAePercentOfBase, 0.00001f);
		EXPECT_EQ(static_cast<uint32_t>(nowUs / Millisecond), snapshot.LastEventAgeMilliseconds);
	}
}

TEST(VeAnalyzeFuelEvents, ExpiredLargePeakIsNotRevivedBySmallTransient) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0.02f));
	auto large = fixture.Collector.snapshot(timeUs(0));
	EXPECT_NEAR(2.0f, large.Banks[0].PeakAePercentOfBase, 0.00001f);

	feed(fixture, fixture.make(1600 * Millisecond, 1.0f, 0.002f));
	auto small = fixture.Collector.snapshot(timeUs(1600 * Millisecond));
	EXPECT_NEAR(0.2f, small.Banks[0].PeakAePercentOfBase, 0.00001f);
	EXPECT_NEAR(0.0f, small.Banks[0].PeakFilmPercentOfBase, 0.00001f);
	EXPECT_NEAR(0.2f, small.Banks[0].PeakCombinedPercentOfBase, 0.00001f);
}

TEST(VeAnalyzeFuelEvents, ZeroCommandedWithoutClampFlagStillUsesCutRecovery) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 0, 0, 0, 0, 1, 1, true, false));

	auto zero = fixture.Collector.snapshot(timeUs(0));
	EXPECT_FALSE(zero.Ready);
	EXPECT_TRUE(zero.hasReason(FuelEventReason::InvalidDenominator));
	EXPECT_TRUE(zero.hasReason(FuelEventReason::ZeroCommandedRecovery));
	EXPECT_FALSE(zero.hasReason(FuelEventReason::WallModelClampRecovery));

	int64_t nowUs = 0;
	feed(fixture, fixture.normal(100 * Millisecond));
	nowUs = 100 * Millisecond;
	feedNormalUntil(fixture, nowUs, 2000 * Millisecond);
	EXPECT_EQ(2000 * Millisecond, nowUs);

	auto before = fixture.Collector.snapshot(timeUs(2099 * Millisecond));
	EXPECT_FALSE(before.Ready);
	EXPECT_TRUE(before.hasReason(FuelEventReason::ZeroCommandedRecovery));

	auto recovered = fixture.Collector.snapshot(timeUs(2100 * Millisecond));
	EXPECT_TRUE(recovered.LastEvent.Fresh);
	EXPECT_TRUE(recovered.Ready);
	EXPECT_FALSE(recovered.hasReason(FuelEventReason::ZeroCommandedRecovery));
}

TEST(VeAnalyzeFuelEvents, WallModelClampFlagUsesCutRecoveryEvenWithNonzeroMass) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0, 0, 0, 1, 1, true, true));

	auto clamped = fixture.Collector.snapshot(timeUs(0));
	EXPECT_FALSE(clamped.Ready);
	EXPECT_TRUE(clamped.hasReason(FuelEventReason::WallModelClampRecovery));
	EXPECT_FALSE(clamped.hasReason(FuelEventReason::ZeroCommandedRecovery));
}

TEST(VeAnalyzeFuelEvents, SchedulerRejectionIsNeverCountedAsAccepted) {
	FuelEventsFixture fixture;
	auto rejected = fixture.make(0, 1.0f, 0, 0, 0, 1, 1, false);
	EXPECT_TRUE(fixture.Collector.observeEvent(rejected));

	auto snapshot = fixture.Collector.snapshot(timeUs(0));
	EXPECT_EQ(1u, snapshot.AttemptedEventCount);
	EXPECT_EQ(0u, snapshot.AcceptedEventCount);
	EXPECT_EQ(1u, snapshot.RejectedEventCount);
	EXPECT_EQ(0u, snapshot.InvalidEvidenceEventCount);
	EXPECT_FALSE(snapshot.LastEvent.SchedulerAccepted);
	EXPECT_FLOAT_EQ(0, snapshot.LastEvent.ScheduledCommandMassMilligrams);
	EXPECT_FLOAT_EQ(1000.0f, snapshot.LastEvent.AttemptedFinalCommandedMassMilligrams);
	EXPECT_TRUE(snapshot.hasReason(FuelEventReason::SchedulingRejectedRecovery));

	feed(fixture, fixture.normal(100 * Millisecond));
	snapshot = fixture.Collector.snapshot(timeUs(100 * Millisecond));
	EXPECT_EQ(2u, snapshot.AttemptedEventCount);
	EXPECT_EQ(1u, snapshot.AcceptedEventCount);
	EXPECT_EQ(1u, snapshot.RejectedEventCount);
	EXPECT_EQ(1u, snapshot.Banks[0].ScheduledEventCount);
	EXPECT_EQ(1u, snapshot.Banks[0].SchedulerRejectedEventCount);
}

TEST(VeAnalyzeFuelEvents, SuppliesStagesWithoutCountingThemTwice) {
	FuelEventsFixture fixture;
	auto event = fixture.make(0, 0.8f, 0.1f, 0.1f);
	event.HasStage1Mass = true;
	event.Stage1MassGrams = 0.6f;
	event.HasStage2Mass = true;
	event.Stage2MassGrams = 0.4f;
	EXPECT_TRUE(fixture.Collector.observeEvent(event));

	auto snapshot = fixture.Collector.snapshot(timeUs(0));
	ASSERT_TRUE(snapshot.LastEvent.HasEvent);
	EXPECT_TRUE(snapshot.LastEvent.Stage1Supplied);
	EXPECT_TRUE(snapshot.LastEvent.Stage2Supplied);
	EXPECT_FLOAT_EQ(600.0f, snapshot.LastEvent.Stage1MassMilligrams);
	EXPECT_FLOAT_EQ(400.0f, snapshot.LastEvent.Stage2MassMilligrams);
	EXPECT_NEAR(12.5f, snapshot.LastEvent.AePercentOfBase, 0.00001f);
	EXPECT_NEAR(12.5f, snapshot.LastEvent.FilmPercentOfBase, 0.00001f);
	EXPECT_FLOAT_EQ(1000.0f, snapshot.LastEvent.AttemptedFinalCommandedMassMilligrams);

	auto badStage = fixture.make(100 * Millisecond, 0.8f, 0.1f, 0.1f);
	badStage.HasStage1Mass = true;
	badStage.Stage1MassGrams = 0.6f;
	badStage.HasStage2Mass = true;
	badStage.Stage2MassGrams = 0.3f;
	EXPECT_FALSE(fixture.Collector.observeEvent(badStage));
	snapshot = fixture.Collector.snapshot(timeUs(100 * Millisecond));
	EXPECT_EQ(1u, snapshot.InvalidEvidenceEventCount);
	EXPECT_EQ(event.Serial, snapshot.LastEvent.Serial);
	EXPECT_EQ(1u, snapshot.rejectedEventCount(FuelEventReason::StageMassMismatch));
}

TEST(VeAnalyzeFuelEvents, ExactThresholdsPassAndRecoveryEndsAtExactDeadline) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 100.0f, 1.0f, -1.0f));

	auto exact = fixture.Collector.snapshot(timeUs(0));
	EXPECT_TRUE(exact.Ready);
	EXPECT_FALSE(exact.hasReason(FuelEventReason::ExcessAeRatio));
	EXPECT_FALSE(exact.hasReason(FuelEventReason::ExcessFilmRatio));
	EXPECT_FALSE(exact.hasReason(FuelEventReason::ExcessCombinedRatio));
	EXPECT_NEAR(2.0f, exact.LastEvent.CombinedPercentOfBase, 0.00001f);

	feed(fixture, fixture.make(100 * Millisecond, 100.0f, 1.001f, 0));
	auto over = fixture.Collector.snapshot(timeUs(100 * Millisecond));
	EXPECT_FALSE(over.Ready);
	EXPECT_TRUE(over.hasReason(FuelEventReason::ExcessAeRatio));

	int64_t nowUs = 100 * Millisecond;
	feed(fixture, fixture.normal(200 * Millisecond));
	nowUs = 200 * Millisecond;
	feedNormalUntil(fixture, nowUs, 1600 * Millisecond);
	auto before = fixture.Collector.snapshot(timeUs(1699 * Millisecond));
	EXPECT_FALSE(before.Ready);
	EXPECT_TRUE(before.hasReason(FuelEventReason::ExcessAeRatio));

	auto recovered = fixture.Collector.snapshot(timeUs(1700 * Millisecond));
	EXPECT_TRUE(recovered.LastEvent.Fresh);
	EXPECT_TRUE(recovered.Ready);
}

TEST(VeAnalyzeFuelEvents, NonBinaryPolicyDurationRoundsUpToFullTick) {
	VeAnalyzeFuelEventPolicy policy;
	policy.NormalRecoverySeconds = 0.4f;
	policy.EventFreshnessSeconds = 1.0f;
	VeAnalyzeFuelEvents collector(policy);

	FuelEventsFixture fixture;
	auto event = fixture.make(0, 1.0f, 0.02f);
	EXPECT_TRUE(collector.observeEvent(event));
	auto normal = fixture.make(200 * Millisecond, 1.0f);
	EXPECT_TRUE(collector.observeEvent(normal));

	auto before = collector.snapshot(timeUs(600 * Millisecond));
	EXPECT_TRUE(before.LastEvent.Fresh);
	EXPECT_FALSE(before.Ready);
	EXPECT_TRUE(before.hasReason(FuelEventReason::NormalRecovery));

	auto recovered = collector.snapshot(timeUs(600 * Millisecond + 1));
	EXPECT_TRUE(recovered.LastEvent.Fresh);
	EXPECT_TRUE(recovered.Ready);
}

TEST(VeAnalyzeFuelEvents, InvalidEvidenceFailsClosedAndDoesNotReplaceLastCoherentEvent) {
	FuelEventsFixture fixture;
	auto invalidMass = fixture.make(0, 1.0f);
	invalidMass.FinalCommandedMassGrams = NAN;
	EXPECT_FALSE(fixture.Collector.observeEvent(invalidMass));

	feed(fixture, fixture.make(100 * Millisecond, 1.0f));
	const uint64_t coherentSerial = fixture.nextSerial - 1;

	auto zeroBase = fixture.make(200 * Millisecond, 0, 1.0f, 0);
	EXPECT_TRUE(fixture.Collector.observeEvent(zeroBase));
	auto zeroSnapshot = fixture.Collector.snapshot(timeUs(200 * Millisecond));
	EXPECT_TRUE(zeroSnapshot.hasReason(FuelEventReason::InvalidDenominator));
	EXPECT_FALSE(zeroSnapshot.LastEvent.RatiosValid);
	EXPECT_EQ(zeroBase.Serial, zeroSnapshot.LastEvent.Serial);

	auto badIdentity = fixture.make(300 * Millisecond, 1.0f);
	badIdentity.FinalCommandedMassGrams = 1.01f;
	EXPECT_FALSE(fixture.Collector.observeEvent(badIdentity));
	auto snapshot = fixture.Collector.snapshot(timeUs(300 * Millisecond));
	EXPECT_EQ(2u, snapshot.InvalidEvidenceEventCount);
	EXPECT_TRUE(snapshot.hasReason(FuelEventReason::MassIdentityMismatch));
	EXPECT_EQ(zeroBase.Serial, snapshot.LastEvent.Serial);
	EXPECT_NE(coherentSerial, snapshot.LastEvent.Serial);
}

TEST(VeAnalyzeFuelEvents, DuplicateAndRegressedTimestampsCannotReplaceEvidence) {
	FuelEventsFixture fixture;
	auto first = fixture.make(100 * Millisecond, 1.0f);
	feed(fixture, first);

	auto duplicate = fixture.make(100 * Millisecond, 2.0f);
	EXPECT_FALSE(fixture.Collector.observeEvent(duplicate));
	auto duplicateSnapshot = fixture.Collector.snapshot(timeUs(100 * Millisecond));
	EXPECT_TRUE(duplicateSnapshot.hasReason(FuelEventReason::DuplicateTimestamp));
	EXPECT_EQ(first.Serial, duplicateSnapshot.LastEvent.Serial);
	EXPECT_FLOAT_EQ(1000.0f, duplicateSnapshot.LastEvent.CalculatedBaseMassMilligrams);

	auto regressed = fixture.make(50 * Millisecond, 3.0f);
	EXPECT_FALSE(fixture.Collector.observeEvent(regressed));
	auto regressedSnapshot = fixture.Collector.snapshot(timeUs(100 * Millisecond));
	EXPECT_TRUE(regressedSnapshot.hasReason(FuelEventReason::RegressedTimestamp));
	EXPECT_EQ(first.Serial, regressedSnapshot.LastEvent.Serial);
	EXPECT_EQ(2u, regressedSnapshot.InvalidEvidenceEventCount);
}

TEST(VeAnalyzeFuelEvents, GenerationChangesInvalidateHistoryAndRegressionFailsClosed) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.make(0, 1.0f, 0, 0, 0, 1, 1, true, false, 1));
	ASSERT_TRUE(fixture.Collector.snapshot(timeUs(0)).Ready);

	feed(fixture, fixture.make(100 * Millisecond, 1.0f, 0, 0, 0, 1, 1, true, false, 2));
	int64_t nowUs = 100 * Millisecond;
	feedNormalUntil(fixture, nowUs, 1500 * Millisecond, 2);

	auto pending = fixture.Collector.snapshot(timeUs(1599 * Millisecond));
	EXPECT_FALSE(pending.Ready);
	EXPECT_TRUE(pending.hasReason(FuelEventReason::ResetRecovery));
	EXPECT_EQ(2u, pending.PublicationGeneration);

	auto recovered = fixture.Collector.snapshot(timeUs(1600 * Millisecond));
	EXPECT_TRUE(recovered.Ready);

	auto regressedGeneration = fixture.normal(1700 * Millisecond);
	regressedGeneration.PublicationGeneration = 1;
	EXPECT_FALSE(fixture.Collector.observeEvent(regressedGeneration));
	auto regressed = fixture.Collector.snapshot(timeUs(1700 * Millisecond));
	EXPECT_FALSE(regressed.Ready);
	EXPECT_TRUE(regressed.hasReason(FuelEventReason::RegressedGeneration));
	EXPECT_EQ(2u, regressed.PublicationGeneration);

	fixture.Collector.syncReset(timeUs(1700 * Millisecond));
	auto reset = fixture.Collector.snapshot(timeUs(1700 * Millisecond));
	EXPECT_FALSE(reset.Ready);
	EXPECT_EQ(0, reset.ObservedBankMask);
	EXPECT_TRUE(reset.hasReason(FuelEventReason::ResetRecovery));
	EXPECT_TRUE(reset.hasReason(FuelEventReason::NoFreshEvent));
}

TEST(VeAnalyzeFuelEvents, MissingObservationsBecomeStaleAndGapNeedsRecovery) {
	FuelEventsFixture fixture;
	feed(fixture, fixture.normal(0));
	EXPECT_TRUE(fixture.Collector.snapshot(timeUs(250 * Millisecond)).Ready);

	auto stale = fixture.Collector.snapshot(timeUs(251 * Millisecond));
	EXPECT_FALSE(stale.Ready);
	EXPECT_TRUE(stale.hasReason(FuelEventReason::StaleEvent));

	feed(fixture, fixture.normal(500 * Millisecond));
	auto resumed = fixture.Collector.snapshot(timeUs(500 * Millisecond));
	EXPECT_TRUE(resumed.LastEvent.Fresh);
	EXPECT_FALSE(resumed.Ready);
	EXPECT_TRUE(resumed.hasReason(FuelEventReason::StaleEvent));
}

TEST(VeAnalyzeFuelEvents, InvalidPolicyFailsClosed) {
	VeAnalyzeFuelEventPolicy policy;
	policy.AeLimitPercent = NAN;
	VeAnalyzeFuelEvents collector(policy);

	EXPECT_FALSE(collector.isValidConfiguration());
	auto event = FuelEventsFixture{}.make(0, 1.0f);
	EXPECT_FALSE(collector.observeEvent(event));
	EXPECT_TRUE(collector.snapshot(timeUs(0)).hasReason(FuelEventReason::InvalidConfiguration));
}
