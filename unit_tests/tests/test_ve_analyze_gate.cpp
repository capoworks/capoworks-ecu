/**
 * @file test_ve_analyze_gate.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"
#include "ve_analyze_gate.h"

#include <cmath>
#include <iterator>
#include <limits>

namespace {
constexpr int64_t Millisecond = 1000;
constexpr int64_t FastPeriod = 4 * Millisecond;

efitick_t timeUs(int64_t microseconds) {
	return efitick_t{US2NT(microseconds).count()};
}

VeAnalyzeGateConfig testPolicy() {
	VeAnalyzeGateConfig config;
	config.QuietHoldSeconds = 0.25f;
	config.CutRecoverySeconds = 0.40f;
	return config;
}

class GateFixture {
public:
	explicit GateFixture(VeAnalyzeGateConfig config = testPolicy())
		: Gate(config) {}

	bool feed(int64_t nowUs, float tps, bool valid = true, bool fresh = true) {
		return Gate.updateTps(timeUs(nowUs), tps, valid, fresh);
	}

	void warm(int64_t& nowUs, float tps = 20.0f, int64_t durationUs = 260 * Millisecond) {
		const int64_t endUs = nowUs + durationUs;
		while (nowUs <= endUs) {
			EXPECT_TRUE(feed(nowUs, tps));
			nowUs += FastPeriod;
		}
	}

	void feedUntil(int64_t& nowUs, int64_t endUs, float tps) {
		nowUs += FastPeriod;
		while (nowUs < endUs) {
			EXPECT_TRUE(feed(nowUs, tps));
			nowUs += FastPeriod;
		}
	}

	VeAnalyzeGate Gate;
};

void expectMovementRecovery(const VeAnalyzeGateSnapshot& snapshot) {
	EXPECT_FALSE(snapshot.Ready);
	EXPECT_TRUE(snapshot.hasReason(VeAnalyzeGateReason::TpsMovementRecovery));
}
} // namespace

TEST(VeAnalyzeGate, StationaryJitteredSamplesBecomeReadyWithoutClampingTps) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));

	constexpr int64_t jitter[] = {3700, 4300, 3900, 4100, 4000, 3600, 4400};
	int64_t nowUs = 0;
	for (size_t index = 0; index < 80; index++) {
		EXPECT_TRUE(fixture.feed(nowUs, 140.0f));
		nowUs += jitter[index % std::size(jitter)];
	}

	auto snapshot = fixture.Gate.snapshot(timeUs(nowUs));
	EXPECT_TRUE(snapshot.TpsValid);
	EXPECT_TRUE(snapshot.RateValid);
	EXPECT_TRUE(snapshot.Ready);
	EXPECT_FLOAT_EQ(140.0f, snapshot.TpsPercent);
	EXPECT_FLOAT_EQ(0, snapshot.AbsTpsRatePercentPerSecond);
	EXPECT_FALSE(snapshot.hasReason(VeAnalyzeGateReason::TpsMovementRecovery));
}

TEST(VeAnalyzeGate, DetectsBothOpeningAndClosingWithoutKnowingAccelEnrichment) {
	for (const float endTps : {30.0f, 10.0f}) {
		GateFixture fixture;
		fixture.Gate.reset(timeUs(0));
		int64_t nowUs = 0;
		fixture.warm(nowUs);

		const int64_t movementUs = nowUs;
		EXPECT_TRUE(fixture.feed(movementUs, endTps));
		auto snapshot = fixture.Gate.snapshot(timeUs(movementUs));
		expectMovementRecovery(snapshot);
		EXPECT_GT(snapshot.AbsTpsRatePercentPerSecond, 5.0f);

		const int64_t recoveredUs = movementUs + 250 * Millisecond;
		fixture.feedUntil(nowUs, recoveredUs, endTps);
		snapshot = fixture.Gate.snapshot(timeUs(recoveredUs));
		EXPECT_TRUE(snapshot.Ready);
	}
}

TEST(VeAnalyzeGate, QuickOutAndBackSurvivesTwentyHertzTelemetryPolling) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);
	fixture.feed(nowUs, 25.0f);
	nowUs += FastPeriod;
	EXPECT_TRUE(fixture.feed(nowUs, 20.0f));

	auto firstTelemetry = fixture.Gate.snapshot(timeUs(nowUs + 20 * Millisecond));
	expectMovementRecovery(firstTelemetry);
	const auto movementAge = firstTelemetry.MillisecondsSinceMovement;

	EXPECT_FALSE(fixture.feed(nowUs + 40 * Millisecond, 20.0f, true, false));
	auto secondTelemetry = fixture.Gate.snapshot(timeUs(nowUs + 40 * Millisecond));
	expectMovementRecovery(secondTelemetry);
	EXPECT_GT(secondTelemetry.MillisecondsSinceMovement, movementAge);
	EXPECT_LT(secondTelemetry.HoldRemainingMilliseconds, firstTelemetry.HoldRemainingMilliseconds);
}

TEST(VeAnalyzeGate, InvalidAndNanTpsFailClosedAndRequireFreshRecovery) {
	for (const float invalidValue : {NAN, INFINITY}) {
		GateFixture fixture;
		fixture.Gate.reset(timeUs(0));
		int64_t nowUs = 0;
		fixture.warm(nowUs);
		ASSERT_TRUE(fixture.Gate.snapshot(timeUs(nowUs)).Ready);

		EXPECT_FALSE(fixture.feed(nowUs, invalidValue));
		auto invalid = fixture.Gate.snapshot(timeUs(nowUs));
		EXPECT_FALSE(invalid.TpsValid);
		EXPECT_FALSE(invalid.Ready);
		EXPECT_TRUE(invalid.hasReason(VeAnalyzeGateReason::InvalidTps));

		nowUs += FastPeriod;
		fixture.warm(nowUs);
		auto recovered = fixture.Gate.snapshot(timeUs(nowUs));
		EXPECT_TRUE(recovered.TpsValid);
		EXPECT_TRUE(recovered.RateValid);
		EXPECT_TRUE(recovered.Ready);
	}

	GateFixture sensorInvalid;
	sensorInvalid.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	sensorInvalid.warm(nowUs);
	EXPECT_FALSE(sensorInvalid.feed(nowUs, 20, false, true));
	auto invalid = sensorInvalid.Gate.snapshot(timeUs(nowUs));
	EXPECT_FALSE(invalid.TpsValid);
	EXPECT_TRUE(invalid.hasReason(VeAnalyzeGateReason::InvalidTps));
}

TEST(VeAnalyzeGate, CachedValidSampleBecomesStaleAndGapReseedsHistory) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);
	ASSERT_TRUE(fixture.Gate.snapshot(timeUs(nowUs)).Ready);

	const int64_t staleUs = nowUs + 60 * Millisecond;
	EXPECT_FALSE(fixture.feed(staleUs, 20.0f, true, false));
	auto stale = fixture.Gate.snapshot(timeUs(staleUs));
	EXPECT_FALSE(stale.TpsValid);
	EXPECT_FALSE(stale.RateValid);
	EXPECT_TRUE(stale.hasReason(VeAnalyzeGateReason::StaleTps));

	const int64_t reseedUs = staleUs + FastPeriod;
	EXPECT_TRUE(fixture.feed(reseedUs, 20.0f));
	auto reseeded = fixture.Gate.snapshot(timeUs(reseedUs));
	EXPECT_FALSE(reseeded.RateValid);
	EXPECT_TRUE(reseeded.hasReason(VeAnalyzeGateReason::SampleGap));

	nowUs = reseedUs + FastPeriod;
	while (nowUs <= reseedUs + 250 * Millisecond) {
		EXPECT_TRUE(fixture.feed(nowUs, 20.0f));
		nowUs += FastPeriod;
	}
	EXPECT_TRUE(fixture.Gate.snapshot(timeUs(reseedUs + 250 * Millisecond)).Ready);
}

TEST(VeAnalyzeGate, RateEqualToThresholdDoesNotTriggerStrictGreaterThanGate) {
	VeAnalyzeGateConfig config = testPolicy();
	config.QuietHoldSeconds = 0.22f;
	config.QuickMovementThresholdPercent = 100.0f;
	GateFixture fixture(config);
	fixture.Gate.reset(timeUs(0));

	int64_t nowUs = 0;
	while (nowUs < 200 * Millisecond) {
		EXPECT_TRUE(fixture.feed(nowUs, 0));
		nowUs += FastPeriod;
	}
	EXPECT_TRUE(fixture.feed(200 * Millisecond, 1.0f));

	auto snapshot = fixture.Gate.snapshot(timeUs(220 * Millisecond));
	EXPECT_FLOAT_EQ(5.0f, snapshot.AbsTpsRatePercentPerSecond);
	EXPECT_TRUE(snapshot.RateValid);
	EXPECT_TRUE(snapshot.Ready);
	EXPECT_FALSE(snapshot.hasReason(VeAnalyzeGateReason::TpsMovementRecovery));
}

TEST(VeAnalyzeGate, DuplicateAndRegressedTimestampsCannotManufactureSamples) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);

	fixture.feed(nowUs, 25.0f);
	const int64_t movementUs = nowUs;
	auto before = fixture.Gate.snapshot(timeUs(nowUs));
	expectMovementRecovery(before);

	EXPECT_FALSE(fixture.feed(nowUs, 20.0f));
	auto duplicate = fixture.Gate.snapshot(timeUs(nowUs));
	EXPECT_TRUE(duplicate.hasReason(VeAnalyzeGateReason::DuplicateTimestamp));
	EXPECT_EQ(before.HoldRemainingMilliseconds, duplicate.HoldRemainingMilliseconds);
	EXPECT_EQ(before.MillisecondsSinceMovement, duplicate.MillisecondsSinceMovement);

	EXPECT_TRUE(fixture.feed(movementUs - FastPeriod, 20.0f));
	auto regressed = fixture.Gate.snapshot(timeUs(movementUs - FastPeriod));
	EXPECT_TRUE(regressed.hasReason(VeAnalyzeGateReason::RegressedTimestamp));
	EXPECT_FALSE(regressed.Ready);
	EXPECT_FALSE(regressed.RateValid);
	EXPECT_TRUE(regressed.hasReason(VeAnalyzeGateReason::InsufficientHistory));
}

TEST(VeAnalyzeGate, MeaningfulGapFailsClosedAndRequiresWindowAndHold) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);
	ASSERT_TRUE(fixture.Gate.snapshot(timeUs(nowUs)).Ready);

	const int64_t resumedUs = nowUs + 100 * Millisecond;
	EXPECT_TRUE(fixture.feed(resumedUs, 20.0f));
	auto gap = fixture.Gate.snapshot(timeUs(resumedUs));
	EXPECT_FALSE(gap.RateValid);
	EXPECT_FALSE(gap.Ready);
	EXPECT_TRUE(gap.hasReason(VeAnalyzeGateReason::SampleGap));

	nowUs = resumedUs + FastPeriod;
	while (nowUs < resumedUs + 250 * Millisecond) {
		EXPECT_TRUE(fixture.feed(nowUs, 20.0f));
		nowUs += FastPeriod;
	}
	auto recovered = fixture.Gate.snapshot(timeUs(resumedUs + 250 * Millisecond));
	EXPECT_TRUE(recovered.Ready);
}

TEST(VeAnalyzeGate, SustainedTransientRenewsHoldAndCountsDownAfterItEnds) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);

	for (int iteration = 0; iteration < 75; iteration++) {
		fixture.Gate.observeTransient(timeUs(nowUs), VeAnalyzeTransient::FuelCorrection, true);
		EXPECT_TRUE(fixture.feed(nowUs, 20.0f));
		nowUs += FastPeriod;
	}
	fixture.Gate.observeTransient(timeUs(nowUs), VeAnalyzeTransient::FuelCorrection, false);
	EXPECT_TRUE(fixture.feed(nowUs, 20.0f));

	auto ended = fixture.Gate.snapshot(timeUs(nowUs));
	EXPECT_FALSE(ended.Ready);
	EXPECT_TRUE(ended.hasReason(VeAnalyzeGateReason::FuelCorrectionRecovery));
	EXPECT_GE(ended.HoldRemainingMilliseconds, 245u);

	const int64_t recoveredUs = nowUs + 250 * Millisecond;
	fixture.feedUntil(nowUs, recoveredUs, 20.0f);
	EXPECT_FALSE(fixture.Gate.snapshot(timeUs(recoveredUs - 1)).Ready);
	EXPECT_TRUE(fixture.Gate.snapshot(timeUs(recoveredUs)).Ready);
}

TEST(VeAnalyzeGate, LaterNormalTransientCannotShortenCutRecovery) {
	GateFixture fixture(VeAnalyzeGateConfig{});
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);

	const int64_t cutStartUs = nowUs;
	fixture.Gate.observeTransient(timeUs(cutStartUs), VeAnalyzeTransient::FuelCut, true);
	fixture.feedUntil(nowUs, cutStartUs + 140 * Millisecond, 20.0f);
	fixture.Gate.observeTransient(timeUs(cutStartUs + 140 * Millisecond), VeAnalyzeTransient::FuelCut, true);
	nowUs = cutStartUs + 140 * Millisecond;
	fixture.Gate.observeTransient(timeUs(nowUs), VeAnalyzeTransient::FuelCut, false);

	const int64_t normalStartUs = nowUs + 10 * Millisecond;
	fixture.Gate.observeTransient(timeUs(normalStartUs), VeAnalyzeTransient::FuelCorrection, true);
	fixture.Gate.observeTransient(timeUs(normalStartUs + 5 * Millisecond), VeAnalyzeTransient::FuelCorrection, false);

	const int64_t cutRecoveredUs = cutStartUs + 140 * Millisecond + 2000 * Millisecond;
	fixture.feedUntil(nowUs, cutRecoveredUs, 20.0f);
	auto cutHeld = fixture.Gate.snapshot(timeUs(cutRecoveredUs - 1));
	EXPECT_FALSE(cutHeld.Ready);
	EXPECT_TRUE(cutHeld.hasReason(VeAnalyzeGateReason::FuelCutRecovery));

	EXPECT_TRUE(fixture.Gate.snapshot(timeUs(cutRecoveredUs)).Ready);
}

TEST(VeAnalyzeGate, CalibrationRecoveryPersistsAcrossTelemetryPolls) {
	GateFixture fixture;
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);

	const int64_t eventUs = nowUs;
	fixture.Gate.observeTransient(timeUs(eventUs), VeAnalyzeTransient::Calibration, true);
	fixture.Gate.observeTransient(timeUs(eventUs), VeAnalyzeTransient::Calibration, false);

	auto firstPoll = fixture.Gate.snapshot(timeUs(eventUs + 20 * Millisecond));
	EXPECT_TRUE(firstPoll.hasReason(VeAnalyzeGateReason::CalibrationRecovery));
	EXPECT_EQ(20u, firstPoll.MillisecondsSinceCorrection);

	auto secondPoll = fixture.Gate.snapshot(timeUs(eventUs + 40 * Millisecond));
	EXPECT_TRUE(secondPoll.hasReason(VeAnalyzeGateReason::CalibrationRecovery));
	EXPECT_EQ(40u, secondPoll.MillisecondsSinceCorrection);
	EXPECT_LT(secondPoll.HoldRemainingMilliseconds, firstPoll.HoldRemainingMilliseconds);

	const int64_t recoveredUs = eventUs + 250 * Millisecond;
	fixture.feedUntil(nowUs, recoveredUs, 20.0f);
	EXPECT_TRUE(fixture.Gate.snapshot(timeUs(recoveredUs)).Ready);
}

TEST(VeAnalyzeGate, LambdaDelayExtendsHoldBeyondWindow) {
	VeAnalyzeGateConfig config = testPolicy();
	config.QuietHoldSeconds = 0.25f;
	config.MaxLambdaDelaySeconds = 0.50f;
	GateFixture fixture(config);
	fixture.Gate.reset(timeUs(0));
	int64_t nowUs = 0;
	fixture.warm(nowUs);
	fixture.Gate.observeTransient(timeUs(nowUs), VeAnalyzeTransient::FuelCorrection, true);
	fixture.Gate.observeTransient(timeUs(nowUs), VeAnalyzeTransient::FuelCorrection, false);

	const int64_t recoveredUs = nowUs + 700 * Millisecond;
	fixture.feedUntil(nowUs, recoveredUs, 20.0f);
	EXPECT_FALSE(fixture.Gate.snapshot(timeUs(recoveredUs - 1)).Ready);
	EXPECT_TRUE(fixture.Gate.snapshot(timeUs(recoveredUs)).Ready);
}

TEST(VeAnalyzeGate, InvalidPolicyFailsClosed) {
	VeAnalyzeGateConfig config;
	config.RateThresholdPercentPerSecond = 0;
	VeAnalyzeGate gate(config);
	EXPECT_FALSE(gate.isValidConfiguration());
	EXPECT_FALSE(gate.updateTps(timeUs(0), 20, true, true));

	auto snapshot = gate.snapshot(timeUs(1000));
	EXPECT_FALSE(snapshot.Ready);
	EXPECT_TRUE(snapshot.hasReason(VeAnalyzeGateReason::InvalidConfiguration));

	config.RateThresholdPercentPerSecond = 5;
	config.MaxLambdaDelaySeconds = NAN;
	EXPECT_FALSE(gate.configure(config));
	EXPECT_TRUE(gate.snapshot(timeUs(2000)).hasReason(VeAnalyzeGateReason::InvalidConfiguration));
}

TEST(VeAnalyzeGate, AgesSaturateAndResetRecoversAtHighUptime) {
	VeAnalyzeGate gate(testPolicy());
	constexpr int64_t highUptimeUs = 200LL * 365 * 24 * 60 * 60 * 1000000;
	gate.reset(timeUs(0));
	gate.updateTps(timeUs(0), 20, true, true);
	gate.updateTps(timeUs(FastPeriod), 30, true, true);

	auto oldMovement = gate.snapshot(timeUs(highUptimeUs));
	EXPECT_EQ(std::numeric_limits<uint32_t>::max(), oldMovement.MillisecondsSinceMovement);
	EXPECT_TRUE(oldMovement.hasReason(VeAnalyzeGateReason::StaleTps));

	const int64_t endUs = highUptimeUs + 300 * Millisecond;
	gate.reset(timeUs(highUptimeUs));
	int64_t nowUs = highUptimeUs;
	while (nowUs <= endUs) {
		EXPECT_TRUE(gate.updateTps(timeUs(nowUs), 20, true, true));
		nowUs += FastPeriod;
	}
	EXPECT_TRUE(gate.snapshot(timeUs(endUs)).Ready);
}
