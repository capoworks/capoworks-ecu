#include "pch.h"
#include "trigger_universal.h"
#include "spark_logic.h"
#include "defaults.h"

namespace {
struct ProfileTrace {
	TriggerWaveform shape;
	TriggerFormDetails details;
	IgnitionCycleProfile predictor;
	uint32_t timestamps[PWM_PHASE_MAX_COUNT]{};
	efitick_t now{1000000LL * US_TO_NT_MULTIPLIER};
	unsigned j = 0;
	unsigned start = 0;
	EnginePhaseInfo phase{};

	explicit ProfileTrace(int teeth = 36, unsigned startingTooth = 0)
		: start(startingTooth) {
		initializeSkippedToothTrigger(&shape, teeth, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
		for (int i = 0; i < 2 * (teeth - 2); i++) {
			float angle = (i / (teeth - 2)) * 360.0f + (i % (teeth - 2)) * (360.0f / teeth);
			details.eventAngles[2 * i] = angle;
			details.eventAngles[2 * i + 1] = angle;
		}
		predictor.configure(shape, details);
	}
	unsigned index() const {
		return ((j + start) % (shape.getLength() / 2)) * 2;
	}
	float nextSpan() const {
		auto i = index();
		float next = i + 2 == shape.getLength() ? 720.0f : details.eventAngles[i + 2];
		return next - details.eventAngles[i];
	}
	void accept() {
		auto i = index();
		auto next = i + 2 == shape.getLength() ? 0 : i + 2;
		phase = {
				now,
				{details.eventAngles[i]},
				{details.eventAngles[next]},
				{details.eventAngles[i]},
				{details.eventAngles[next]}};
		predictor.update(timestamps, i, phase);
		timestamps[i] = now;
		j++;
	}
};
void dummy(void*) {}
} // namespace

TEST(IgnitionCycleProfile, DefaultAndResetAreLegacy) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EXPECT_FALSE(engineConfiguration->ignitionCycleProfile);
	engineConfiguration->ignitionCycleProfile = true;
	setDefaultIgnition();
	EXPECT_FALSE(engineConfiguration->ignitionCycleProfile);
}

TEST(IgnitionCycleProfile, UniformReadinessAndMissingToothGeometry) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (int teeth : {36, 60}) {
		for (int rpm : {1200, 5000, 10000}) {
			for (unsigned start : {0u, 13u}) {
				ProfileTrace t(teeth, start);
				ASSERT_EQ(t.predictor.toothCount(), 2u * (teeth - 2));
				for (unsigned j = 0; j < 4u * t.predictor.toothCount(); j++) {
					float span = t.nextSpan();
					t.accept();
					auto predicted = t.predictor.getDelayNt(t.phase, span / 2);
					if (j < t.predictor.toothCount() + 3u) {
						EXPECT_FALSE(predicted) << j;
					} else {
						ASSERT_TRUE(predicted) << j;
						EXPECT_NEAR(predicted.Value, USF2NT(60000000.0f / rpm / 360 * span / 2), 2);
						EXPECT_TRUE(t.predictor.getDelayNt(t.phase, 0));
						EXPECT_FALSE(t.predictor.getDelayNt(t.phase, span));
					}
					t.now += efidur_t{static_cast<int32_t>(USF2NT(60000000.0f / rpm / 360 * span))};
				}
			}
		}
	}
}

TEST(IgnitionCycleProfile, NonuniformExactFormulaAndClockWrap) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	ProfileTrace t;
	t.now = efitick_t{0}; // zero is a trusted timestamp, not a validity sentinel
	std::vector<uint32_t> times;
	for (unsigned j = 0; j < 500; j++) {
		float span = t.nextSpan();
		times.push_back(t.now);
		t.accept();
		if (j >= 71) {
			auto delay = t.predictor.getDelayNt(t.phase, span / 2);
			ASSERT_TRUE(delay);
			float exact = static_cast<float>(times[j - 67] - times[j - 68]) *
						  static_cast<float>(times[j] - times[j - 3]) /
						  static_cast<float>(times[j - 68] - times[j - 71]);
			EXPECT_NEAR(delay.Value * 2, exact, 2);
		}
		// Exactly repeating 720 degree ripple, including both gaps and three-interval windows.
		float duration = USF2NT(span * (150.0f + 40.0f * std::sin((j % 68) * 0.18f)));
		t.now += efidur_t{static_cast<int32_t>(duration)};
	}
	t.predictor.reset();
	t.j = 0;
	t.now = efitick_t{static_cast<int64_t>(UINT32_MAX) - 400000};
	for (unsigned j = 0; j < 150; j++) {
		float span = t.nextSpan();
		t.accept();
		if (j >= 71) {
			ASSERT_TRUE(t.predictor.getDelayNt(t.phase, 1));
		}
		t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
	}
}

TEST(IgnitionCycleProfile, InvalidHistoryAndContext) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (int failure = 0; failure < 7; failure++) {
		ProfileTrace t;
		for (int j = 0; j < 150; j++) {
			float span = t.nextSpan();
			t.accept();
			t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
		}
		ASSERT_TRUE(t.predictor.getDelayNt(t.phase, 1));
		auto wrong = t.phase;
		wrong.timestamp += US2NT(1);
		EXPECT_FALSE(t.predictor.getDelayNt(wrong, 1));
		wrong = t.phase;
		wrong.currentEngPhase.angle += 360;
		EXPECT_FALSE(t.predictor.getDelayNt(wrong, 1));
		EXPECT_FALSE(t.predictor.getDelayNt(t.phase, NAN));
		EXPECT_FALSE(t.predictor.getDelayNt(t.phase, -1));
		switch (failure) {
			case 0:
				t.predictor.reset();
				break;
			case 1:
				t.j++;
				break; // skipped reference
			case 2:
				t.j--;
				t.now = t.phase.timestamp;
				break; // repeated tooth and zero time
			case 3:
				t.now += US2NT(50000000);
				break; // longer than an entire uint32 wrap
			case 4:
				t.timestamps[t.index()] = t.now;
				break; // zero cycle elapsed
			case 5:
				t.details.eventAngles[3] += 1;
				t.predictor.configure(t.shape, t.details);
				break;
			case 6:
				t.shape.needSecondTriggerInput = true;
				t.predictor.configure(t.shape, t.details);
				break;
		}
		t.accept();
		EXPECT_FALSE(t.predictor.getDelayNt(t.phase, 1)) << failure;
		if (failure < 4) {
			for (int j = 0; j < 71; j++) {
				t.now += US2NT(1000);
				t.accept();
				if (j < 70) {
					EXPECT_FALSE(t.predictor.getDelayNt(t.phase, 1));
				}
			}
			EXPECT_TRUE(t.predictor.getDelayNt(t.phase, 1));
		}
	}
}

TEST(IgnitionCycleProfile, ImmediateQueuedAndNonIgnitionPolicy) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->ignitionCycleProfile = true;
	engine->rpmCalculator.setRpmValue(1200);
	auto& tc = engine->triggerCentral;
	ProfileTrace t;
	for (int j = 0; j < 72; j++) {
		float span = t.nextSpan();
		t.accept();
		t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
	}
	tc.instantRpm.ignitionProfile = t.predictor;
	// Mapping is an identity in this fixture.
	tc.triggerShape.tdcPosition = -engineConfiguration->globalTriggerAngleOffset;
	AngleBasedEvent immediate, legacy, queued;
	auto scheduler = engine->module<TriggerScheduler>();
	EngPhase target = t.phase.currentEngPhase + 5;
	ASSERT_TRUE(scheduler->scheduleOrQueue(&immediate, target, dummy, t.phase, AngleTimingPolicy::Ignition));
	ASSERT_TRUE(scheduler->scheduleOrQueue(&legacy, target, dummy, t.phase));
	EXPECT_EQ(immediate.scheduling.momentX, NT2US(t.phase.timestamp) + 500);
	EXPECT_NEAR(legacy.scheduling.momentX, NT2US(t.phase.timestamp) + 5 * engine->rpmCalculator.oneDegreeUs, 1);
	// Keep a distant event as an angle until its final interval.
	scheduler->schedule(&queued, EngPhase{350}, dummy, AngleTimingPolicy::Ignition);
	auto early = t.phase;
	early.currentTrgPhase = {100};
	early.nextTrgPhase = {110};
	scheduler->onEnginePhase(1200, early);
	EXPECT_EQ(scheduler->getElementAtIndexForUnitTest(0), &queued);
	// Move through actual sequence until the 330 -> 360 gap.
	while (t.phase.currentTrgPhase.angle != 330) {
		float span = t.nextSpan();
		t.accept();
		t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
	}
	tc.instantRpm.ignitionProfile = t.predictor;
	scheduler->onEnginePhase(1200, t.phase);
	EXPECT_EQ(queued.scheduling.momentX, NT2US(t.phase.timestamp) + 2000);
	tc.instantRpm.spinningEventIndex = 0;
	tc.instantRpm.movePreSynchTimestamps();
	EXPECT_TRUE(tc.instantRpm.ignitionProfile.getDelayNt(t.phase, 0));
	tc.instantRpm.spinningEventIndex = 1;
	tc.instantRpm.movePreSynchTimestamps();
	EXPECT_FALSE(tc.instantRpm.ignitionProfile.getDelayNt(t.phase, 0));
	eth.clearQueue();
}

TEST(IgnitionCycleProfile, PrimaryAndCamDecoderLifecycle) {
	for (int teeth : {36, 60}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		engineConfiguration->ignitionCycleProfile = true;
		engineConfiguration->isIgnitionEnabled = false;
		engineConfiguration->isInjectionEnabled = false;
		setCrankOperationMode();
		engineConfiguration->skippedWheelOnCam = false;
		engineConfiguration->trigger.customTotalToothCount = teeth;
		engineConfiguration->trigger.customSkippedToothCount = 2;
		engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH;
		engineConfiguration->camInputs[0] = Gpio::A10;
		engineConfiguration->vvtOffsets[0] = 0;
		engineConfiguration->globalTriggerAngleOffset = 0;
		eth.setTriggerType(trigger_type_e::TT_TOOTHED_WHEEL);
		auto& tc = engine->triggerCentral;
		ASSERT_EQ(tc.instantRpm.ignitionProfile.toothCount(), 2u * (teeth - 2));
		setTimeNowUs(1000000);
		unsigned valid = 0;
		for (unsigned j = 0; j < unsigned(12 * (teeth - 2)); j++) {
			float span = j % (teeth - 2) == 0 ? 3 * 360.0f / teeth : 360.0f / teeth;
			// Cam edge halfway between teeth, every 720 degrees, away from crank sync.
			if (j % (2 * (teeth - 2)) == 10) {
				eth.moveTimeForwardUs(span * 100 / 2);
				hwHandleVvtCamSignal(true, getTimeNowNt(), 0);
				eth.moveTimeForwardUs(span * 100 / 2);
			} else {
				eth.moveTimeForwardUs(span * 100);
			}
			eth.firePrimaryTriggerRise();
			if (tc.triggerState.getShaftSynchronized()) {
				const auto index = tc.triggerState.getCurrentIndex() +
								   (tc.triggerState.getCrankSynchronizationCounter() % 2) * tc.triggerShape.getSize();
				const auto next = index + 2 == tc.engineCycleEventCount ? 0 : index + 2;
				TrgPhase current{tc.triggerFormDetails.eventAngles[index]};
				TrgPhase following{tc.triggerFormDetails.eventAngles[next]};
				EnginePhaseInfo info{
						getTimeNowNt(), current, following, tc.toEngPhase(current), tc.toEngPhase(following)};
				if (auto delay = tc.instantRpm.ignitionProfile.getDelayNt(info, 1)) {
					valid++;
					EXPECT_NEAR(delay.Value, USF2NT(100.0f), USF2NT(0.5f));
				}
			}
		}
		EXPECT_TRUE(tc.triggerState.hasSynchronizedPhase());
		EXPECT_GT(valid, 50u);
		tc.syncAndReport(2, 1 - (tc.triggerState.getCrankSynchronizationCounter() % 2));
		// Phase shift invalidates a formerly ready cache immediately.
		EnginePhaseInfo old{getTimeNowNt(), {0}, {10}, {0}, {10}};
		EXPECT_FALSE(tc.instantRpm.ignitionProfile.getDelayNt(old, 1));
		engine->OnTriggerSynchronizationLost();
		EXPECT_FALSE(tc.instantRpm.ignitionProfile.getDelayNt(old, 1));
	}
}

TEST(IgnitionCycleProfile, ChargeMinimumDwellAndTriggerLossProtection) {
	for (bool queued : {false, true}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->ignitionCycleProfile = true;
		engineConfiguration->isIgnitionEnabled = true;
		engineConfiguration->isInjectionEnabled = false;
		engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
		engine->rpmCalculator.setRpmValue(1200);
		engine->ignitionState.dwellAngle = 1;
		engine->ignitionState.sparkDwell = 2; // 2ms nominal: 1.6ms minimum, 3ms over dwell
		engine->cylinders[0].setIgnitionTimingBtdc(queued ? -100 : -5);
		initializeIgnitionActions();
		ProfileTrace t;
		// Warm the predictor at 0 degrees with 100us/degree, while RPM conversion is 139us/degree.
		for (int j = 0; j < 137; j++) {
			float span = t.nextSpan();
			t.accept();
			t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
		}
		ASSERT_EQ(t.phase.currentEngPhase.angle, 0);
		auto& tc = engine->triggerCentral;
		tc.triggerShape.tdcPosition = -engineConfiguration->globalTriggerAngleOffset;
		tc.instantRpm.ignitionProfile = t.predictor;
		auto& event = engine->ignitionEvents.elements[0];
		event.dwellAngle = 2;
		event.sparkDwell = 2;
		setTimeNowUs(NT2US(t.phase.timestamp));
		onTriggerEventSparkLogic(t.phase);
		EXPECT_EQ(event.dwellStartTimer.momentX, getTimeNowUs() + 200);
		unsigned charges = 0, discharges = 0;
		bool over = false;
		engine->onIgnitionEvent = [&](IgnitionContext ctx, bool charge) {
			if (charge) {
				charges++;
			} else {
				discharges++;
				over = ctx.isOverdwellProtect;
				EXPECT_TRUE(enginePins.coils[0].getLogicValue());
				EXPECT_NEAR(event.actualDwellTimer.getElapsedSeconds(getTimeNowNt()) * 1000, queued ? 3 : 1.6f, 0.02f);
			}
		};
		eth.setTimeAndInvokeEventsUs(getTimeNowUs() + 4000);
		EXPECT_EQ(charges, 1u);
		EXPECT_EQ(discharges, 1u);
		EXPECT_EQ(over, queued);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		engine->onIgnitionEvent = {};
		eth.clearQueue();
	}
}

TEST(IgnitionCycleProfile, DisabledAndUnsupportedUseLegacyTiming) {
	for (bool unsupported : {false, true}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		engineConfiguration->ignitionCycleProfile = unsupported;
		engine->rpmCalculator.setRpmValue(1200);
		ProfileTrace t;
		for (int j = 0; j < 137; j++) {
			float span = t.nextSpan();
			t.accept();
			t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f))};
		}
		auto& tc = engine->triggerCentral;
		tc.instantRpm.ignitionProfile = t.predictor;
		if (unsupported) {
			t.shape.useOnlyRisingEdges = false;
			tc.instantRpm.ignitionProfile.configure(t.shape, t.details);
		}
		scheduling_s timer;
		auto fire = scheduleByAngleInPhase(&timer, t.phase, 5, dummy, AngleTimingPolicy::Ignition);
		EXPECT_NEAR(NT2US(fire - t.phase.timestamp), 5 * engine->rpmCalculator.oneDegreeUs, 1);
		engine->scheduler.cancel(&timer);
		// No matching local context: generic and beyond-interval scheduling stays legacy.
		fire = scheduleByAngleInPhase(&timer, t.phase, 50, dummy, AngleTimingPolicy::Ignition);
		EXPECT_NEAR(NT2US(fire - t.phase.timestamp), 50 * engine->rpmCalculator.oneDegreeUs, 1);
		eth.clearQueue();
	}
}

TEST(IgnitionCycleProfile, RejectLargeAdaptationWithoutClamping) {
	for (float scale : {0.4f, 0.5f, 2.0f, 2.5f}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		ProfileTrace t;
		for (int j = 0; j < 140; j++) {
			float span = t.nextSpan();
			t.accept();
			if (j == 139) {
				EXPECT_EQ(bool(t.predictor.getDelayNt(t.phase, 1)), scale >= 0.5f && scale <= 2.0f);
			}
			t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f * (j >= 136 ? scale : 1.0f)))};
		}
	}
}
