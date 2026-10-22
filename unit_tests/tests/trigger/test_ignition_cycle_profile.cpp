#include "pch.h"
#include "trigger_universal.h"
#include "spark_logic.h"
#include "defaults.h"
#include "ignition_retarget.h"

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

TEST(IgnitionCycleProfile, RestoringDefaultsDisablesBothExperimentalSettings) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	EXPECT_FALSE(engineConfiguration->ignitionCycleProfile);
	EXPECT_FALSE(engineConfiguration->ignitionTimeBudget);
	engineConfiguration->ignitionCycleProfile = true;
	engineConfiguration->ignitionTimeBudget = true;
	setDefaultIgnition();
	EXPECT_FALSE(engineConfiguration->ignitionCycleProfile);
	EXPECT_FALSE(engineConfiguration->ignitionTimeBudget);
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

TEST(IgnitionCycleProfile, CoherentLocalFallbackKeepsHistoricalAdaptationBounds) {
	for (float scale : {0.4f, 0.5f, 2.0f, 2.5f}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		ProfileTrace t;
		for (int j = 0; j < 140; j++) {
			float span = t.nextSpan();
			t.accept();
			if (j == 139) {
				auto local = t.predictor.getDelayNt(t.phase, 1);
				ASSERT_TRUE(local);
				EXPECT_NEAR(USF2NT(100.0f * scale), local.Value, 2);
				EXPECT_EQ(bool(t.predictor.getTimeToAngleNt(t.phase, 90)), scale >= 0.5f && scale <= 2.0f);
			}
			t.now += efidur_t{static_cast<int32_t>(USF2NT(span * 100.0f * (j >= 136 ? scale : 1.0f)))};
		}
	}
}

TEST(IgnitionCycleProfile, BoundedTimeBudgetUsesUnmodifiedFutureHistoricalSlots) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (int teeth : {36, 60}) {
		ProfileTrace t(teeth);
		for (unsigned j = 0; j < 4u * t.predictor.toothCount(); j++) {
			const float span = t.nextSpan();
			t.accept();
			if (j >= t.predictor.toothCount() + 3u) {
				for (float angle : {0.0f, 1.0f, 17.0f, 90.0f, 180.0f}) {
					auto budget = t.predictor.getTimeToAngleNt(t.phase, angle);
					ASSERT_TRUE(budget) << teeth << ":" << j << ":" << angle;
					EXPECT_NEAR(USF2NT(angle * 100.0f), budget.Value, 5);
				}
				EXPECT_FALSE(t.predictor.getTimeToAngleNt(t.phase, 180.01f));
				auto stale = t.phase;
				stale.timestamp += US2NT(1);
				EXPECT_FALSE(t.predictor.getTimeToAngleNt(stale, 20));
			}
			t.now += static_cast<int32_t>(USF2NT(span * 100.0f));
		}
		t.predictor.reset();
		EXPECT_FALSE(t.predictor.getTimeToAngleNt(t.phase, 20));
	}
}

TEST(IgnitionTimeBudget, LiveTargetArmsOncePerTdcOccurrence) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->ignitionTimeBudget = true;
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->minimumIgnitionTiming = -100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	auto& tc = engine->triggerCentral;
	initializeSkippedToothTrigger(&tc.triggerShape, 36, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	for (int i = 0; i < 68; i++) {
		tc.triggerFormDetails.eventAngles[2 * i] = (i / 34) * 360 + (i % 34) * 10;
		tc.triggerFormDetails.eventAngles[2 * i + 1] = tc.triggerFormDetails.eventAngles[2 * i];
	}
	tc.instantRpm.ignitionProfile.configure(tc.triggerShape, tc.triggerFormDetails);
	tc.triggerState.setNeedsDisambiguation(true, true);
	tc.triggerState.syncEnginePhase(2, 0, 720);
	onTriggerEventSparkLogic({getTimeNowNt(), 0, 10, 0, 10});
	EXPECT_EQ(0u, engine->engineState.sparkCounter);
	// Change advance after preparation: planning uses the live target 35 ATDC.
	engine->cylinders[0].setIgnitionTimingBtdc(-35);
	onTriggerEventSparkLogic({getTimeNowNt(), 20, 30, 20, 30});
	auto& event = engine->ignitionEvents.elements[0];
	ASSERT_EQ(1u, engine->engineState.sparkCounter);
	EXPECT_FLOAT_EQ(35, event.plannedSparkAngle);
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	// A later tune update may retarget inside the immutable physical guard.
	engine->cylinders[0].setIgnitionTimingBtdc(-45);
	const auto deadline = enginePins.coils[0].hardDeadline();
	onTriggerEventSparkLogic({getTimeNowNt(), 20, 30, 20, 30});
	EXPECT_EQ(deadline, enginePins.coils[0].hardDeadline());
	engine->module<TriggerScheduler>()->onEnginePhase(1000, {getTimeNowNt(), 30, 40, 30, 40});
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(deadline, enginePins.coils[0].hardDeadline());
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	onTriggerEventSparkLogic({getTimeNowNt(), 40, 50, 40, 50});
	EXPECT_EQ(1u, engine->engineState.sparkCounter);
}

TEST(IgnitionCycleProfile, CoherentLocalFallbackDoesNotRescueRejectedLongHorizon) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (bool coherent : {false, true}) {
		ProfileTrace t;
		for (unsigned j = 0; j <= 91; j++) {
			float span = t.nextSpan();
			t.accept();
			if (j == 91) {
				break;
			}
			float factor = 1;
			if (j >= 88) {
				factor = coherent ? 3.0f : (j == 89 ? 5.0f : 1.0f);
			}
			t.now += static_cast<uint32_t>(USF2NT(span * 100.0f * factor));
		}
		auto delay = t.predictor.getDelayNt(t.phase, 1);
		EXPECT_EQ(coherent, bool(delay));
		if (delay) {
			EXPECT_NEAR(USF2NT(300), delay.Value, 2);
		}
		EXPECT_FALSE(t.predictor.getTimeToAngleNt(t.phase, 90));
	}
}

TEST(IgnitionTimeBudget, ElectricalDecoderWithMissingAndExtraPulses) {
	// Both RPM consumers read the shared timestamp ring before it is overwritten.
	for (int feedbackMode : {0, 1}) {
		SCOPED_TRACE(::testing::Message() << "rolling RPM feedback mode=" << feedbackMode);
		for (int teeth : {36, 60}) {
			for (int noise : {0, 1, 2}) {
				EngineTestHelper eth(engine_type_e::TEST_ENGINE);
				engineConfiguration->ignitionCycleProfile = true;
				engineConfiguration->idleTimingUseRollingRpm = feedbackMode != 0;
				engineConfiguration->isIgnitionEnabled = true;
				engineConfiguration->ignitionTimeBudget = true;
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
				unsigned discharges = 0;
				engine->onIgnitionEvent = [&](IgnitionContext ctx, bool charging) {
					if (charging) {
						return;
					}
					forEachSetBit(ctx.outputsMask(), [&](size_t index) {
						auto& pin = enginePins.coils[index];
						if (pin.ownedBy(ctx.owner())) {
							discharges++;
							const float ageMs = NT2USF(getTimeNowNt() - pin.firstHigh()) * 0.001f;
							EXPECT_LE(
									ageMs,
									1.5f * engine->ignitionEvents.elements[ctx.eventIndex].occurrenceDwell + 0.02f);
						}
					});
				};
				for (unsigned j = 0; j < unsigned(16 * (teeth - 2)); j++) {
					float span = j % (teeth - 2) == 0 ? 3 * 360.0f / teeth : 360.0f / teeth;
					// Cam edge halfway between teeth, every 720 degrees, away from crank sync.
					if (j % (2 * (teeth - 2)) == 10) {
						eth.moveTimeForwardAndInvokeEventsUs(span * 100 / 2);
						hwHandleVvtCamSignal(true, getTimeNowNt(), 0);
						eth.moveTimeForwardAndInvokeEventsUs(span * 100 / 2);
					} else {
						eth.moveTimeForwardAndInvokeEventsUs(span * 100);
					}
					engine->periodicFastCallback();
					const bool disturbed = j == unsigned(7 * (teeth - 2) + 15);
					if (noise != 1 || !disturbed) {
						eth.firePrimaryTriggerRise();
					}
					if (noise == 2 && disturbed) {
						eth.moveTimeForwardAndInvokeEventsUs(50);
						eth.firePrimaryTriggerRise();
					}
					if (tc.triggerState.getShaftSynchronized()) {
						const auto index =
								tc.triggerState.getCurrentIndex() +
								(tc.triggerState.getCrankSynchronizationCounter() % 2) * tc.triggerShape.getSize();
						const auto next = index + 2 == tc.engineCycleEventCount ? 0 : index + 2;
						TrgPhase current{tc.triggerFormDetails.eventAngles[index]};
						TrgPhase following{tc.triggerFormDetails.eventAngles[next]};
						EnginePhaseInfo info{
								getTimeNowNt(), current, following, tc.toEngPhase(current), tc.toEngPhase(following)};
						if (auto delay = tc.instantRpm.ignitionProfile.getDelayNt(info, 1)) {
							valid++;
							if (noise == 0) {
								EXPECT_NEAR(delay.Value, USF2NT(100.0f), USF2NT(0.5f));
							}
						}
					}
				}
				EXPECT_GT(discharges, 10u);
				EXPECT_TRUE(tc.triggerState.hasSynchronizedPhase());
				EXPECT_GT(valid, 50u);
				if (feedbackMode != 0) {
					EXPECT_GT(engine->rpmCalculator.getRollingCycleRpm().rpm, 0);
				}
				tc.syncAndReport(2, 1 - (tc.triggerState.getCrankSynchronizationCounter() % 2));
				// Phase shift invalidates a formerly ready cache immediately.
				EnginePhaseInfo old{getTimeNowNt(), {0}, {10}, {0}, {10}};
				EXPECT_FALSE(tc.instantRpm.ignitionProfile.getDelayNt(old, 1));
				engine->OnTriggerSynchronizationLost();
				eth.moveTimeForwardAndInvokeEventsUs(10000);
				for (auto& pin : enginePins.coils) {
					EXPECT_FALSE(pin.getLogicValue());
				}
				engine->onIgnitionEvent = {};
				EXPECT_FALSE(tc.instantRpm.ignitionProfile.getDelayNt(old, 1));
			}
		}
	}
}

TEST(IgnitionTimeBudget, RevisionInvalidatesOnlyOldChargeToken) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->ignitionTimeBudget = true;
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->minimumIgnitionTiming = -100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	auto& tc = engine->triggerCentral;
	initializeSkippedToothTrigger(&tc.triggerShape, 36, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	for (int i = 0; i < 68; i++) {
		tc.triggerFormDetails.eventAngles[2 * i] = (i / 34) * 360 + (i % 34) * 10;
		tc.triggerFormDetails.eventAngles[2 * i + 1] = tc.triggerFormDetails.eventAngles[2 * i];
	}
	tc.instantRpm.ignitionProfile.configure(tc.triggerShape, tc.triggerFormDetails);
	tc.triggerState.setNeedsDisambiguation(true, true);
	tc.triggerState.syncEnginePhase(2, 0, 720);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	auto& event = engine->ignitionEvents.elements[0];
	ASSERT_EQ(IgnitionOccurrenceState::ChargePending, event.state);
	const auto occurrenceGeneration = event.generation;
	const auto chargeGeneration = event.chargeGeneration;
	auto oldCharge = event.dwellStartTimer.action;
	const auto oldFireArgument = event.sparkEvent.action.getArgument();
	eth.moveTimeForwardAndInvokeEventsUs(200);
	engine->rpmCalculator.oneDegreeUs = 400;
	engine->module<TriggerScheduler>()->onEnginePhase(1000, {getTimeNowNt(), 20, 30, 20, 30});
	EXPECT_EQ(occurrenceGeneration, event.generation);
	EXPECT_NE(chargeGeneration, event.chargeGeneration);
	EXPECT_EQ(oldFireArgument, event.sparkEvent.scheduling.action.getArgument());
	oldCharge.execute();
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(300);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(700);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionTimeBudget, ExpiredTargetIsCountedAndCannotBeRevivedByRetard) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->ignitionTimeBudget = true;
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->minimumIgnitionTiming = -100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	auto& tc = engine->triggerCentral;
	initializeSkippedToothTrigger(&tc.triggerShape, 36, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
	for (int i = 0; i < 68; i++) {
		tc.triggerFormDetails.eventAngles[2 * i] = (i / 34) * 360 + (i % 34) * 10;
		tc.triggerFormDetails.eventAngles[2 * i + 1] = tc.triggerFormDetails.eventAngles[2 * i];
	}
	tc.instantRpm.ignitionProfile.configure(tc.triggerShape, tc.triggerFormDetails);
	tc.triggerState.setNeedsDisambiguation(true, true);
	tc.triggerState.syncEnginePhase(2, 0, 720);
	onTriggerEventSparkLogic({getTimeNowNt(), 40, 50, 40, 50});
	auto& event = engine->ignitionEvents.elements[0];
	EXPECT_EQ(1u, event.expiredTargetCount);
	engine->cylinders[0].setIgnitionTimingBtdc(-45);
	onTriggerEventSparkLogic({getTimeNowNt(), 40, 50, 40, 50});
	EXPECT_EQ(0u, engine->engineState.sparkCounter);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	onTriggerEventSparkLogic({getTimeNowNt(), 0, 10, 0, 10});
	onTriggerEventSparkLogic({getTimeNowNt(), 30, 40, 30, 40});
	EXPECT_EQ(1u, engine->engineState.sparkCounter);
	eth.moveTimeForwardAndInvokeEventsUs(2000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionRetarget, DirectionAndFeasibleWindow) {
	EXPECT_EQ(IgnitionRetargetStatus::Full, selectIgnitionTarget(20, 15, 10, 30).status);
	EXPECT_FLOAT_EQ(10, selectIgnitionTarget(20, 5, 10, 30).distance);
	EXPECT_EQ(IgnitionRetargetStatus::Limited, selectIgnitionTarget(20, 5, 10, 30).status);
	EXPECT_FLOAT_EQ(30, selectIgnitionTarget(20, 40, 10, 30).distance);
	EXPECT_EQ(IgnitionRetargetStatus::Rejected, selectIgnitionTarget(10, 5, 10, 30).status);
	EXPECT_EQ(IgnitionRetargetStatus::Rejected, selectIgnitionTarget(30, 40, 10, 30).status);
	EXPECT_EQ(IgnitionRetargetStatus::Rejected, selectIgnitionTarget(20, 25, 30, 10).status);
	EXPECT_EQ(IgnitionRetargetStatus::Recovered, selectIgnitionTarget(5, -2, 10, 30).status);
	EXPECT_FLOAT_EQ(10, selectIgnitionTarget(5, -2, 10, 30).distance);
	EXPECT_FLOAT_EQ(5, selectIgnitionTarget(5, 7, 10, 30, true).distance);
	EXPECT_EQ(IgnitionRetargetStatus::Rejected, selectIgnitionTarget(5, 7, 10, 30, true).status);
	EXPECT_FLOAT_EQ(15, selectIgnitionTarget(5, 15, 10, 30, true).distance);
	EXPECT_EQ(IgnitionRetargetStatus::Full, selectIgnitionTarget(5, 15, 10, 30, true).status);
	EXPECT_FLOAT_EQ(10, selectIgnitionTarget(15, 5, 10, 30, true).distance);
}

TEST(IgnitionCycleProfile, InverseBudgetMatchesForwardAcrossGapsAndWrap) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (int teeth : {36, 60}) {
		ProfileTrace t(teeth);
		for (unsigned j = 0; j < 4 * t.predictor.toothCount(); j++) {
			const float span = t.nextSpan();
			t.accept();
			if (j > t.predictor.toothCount() + 3) {
				for (float distance : {0.0f, 1.0f, 15.0f, 45.0f, 179.0f}) {
					auto time = t.predictor.getTimeToAngleNt(t.phase, distance);
					ASSERT_TRUE(time);
					auto inverse = t.predictor.getAngleForTimeNt(t.phase, time.Value);
					ASSERT_TRUE(inverse);
					EXPECT_NEAR(distance, inverse.Value, .0001f);
				}
				EXPECT_FALSE(t.predictor.getAngleForTimeNt(t.phase, USF2NT(100000)));
				auto stale = t.phase;
				stale.timestamp += US2NT(1);
				EXPECT_FALSE(t.predictor.getAngleForTimeNt(stale, USF2NT(100)));
			}
			t.now += static_cast<int32_t>(USF2NT(span * 100));
		}
		t.predictor.reset();
		EXPECT_FALSE(t.predictor.getAngleForTimeNt(t.phase, 0));
	}
}

namespace {
struct RetargetFixture {
	EngineTestHelper eth{engine_type_e::TEST_ENGINE};
	IgnitionEvent& event = engine->ignitionEvents.elements[0];
	RetargetFixture() {
		setCylinderCount(1);
		engineConfiguration->ignitionTimeBudget = true;
		engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
		engineConfiguration->minimumIgnitionTiming = -100;
		engineConfiguration->maximumIgnitionTiming = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->rpmCalculator.oneDegreeUs = 100;
		auto& tc = engine->triggerCentral;
		initializeSkippedToothTrigger(&tc.triggerShape, 36, 2, FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);
		for (int i = 0; i < 68; i++) {
			tc.triggerFormDetails.eventAngles[2 * i] = (i / 34) * 360 + (i % 34) * 10;
			tc.triggerFormDetails.eventAngles[2 * i + 1] = tc.triggerFormDetails.eventAngles[2 * i];
		}
		tc.instantRpm.ignitionProfile.configure(tc.triggerShape, tc.triggerFormDetails);
		tc.triggerState.setNeedsDisambiguation(true, true);
		tc.triggerState.syncEnginePhase(2, 0, 720);
	}
	void command(float rawTarget) {
		engine->cylinders[0].setIgnitionTimingBtdc(-rawTarget);
	}
	void step(float angle, float next) {
		const EnginePhaseInfo phase{getTimeNowNt(), {angle}, {next}, {angle}, {next}};
		engine->module<TriggerScheduler>()->onEnginePhase(1000, phase);
		onTriggerEventSparkLogic(phase);
	}
	void move(int us) {
		eth.moveTimeForwardAndInvokeEventsUs(us);
	}
	void arm() {
		command(25);
		step(10, 20);
	}
};
} // namespace

TEST(IgnitionTimeBudget, ALSSoftCutUsesLiveAntiLagState) {
	{
		RetargetFixture f;
		engineConfiguration->ALSSkipRatio = 2;
		engineConfiguration->antiLagEnabled = true;
		engine->antilagController.isAntilagCondition = true;
		f.command(25);
		f.step(10, 20);
		EXPECT_EQ(1u, engine->engineState.sparkCounter);
		EXPECT_TRUE(f.event.wasSparkLimited);
	}

	for (bool configured : {false, true}) {
		RetargetFixture f;
		engineConfiguration->ALSSkipRatio = 2;
		engineConfiguration->antiLagEnabled = configured;
		engine->antilagController.isAntilagCondition = !configured;
		f.command(25);
		f.step(10, 20);
		EXPECT_EQ(1u, engine->engineState.sparkCounter);
		EXPECT_FALSE(f.event.wasSparkLimited);
	}
}

TEST(IgnitionRetarget, RetainedCandidateAppliesPartialAdvanceBeforeArming) {
	RetargetFixture f;
	f.command(40);
	f.step(0, 10);
	ASSERT_TRUE(f.event.candidateValid);
	ASSERT_EQ(IgnitionOccurrenceState::Closed, f.event.state);
	f.move(2000);
	f.command(15); // passed command; old target 40 is still feasible
	f.step(20, 30);
	EXPECT_EQ(0u, f.event.expiredTargetCount);
	EXPECT_EQ(static_cast<int>(IgnitionRetargetStatus::Limited), f.event.retargetStatus);
	EXPECT_NEAR(29, f.event.plannedSparkAngle, .02);
	f.move(0);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	f.move(950);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	f.command(45);
	f.step(30, 40);
	EXPECT_EQ(1u, engine->engineState.sparkCounter);
}

TEST(IgnitionRetarget, FullPrearmChangeAndSpeedRecoveryAreDistinct) {
	for (bool recovery : {false, true}) {
		RetargetFixture f;
		f.command(recovery ? 25 : 40);
		f.step(0, 10);
		f.move(recovery ? 4000 : 2000);
		f.command(recovery ? 20 : 35);
		f.step(recovery ? 40 : 20, recovery ? 50 : 30);
		EXPECT_EQ(
				static_cast<int>(recovery ? IgnitionRetargetStatus::Recovered : IgnitionRetargetStatus::Full),
				f.event.retargetStatus);
		EXPECT_NEAR(recovery ? 49 : 35, f.event.plannedSparkAngle, .02);
		f.move(2000);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	}
}

TEST(IgnitionRetarget, PendingRetargetInvalidatesExtractedHighAndLow) {
	RetargetFixture f;
	f.arm();
	ASSERT_EQ(IgnitionOccurrenceState::ChargePending, f.event.state);
	auto high = f.event.dwellStartTimer.action;
	auto low = f.event.sparkEvent.action;
	const auto owner = f.event.generation;
	const auto fireGeneration = f.event.fireGeneration;
	f.move(200);
	f.command(28);
	f.step(12, 22);
	EXPECT_EQ(owner, f.event.generation);
	EXPECT_NE(fireGeneration, f.event.fireGeneration);
	EXPECT_FLOAT_EQ(28, f.event.plannedSparkAngle);
	high.execute();
	low.execute();
	EXPECT_EQ(IgnitionOccurrenceState::ChargePending, f.event.state);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	f.move(600);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	low.execute();
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	f.move(1500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionRetarget, ChargingAdvanceAndRetardRespectPhysicalWindow) {
	for (bool retard : {false, true}) {
		RetargetFixture f;
		f.arm();
		auto oldLow = f.event.sparkEvent.action;
		f.move(500);
		ASSERT_TRUE(enginePins.coils[0].getLogicValue());
		const auto deadline = enginePins.coils[0].hardDeadline();
		const auto high = enginePins.coils[0].firstHigh();
		f.move(500);
		f.command(retard ? 40 : 21);
		f.step(20, 30);
		EXPECT_EQ(static_cast<int>(IgnitionRetargetStatus::Limited), f.event.retargetStatus);
		EXPECT_NEAR(retard ? 29 : 23, f.event.plannedSparkAngle, .02);
		EXPECT_EQ(deadline, enginePins.coils[0].hardDeadline());
		EXPECT_EQ(high, enginePins.coils[0].firstHigh());
		oldLow.execute();
		EXPECT_TRUE(enginePins.coils[0].getLogicValue());
		f.move(retard ? 850 : 250);
		EXPECT_TRUE(enginePins.coils[0].getLogicValue());
		f.move(100);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	}
}

TEST(IgnitionRetarget, RegisteredNearFireNeedsNoNewServiceReserve) {
	for (bool change : {false, true}) {
		RetargetFixture f;
		f.arm();
		f.move(1000);
		f.step(20, 30);
		const auto action = f.event.sparkEvent.scheduling.action;
		const auto when = f.event.sparkEvent.scheduling.momentX;
		const auto generation = f.event.fireGeneration;
		f.move(450);
		if (change) {
			f.command(25.2f);
		}
		f.step(24.5f, 30);
		EXPECT_EQ(when, f.event.sparkEvent.scheduling.momentX);
		EXPECT_EQ(generation, f.event.fireGeneration);
		EXPECT_EQ(action.getArgument(), f.event.sparkEvent.scheduling.action.getArgument());
		EXPECT_FLOAT_EQ(25, f.event.plannedSparkAngle);
		if (change) {
			EXPECT_EQ(static_cast<int>(IgnitionRetargetStatus::Rejected), f.event.retargetStatus);
		}
		f.move(50);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	}
}

TEST(IgnitionRetarget, NormalLowSurvivesFlushAndLossOfPlannerSupport) {
	for (int mode : {0, 1, 2}) {
		RetargetFixture f;
		f.arm();
		f.move(1000);
		f.step(20, 30);
		const auto when = f.event.sparkEvent.scheduling.momentX;
		if (mode == 0) {
			engine->module<TriggerScheduler>()->flush();
		}
		if (mode == 1) {
			engineConfiguration->ignitionTimeBudget = false;
			f.step(21, 30);
		}
		if (mode == 2) {
			engine->OnTriggerSynchronizationLost();
		}
		EXPECT_EQ(when, f.event.sparkEvent.scheduling.momentX);
		f.move(500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0u, f.event.hardGuardCount);
	}
}

TEST(IgnitionRetarget, WrappedTdcCandidateCannotBeRevivedAfterFire) {
	RetargetFixture f;
	f.command(-20);
	f.step(660, 670);
	ASSERT_TRUE(f.event.candidateValid);
	EXPECT_EQ(1u, f.event.candidateTdcCycle);
	f.move(2000);
	f.command(-45);
	f.step(680, 690);
	EXPECT_EQ(1u, f.event.plannedTdcCycle);
	EXPECT_NEAR(689, f.event.plannedSparkAngle, .02);
	f.move(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	f.command(5);
	f.step(0, 10);
	EXPECT_EQ(1u, engine->engineState.sparkCounter);
}

TEST(IgnitionRetarget, RecoveryCannotExceedConfiguredMaximumRetard) {
	RetargetFixture f;
	engineConfiguration->minimumIgnitionTiming = 0;
	f.command(-10);
	f.step(680, 690);
	ASSERT_TRUE(f.event.candidateValid);
	f.move(3500);
	f.command(-5); // A new command cannot authorize ignition past the configured limit.
	f.step(715, 0);
	EXPECT_EQ(0u, engine->engineState.sparkCounter);
	EXPECT_EQ(1u, f.event.expiredTargetCount);
	EXPECT_FALSE(f.event.candidateValid);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	f.command(0);
	f.step(716, 0);
	EXPECT_EQ(0u, engine->engineState.sparkCounter);
}

TEST(IgnitionRetarget, BothSameTickOrdersRemainOneOccurrence) {
	for (bool retargetFirst : {false, true}) {
		RetargetFixture f;
		f.arm();
		f.move(1000);
		f.step(20, 30);
		auto oldLow = f.event.sparkEvent.scheduling.action;
		if (retargetFirst) {
			setTimeNowUs(getTimeNowUs() + 500);
		} else {
			f.move(500);
		}
		f.command(28);
		f.step(25, 35);
		oldLow.execute();
		EXPECT_EQ(retargetFirst, enginePins.coils[0].getLogicValue());
		f.move(500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(1u, engine->engineState.sparkCounter);
	}
}

TEST(IgnitionRetarget, FireTokenWrapAndOldMainCallbackCannotFireMultispark) {
	RetargetFixture f;
	f.event.fireGeneration = 0x7ffffffe;
	engine->engineState.multispark.count = 1;
	engine->engineState.multispark.dwell = US2NT(500);
	engine->engineState.multispark.delay = US2NT(250);
	f.arm();
	EXPECT_EQ(0x7ffffffu, f.event.fireGeneration);
	auto oldLow = f.event.sparkEvent.action;
	f.move(200);
	f.command(28);
	f.step(12, 22);
	EXPECT_EQ(1u, f.event.fireGeneration);
	f.move(1200);
	f.step(24, 34);
	auto mainLow = f.event.sparkEvent.scheduling.action;
	f.move(650); // main fire 1800, multispark HIGH 2050
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(2u, f.event.fireGeneration);
	oldLow.execute();
	mainLow.execute();
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	f.move(500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionRetarget, CutRejectsPendingChangeAndKeepsChargedLow) {
	for (bool charging : {false, true}) {
		RetargetFixture f;
		f.arm();
		f.move(charging ? 1000 : 200);
		if (charging) {
			f.step(20, 30);
		}
		engineConfiguration->isIgnitionEnabled = false;
		f.command(28);
		f.step(charging ? 21 : 12, 30);
		EXPECT_EQ(charging, enginePins.coils[0].getLogicValue());
		f.move(2000);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0u, f.event.hardGuardCount);
	}
}

TEST(IgnitionRetarget, UnchangedShortBudgetPreservesAdmissionAndPhysicalMinimum) {
	RetargetFixture f;
	engine->rpmCalculator.oneDegreeUs = 10;
	f.command(25);
	f.step(0, 10);
	EXPECT_FLOAT_EQ(25, f.event.plannedSparkAngle);
	EXPECT_EQ(static_cast<int>(IgnitionRetargetStatus::Unchanged), f.event.retargetStatus);
	f.move(0);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	const auto cap = enginePins.coils[0].hardDeadline();
	f.move(200);
	f.step(20, 30);
	f.move(599);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(cap, enginePins.coils[0].hardDeadline());
	f.move(1);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0u, f.event.hardGuardCount);
}
