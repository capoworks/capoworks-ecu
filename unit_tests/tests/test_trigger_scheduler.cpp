/*
 * Lifecycle of events shared by the trigger-tooth queue and the time scheduler.
 */

#include "pch.h"
#include "spark_logic.h"

namespace {

void countAction(int* count) {
	++*count;
}

EnginePhaseInfo phaseAtCurrentTooth() {
	EnginePhaseInfo phase;
	phase.timestamp = getTimeNowNt();
	phase.currentEngPhase = EngPhase{100};
	phase.nextEngPhase = EngPhase{110};
	phase.currentTrgPhase = TrgPhase{100};
	phase.nextTrgPhase = TrgPhase{110};
	return phase;
}

// Schedule() can run a timer callback inline. Inject a cancellation or stop while
// the first of two due events is being promoted to the time scheduler.
class InterleavingExecutor : public Scheduler {
public:
	TriggerScheduler* triggerScheduler = nullptr;
	AngleBasedEvent* victim = nullptr;
	bool stopOnSchedule = false;
	std::vector<scheduling_s*> scheduled;

	void schedule(const char*, scheduling_s* scheduling, efitick_t, action_s) override {
		scheduled.push_back(scheduling);
		if (scheduled.size() == 1) {
			if (stopOnSchedule) {
				triggerScheduler->flush();
			} else {
				triggerScheduler->cancel(victim);
			}
		}
	}

	void cancel(scheduling_s*) override {}
};

// Use the real timer queue and callbacks, including nested registration. Like
// SingleTimerExecutor, drain expired actions on registration unless already draining.
class InlineExecutor : public TestExecutor {
public:
	void schedule(const char* msg, scheduling_s* scheduling, efitick_t time, action_s action) override {
		TestExecutor::schedule(msg, scheduling, time, action);
		if (!executing) {
			executing = true;
			executeAll(getTimeNowUs());
			executing = false;
		}
	}

private:
	bool executing = false;
};

} // namespace

TEST(TriggerScheduler, engineStopEmptiesQueueButKeepsArmedTimer) {
	AngleBasedEvent event;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	int oldCount = 0;
	int newCount = 0;

	triggerScheduler->schedule(&event, EngPhase{123}, {countAction, &oldCount});
	engine->scheduler.schedule(
			"ordinary", &event.scheduling, getTimeNowNt() + US2NT(2000), {countAction, &oldCount});
	ASSERT_EQ(1, triggerScheduler->getQueueSizeForUnitTest());

	engine->OnTriggerSynchronizationLost();
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	ASSERT_EQ(1, engine->scheduler.size());

	// A new schedule may be queued while an ordinary timer from the old schedule is
	// still pending. The old timer remains armed and the new angle event stays queued.
	EXPECT_FALSE(
			triggerScheduler->scheduleOrQueue(&event, EngPhase{125}, {countAction, &newCount}, phaseAtCurrentTooth()));
	EXPECT_EQ(1, triggerScheduler->getQueueSizeForUnitTest());

	eth.moveTimeForwardAndInvokeEventsUs(2000);
	EXPECT_EQ(1, oldCount);
	EXPECT_EQ(0, newCount);
	EXPECT_EQ(1, triggerScheduler->getQueueSizeForUnitTest());

	auto phase = phaseAtCurrentTooth();
	phase.currentTrgPhase = TrgPhase{120};
	phase.nextTrgPhase = TrgPhase{130};
	triggerScheduler->onEnginePhase(1000, phase);
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_EQ(1, newCount);
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(TriggerScheduler, scheduleByTimeReplacesStaleEventAndOverdwellTimer) {
	AngleBasedEvent event;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	int staleCount = 0;
	int fallbackCount = 0;
	int newCount = 0;

	triggerScheduler->schedule(&event, EngPhase{123}, {countAction, &staleCount});
	engine->scheduler.schedule(
			"overdwell", &event.scheduling, getTimeNowNt() + US2NT(2000), {countAction, &fallbackCount});
	ASSERT_EQ(1, triggerScheduler->getQueueSizeForUnitTest());

	EXPECT_TRUE(
			triggerScheduler->scheduleOrQueue(&event, EngPhase{105}, {countAction, &newCount}, phaseAtCurrentTooth()));
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	ASSERT_EQ(1, engine->scheduler.size());

	eth.moveTimeForwardAndInvokeEventsUs(3000);
	EXPECT_EQ(1, newCount);
	EXPECT_EQ(0, staleCount);
	EXPECT_EQ(0, fallbackCount);
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(TriggerScheduler, toothPromotionReplacesOverdwellTimer) {
	AngleBasedEvent event;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	int sparkCount = 0;
	int fallbackCount = 0;

	triggerScheduler->schedule(&event, EngPhase{105}, {countAction, &sparkCount});
	engine->scheduler.schedule(
			"overdwell", &event.scheduling, getTimeNowNt() + US2NT(2000), {countAction, &fallbackCount});
	triggerScheduler->onEnginePhase(1000, phaseAtCurrentTooth());
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	ASSERT_EQ(1, engine->scheduler.size());

	eth.moveTimeForwardAndInvokeEventsUs(3000);
	EXPECT_EQ(1, sparkCount);
	EXPECT_EQ(0, fallbackCount);
}

TEST(TriggerScheduler, cancelOnEmptyQueueIsSafe) {
	AngleBasedEvent event;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->module<TriggerScheduler>()->cancel(&event);
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());
}

TEST(TriggerScheduler, obsoleteCallbackCannotDischargeNewOwner) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto oldFire = engine->ignitionEvents.elements[0].sparkEvent.scheduling.action;
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	oldFire.execute();
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(TriggerScheduler, mixedQueuePromotesDueEventsAndPreservesWaitingOrder) {
	AngleBasedEvent events[6];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	int fired[6] = {};
	for (int i = 0; i < 6; i++) {
		// Alternate waiting and due entries, so removal covers middle and tail links.
		scheduler.schedule(&events[i], EngPhase{i % 2 ? 105.f : 125.f}, {countAction, &fired[i]});
	}
	scheduler.onEnginePhase(1000, phaseAtCurrentTooth());
	ASSERT_EQ(3, scheduler.getQueueSizeForUnitTest());
	ASSERT_EQ(3, engine->scheduler.size());
	for (int i = 0; i < 3; i++) {
		EXPECT_EQ(&events[2 * i], scheduler.getElementAtIndexForUnitTest(i));
	}
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	for (int i = 0; i < 6; i++) {
		EXPECT_EQ(i % 2, fired[i]);
	}

	auto phase = phaseAtCurrentTooth();
	phase.currentTrgPhase = TrgPhase{120};
	phase.nextTrgPhase = TrgPhase{130};
	scheduler.onEnginePhase(1000, phase);
	EXPECT_EQ(0, scheduler.getQueueSizeForUnitTest());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	for (auto count : fired) {
		EXPECT_EQ(1, count);
	}
}

TEST(TriggerScheduler, ignitionRegistersPhysicalGuardForEachNewCycle) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->firingOrder = FO_1;
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	auto& scheduler = *engine->module<TriggerScheduler>();

	// Dwell at 15 degrees, spark at 25: the physical guard drains each occurrence.
	for (int cycle = 0; cycle < 2; cycle++) {
		onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
		ASSERT_EQ(1, scheduler.getQueueSizeForUnitTest());
		eth.moveTimeForwardAndInvokeEventsUs(500);
		EXPECT_TRUE(enginePins.coils[0].getLogicValue());
		eth.moveTimeForwardAndInvokeEventsUs(1500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, scheduler.getQueueSizeForUnitTest());
		EXPECT_EQ(0, engine->scheduler.size());
	}
}

TEST(TriggerScheduler, rapidRestartDoesNotChargeAfterOldOverdwellWithoutProtection) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->firingOrder = FO_1;
	engineConfiguration->minimumIgnitionTiming = -25;
	auto prepare = [] {
		engine->rpmCalculator.setRpmValue(1666.6667f);
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);
	};
	prepare();
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1300);
	engine->OnTriggerSynchronizationLost();
	prepare();
	// Reacquisition before the old hard guard: the next charge would start at
	// 2.3 ms, after the old guard at 2 ms. No more teeth arrive afterwards.
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(300);
	eth.moveTimeForwardAndInvokeEventsUs(5000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());
	EXPECT_EQ(0, engine->scheduler.size());

	// The next opportunity after the old timer drains must still work normally.
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(TriggerScheduler, restartWithSmallerCoilMaskPreservesPreviousDischarge) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(4);
	engineConfiguration->ignitionMode = IM_WASTED_SPARK;
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	for (auto& cylinder : engine->cylinders) {
		cylinder.setIgnitionTimingBtdc(-25);
	}
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	const auto oldMask = engine->ignitionEvents.elements[0].calculateIgnitionOutputMask();
	ASSERT_NE(0, oldMask & (oldMask - 1)); // A wasted pair, not just one output.
	eth.moveTimeForwardAndInvokeEventsUs(500);
	forEachSetBit(oldMask, [](size_t i) { EXPECT_TRUE(enginePins.coils[i].getLogicValue()); });
	engine->OnTriggerSynchronizationLost();
	engineConfiguration->ignitionMode = IM_ONE_COIL;
	engine->rpmCalculator.oneDegreeUs = 100;
	// The new spark is on this tooth and would replace the old pair's timer.
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	EXPECT_EQ(1, engine->ignitionEvents.elements[0].calculateIgnitionOutputMask());
	eth.moveTimeForwardAndInvokeEventsUs(3000);
	forEachSetBit(oldMask, [](size_t i) { EXPECT_FALSE(enginePins.coils[i].getLogicValue()); });
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());
}

TEST(TriggerScheduler, overlappingCyclePreservesMultisparkAndResumesAfterDrain) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->firingOrder = FO_1;
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	engine->engineState.multispark.count = 1;
	engine->engineState.multispark.delay = US2NT(500);
	engine->engineState.multispark.dwell = US2NT(1000);
	int charges = 0;
	int discharges = 0;
	engine->onIgnitionEvent = [&](IgnitionContext ctx, bool charging) {
		EXPECT_FALSE(ctx.isOverdwellProtect);
		(charging ? charges : discharges)++;
	};
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(300);
	const auto counterBefore = engine->engineState.sparkCounter;
	auto* charge = engine->scheduler.getForUnitTest(0);
	auto* discharge = engine->scheduler.getForUnitTest(1);
	const auto chargeTime = charge->momentX;
	const auto dischargeTime = discharge->momentX;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	EXPECT_EQ(counterBefore, engine->engineState.sparkCounter);
	EXPECT_EQ(2, engine->scheduler.size());
	EXPECT_EQ(chargeTime, charge->momentX);
	EXPECT_EQ(dischargeTime, discharge->momentX);
	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(2, charges);
	EXPECT_EQ(2, discharges);
	EXPECT_EQ(0, engine->scheduler.size());
	engine->engineState.multispark.count = 0;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_EQ(3, charges);
	EXPECT_EQ(3, discharges);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(TriggerScheduler, overdwellRemovesPendingSpark) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	auto& event = engine->ignitionEvents.elements[0].sparkEvent;
	int sparkCount = 0;

	triggerScheduler->schedule(&event, EngPhase{123}, {countAction, &sparkCount});
	ASSERT_EQ(1, triggerScheduler->getQueueSizeForUnitTest());

	IgnitionContext ctx;
	ctx.eventIndex = 0;
	ctx.isOverdwellProtect = true;
	engine->ignitionEvents.elements[0].state = IgnitionOccurrenceState::Charging;
	event.fallbackIsCurrent = true;
	engine->ignitionState.dwellAngle = NAN; // Stop after discharging the coil.
	fireSparkAndPrepareNextSchedule(ctx);

	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	EXPECT_EQ(0, sparkCount);
}

TEST(TriggerScheduler, cancelDuringPromotionPreventsLaterEvent) {
	InterleavingExecutor executor;
	AngleBasedEvent first;
	AngleBasedEvent second;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	int fireCount = 0;

	executor.triggerScheduler = triggerScheduler;
	executor.victim = &second;
	engine->scheduler.setMockExecutor(&executor);
	triggerScheduler->schedule(&first, EngPhase{105}, {countAction, &fireCount});
	triggerScheduler->schedule(&second, EngPhase{106}, {countAction, &fireCount});
	triggerScheduler->onEnginePhase(1000, phaseAtCurrentTooth());

	ASSERT_EQ(1u, executor.scheduled.size());
	EXPECT_EQ(&first.scheduling, executor.scheduled[0]);
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(TriggerScheduler, stopDuringPromotionDropsRemainingEvents) {
	InterleavingExecutor executor;
	AngleBasedEvent first;
	AngleBasedEvent second;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto* triggerScheduler = &*engine->module<TriggerScheduler>();
	int fireCount = 0;

	executor.triggerScheduler = triggerScheduler;
	executor.stopOnSchedule = true;
	engine->scheduler.setMockExecutor(&executor);
	triggerScheduler->schedule(&first, EngPhase{105}, {countAction, &fireCount});
	triggerScheduler->schedule(&second, EngPhase{106}, {countAction, &fireCount});
	triggerScheduler->onEnginePhase(1000, phaseAtCurrentTooth());

	ASSERT_EQ(1u, executor.scheduled.size());
	EXPECT_EQ(&first.scheduling, executor.scheduled[0]);
	EXPECT_EQ(0, triggerScheduler->getQueueSizeForUnitTest());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(TriggerScheduler, cancelHeadMiddleTailAndAppendAfterLastRemoval) {
	AngleBasedEvent events[5];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	int count = 0;
	for (int index : {0, 1, 2, 3}) {
		scheduler.schedule(&events[index], EngPhase{125}, {countAction, &count});
	}
	for (int index : {0, 2, 3, 1}) {
		scheduler.cancel(&events[index]);
		EXPECT_EQ(TriggerQueueMembership::None, events[index].queueMembership);
		EXPECT_EQ(nullptr, events[index].next);
		ASSERT_TRUE(scheduler.validateQueuesForUnitTest());
	}
	scheduler.schedule(&events[4], EngPhase{125}, {countAction, &count});
	EXPECT_EQ(&events[4], scheduler.getElementAtIndexForUnitTest(0));
	EXPECT_EQ(1, scheduler.getQueueSizeForUnitTest());
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
}

TEST(TriggerScheduler, flushResetsMembershipButPreservesFallbackAssociation) {
	AngleBasedEvent events[8];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	int count = 0;
	for (auto& event : events) {
		scheduler.schedule(&event, EngPhase{125}, {countAction, &count});
		event.fallbackIsCurrent = true;
	}
	scheduler.flush();
	for (auto& event : events) {
		EXPECT_EQ(TriggerQueueMembership::None, event.queueMembership);
		EXPECT_EQ(nullptr, event.next);
		EXPECT_TRUE(event.fallbackIsCurrent);
	}
	for (int index = 7; index >= 0; index--) {
		scheduler.schedule(&events[index], EngPhase{125}, {countAction, &count});
	}
	ASSERT_TRUE(scheduler.validateQueuesForUnitTest());
	for (int index = 0; index < 8; index++) {
		EXPECT_EQ(&events[7 - index], scheduler.getElementAtIndexForUnitTest(index));
	}
}

TEST(TriggerScheduler, duplicateInsertionPreservesMembershipAndTail) {
	AngleBasedEvent events[3];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	int count = 0;
	scheduler.schedule(&events[0], EngPhase{125}, {countAction, &count});
	scheduler.schedule(&events[1], EngPhase{125}, {countAction, &count});
	scheduler.schedule(&events[0], EngPhase{125}, {countAction, &count});
	scheduler.schedule(&events[2], EngPhase{125}, {countAction, &count});
	ASSERT_TRUE(scheduler.validateQueuesForUnitTest());
	EXPECT_EQ(3, scheduler.getQueueSizeForUnitTest());
	for (int i = 0; i < 3; i++) {
		EXPECT_EQ(&events[i], scheduler.getElementAtIndexForUnitTest(i));
	}
	EXPECT_EQ(1, eth.getWarningCounter());
}

TEST(TriggerScheduler, inlineDueCancellationAllowsReuseAndAppend) {
	InterleavingExecutor executor;
	AngleBasedEvent events[4];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	int count = 0;
	executor.triggerScheduler = &scheduler;
	executor.victim = &events[2];
	engine->scheduler.setMockExecutor(&executor);
	for (int i = 0; i < 3; i++) {
		scheduler.schedule(&events[i], EngPhase{105}, {countAction, &count});
	}
	scheduler.onEnginePhase(1000, phaseAtCurrentTooth());
	EXPECT_EQ(2u, executor.scheduled.size());
	ASSERT_TRUE(scheduler.validateQueuesForUnitTest());
	scheduler.schedule(&events[2], EngPhase{125}, {countAction, &count});
	scheduler.schedule(&events[3], EngPhase{125}, {countAction, &count});
	EXPECT_EQ(2, scheduler.getQueueSizeForUnitTest());
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(TriggerScheduler, inlineFlushClearsWaitingAndDueMembership) {
	InterleavingExecutor executor;
	AngleBasedEvent events[4];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	int count = 0;
	executor.triggerScheduler = &scheduler;
	executor.stopOnSchedule = true;
	engine->scheduler.setMockExecutor(&executor);
	for (int i = 0; i < 4; i++) {
		scheduler.schedule(&events[i], EngPhase{i % 2 ? 125.f : 105.f}, {countAction, &count});
	}
	scheduler.onEnginePhase(1000, phaseAtCurrentTooth());
	EXPECT_EQ(1u, executor.scheduled.size());
	ASSERT_TRUE(scheduler.validateQueuesForUnitTest());
	for (auto& event : events) {
		EXPECT_EQ(TriggerQueueMembership::None, event.queueMembership);
		scheduler.schedule(&event, EngPhase{125}, {countAction, &count});
	}
	EXPECT_EQ(4, scheduler.getQueueSizeForUnitTest());
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(TriggerScheduler, queueOperationsMatchReferenceAcrossRepeatedReuse) {
	AngleBasedEvent events[16];
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	std::vector<int> expected;
	uint32_t state = 0x12345678;
	int count = 0;
	int expectedCount = 0;
	engine->rpmCalculator.oneDegreeUs = 100;
	for (int step = 0; step < 2000; step++) {
		state = state * 1664525 + 1013904223;
		int index = (state >> 16) % 16;
		auto found = std::find(expected.begin(), expected.end(), index);
		switch ((state >> 24) % 4) {
			case 0:
				if (found == expected.end()) {
					scheduler.schedule(&events[index], EngPhase{index % 2 ? 105.f : 125.f}, {countAction, &count});
					expected.push_back(index);
				}
				break;
			case 1:
				scheduler.cancel(&events[index]);
				if (found != expected.end()) {
					expected.erase(found);
				}
				break;
			case 2:
				scheduler.flush();
				expected.clear();
				break;
			case 3: {
				auto phase = phaseAtCurrentTooth();
				const bool laterTooth = state & 1;
				if (laterTooth) {
					phase.currentTrgPhase = TrgPhase{120};
					phase.nextTrgPhase = TrgPhase{130};
				}
				scheduler.onEnginePhase(1000, phase);
				expected.erase(
						std::remove_if(
								expected.begin(),
								expected.end(),
								[&](int i) {
									if (bool(i % 2) != laterTooth) {
										++expectedCount;
										return true;
									}
									return false;
								}),
						expected.end());
				eth.moveTimeForwardAndInvokeEventsUs(1000);
				EXPECT_EQ(expectedCount, count) << step;
				EXPECT_EQ(0, engine->scheduler.size()) << step;
				break;
			}
		}
		ASSERT_TRUE(scheduler.validateQueuesForUnitTest()) << step;
		ASSERT_EQ(expected.size(), scheduler.getQueueSizeForUnitTest()) << step;
		for (size_t i = 0; i < expected.size(); i++) {
			EXPECT_EQ(&events[expected[i]], scheduler.getElementAtIndexForUnitTest(i));
		}
		for (int i = 0; i < 16; i++) {
			bool present = std::find(expected.begin(), expected.end(), i) != expected.end();
			EXPECT_EQ(
					present ? TriggerQueueMembership::Waiting : TriggerQueueMembership::None,
					events[i].queueMembership);
		}
	}
	EXPECT_GT(expectedCount, 0);
}

TEST(TriggerScheduler, inlineCallbackReusesDueEventAndRequeuesItself) {
	InlineExecutor executor;
	AngleBasedEvent first;
	AngleBasedEvent victim;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	struct Context {
		TriggerScheduler* scheduler;
		AngleBasedEvent* first;
		AngleBasedEvent* victim;
		int newCount = 0;
		int selfCount = 0;
		int inlineCount = 0;
	} ctx{&scheduler, &first, &victim};
	int staleCount = 0;
	engine->scheduler.setMockExecutor(&executor);
	scheduler.schedule(
			&first,
			EngPhase{100},
			{+[](Context* c) {
				 ++c->inlineCount;
				 EXPECT_EQ(TriggerQueueMembership::Due, c->victim->queueMembership);
				 EXPECT_TRUE(c->scheduler->scheduleOrQueue(
						 c->victim, EngPhase{105}, {countAction, &c->newCount}, phaseAtCurrentTooth()));
				 c->scheduler->schedule(c->first, EngPhase{125}, {countAction, &c->selfCount});
			 },
			 &ctx});
	scheduler.schedule(&victim, EngPhase{106}, {countAction, &staleCount});
	scheduler.onEnginePhase(1000, phaseAtCurrentTooth());
	EXPECT_EQ(1, ctx.inlineCount);
	EXPECT_EQ(1, scheduler.getQueueSizeForUnitTest());
	EXPECT_EQ(&first, scheduler.getElementAtIndexForUnitTest(0));
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
	eth.moveTimeForwardUs(1000);
	executor.executeAll(getTimeNowUs());
	EXPECT_EQ(1, ctx.newCount);
	EXPECT_EQ(0, staleCount);
	EXPECT_EQ(0, ctx.selfCount);

	auto phase = phaseAtCurrentTooth();
	phase.currentTrgPhase = TrgPhase{120};
	phase.nextTrgPhase = TrgPhase{130};
	scheduler.onEnginePhase(1000, phase);
	eth.moveTimeForwardUs(1000);
	executor.executeAll(getTimeNowUs());
	EXPECT_EQ(1, ctx.selfCount);
	EXPECT_EQ(0, scheduler.getQueueSizeForUnitTest());
	EXPECT_EQ(0, executor.size());
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
	EXPECT_EQ(0, eth.getWarningCounter());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(TriggerScheduler, inlineSynchronizationLossDropsDueAndWaitingButPreservesSafetyTimer) {
	InlineExecutor executor;
	AngleBasedEvent events[3];
	scheduling_s safetyTimer;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& scheduler = *engine->module<TriggerScheduler>();
	engine->rpmCalculator.oneDegreeUs = 100;
	int staleCount = 0;
	int safetyCount = 0;
	engine->scheduler.setMockExecutor(&executor);
	engine->scheduler.schedule("safety", &safetyTimer, getTimeNowNt() + US2NT(1000), {countAction, &safetyCount});
	scheduler.schedule(&events[0], EngPhase{100}, {+[](void*) { engine->OnTriggerSynchronizationLost(); }});
	scheduler.schedule(&events[1], EngPhase{106}, {countAction, &staleCount});
	scheduler.schedule(&events[2], EngPhase{125}, {countAction, &staleCount});
	scheduler.onEnginePhase(1000, phaseAtCurrentTooth());
	EXPECT_EQ(0, scheduler.getQueueSizeForUnitTest());
	EXPECT_TRUE(scheduler.validateQueuesForUnitTest());
	EXPECT_EQ(1, executor.size());
	eth.moveTimeForwardUs(2000);
	executor.executeAll(getTimeNowUs());
	EXPECT_EQ(0, staleCount);
	EXPECT_EQ(1, safetyCount);
	EXPECT_EQ(0, executor.size());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(IgnitionOccurrence, fireBeforePendingChargeInvalidatesExtractedCallback) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	enginePins.coils[0].setLow();
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto& event = engine->ignitionEvents.elements[0];
	auto oldCharge = event.dwellStartTimer.action;
	auto oldFire = event.sparkEvent.scheduling.action;
	ASSERT_TRUE(oldCharge);
	ASSERT_TRUE(oldFire);
	oldFire.execute();
	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_FALSE(event.dwellStartTimer.action);
	oldCharge.execute();
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	const auto newGeneration = event.generation;
	oldCharge.execute();
	oldFire.execute();
	EXPECT_EQ(newGeneration, event.generation);
	EXPECT_EQ(IgnitionOccurrenceState::ChargePending, event.state);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionOccurrence, duplicateChargeDoesNotRenewDwell) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	enginePins.coils[0].setLow();
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto charge = engine->ignitionEvents.elements[0].dwellStartTimer.action;
	eth.moveTimeForwardAndInvokeEventsUs(500);
	eth.moveTimeForwardAndInvokeEventsUs(900);
	charge.execute();
	eth.moveTimeForwardAndInvokeEventsUs(100);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionOccurrence, staleCallbacksCannotMutateNewerOccurrence) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	enginePins.coils[0].setLow();
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto& event = engine->ignitionEvents.elements[0];
	auto oldCharge = event.dwellStartTimer.action;
	auto oldFire = event.sparkEvent.scheduling.action;
	const auto originalFireTime = event.sparkEvent.scheduling.momentX;
	ASSERT_TRUE(oldCharge);
	ASSERT_TRUE(oldFire);

	// Charge for 500 us, then ask the ordinary fire callback to fire early.
	// The executor clears the scheduling record before invoking the callback.
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_EQ(IgnitionOccurrenceState::Charging, event.state);
	const auto physicalDeadline = enginePins.coils[0].hardDeadline();
	engine->scheduler.cancel(&event.sparkEvent.scheduling);
	oldFire.execute();
	auto oldUnderdwell = event.sparkEvent.scheduling.action;
	ASSERT_TRUE(oldUnderdwell);
	ASSERT_EQ(2, engine->scheduler.size());
	ASSERT_EQ(&event.sparkEvent.scheduling, engine->scheduler.getHead());
	ASSERT_LT(event.sparkEvent.scheduling.momentX, originalFireTime);
	auto* physicalGuard = engine->scheduler.getForUnitTest(1);
	ASSERT_NE(nullptr, physicalGuard);
	EXPECT_EQ(NT2US(physicalDeadline), physicalGuard->momentX);
	EXPECT_LT(event.sparkEvent.scheduling.momentX, physicalGuard->momentX);

	// Let the retry discharge and close the old occurrence, then arm a new one.
	eth.moveTimeForwardAndInvokeEventsUs(900);
	ASSERT_EQ(IgnitionOccurrenceState::Closed, event.state);
	const auto oldGeneration = event.generation;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	const auto newGeneration = event.generation;
	ASSERT_NE(newGeneration, oldGeneration);

	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_EQ(IgnitionOccurrenceState::Charging, event.state);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());

	oldCharge.execute();
	oldFire.execute();
	oldUnderdwell.execute();
	EXPECT_EQ(newGeneration, event.generation);
	EXPECT_EQ(IgnitionOccurrenceState::Charging, event.state);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());

	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionOccurrence, CutBetweenArmAndChargeCannotRevivePendingHigh) {
	for (int cut : {0, 1, 2, 3}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);
		enginePins.coils[0].setLow();
		int mainChargeEvents = 0;
		engine->onIgnitionEvent = [&](IgnitionContext, bool charging) {
			if (charging) {
				mainChargeEvents++;
			}
		};
		onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
		auto stale = engine->ignitionEvents.elements[0].dwellStartTimer.action;
		if (cut == 0) {
			engineConfiguration->isIgnitionEnabled = false;
		}
		if (cut == 1) {
			getLimpManager()->fatalError();
			turnAllPinsOff();
		}
		if (cut == 2) {
			engine->OnTriggerSynchronizationLost();
		}
		if (cut == 3) {
			engine->module<TriggerScheduler>()->flush();
		}
		eth.moveTimeForwardAndInvokeEventsUs(2000);
		stale.execute();
		EXPECT_EQ(0, mainChargeEvents);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
		EXPECT_EQ(IgnitionOccurrenceState::Closed, engine->ignitionEvents.elements[0].state);
	}
}

TEST(IgnitionOccurrence, flushPreservesChargingOccurrenceForOwedLow) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	auto& event = engine->ignitionEvents.elements[0];

	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_EQ(IgnitionOccurrenceState::Charging, event.state);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());

	engine->module<TriggerScheduler>()->flush();
	EXPECT_EQ(IgnitionOccurrenceState::Charging, event.state);
	ASSERT_EQ(1, engine->scheduler.size());

	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionOccurrence, CutWhileChargingPreservesLowButSuppressesMultispark) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	engine->engineState.multispark.count = 2;
	engine->engineState.multispark.delay = US2NT(500);
	engine->engineState.multispark.dwell = US2NT(1000);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	engineConfiguration->isIgnitionEnabled = false;
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionTrailing, pendingMultisparkHandoffCutDischargesTrailingHigh) {
	for (bool liveCut : {false, true}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engineConfiguration->enableTrailingSparks = true;
		engine->engineState.trailingSparkAngle = 10;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);
		engine->engineState.multispark.count = 1;
		engine->engineState.multispark.delay = US2NT(500);
		engine->engineState.multispark.dwell = US2NT(500);

		int ignitionCallbacks = 0;
		engine->onIgnitionEvent = [&](IgnitionContext, bool) { ignitionCallbacks++; };

		onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
		auto& event = engine->ignitionEvents.elements[0];
		eth.moveTimeForwardAndInvokeEventsUs(500);
		ASSERT_TRUE(enginePins.coils[0].getLogicValue());
		auto copiedTrailingCharge = event.trailingSparkCharge.action;
		ASSERT_TRUE(copiedTrailingCharge);
		const auto initialMainDeadline = enginePins.coils[0].hardDeadline();

		eth.moveTimeForwardAndInvokeEventsUs(500);
		engine->module<TriggerScheduler>()->onEnginePhase(
				1000, {getTimeNowNt(), 20, 30, 20, 30});
		eth.moveTimeForwardAndInvokeEventsUs(500);
		ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());

		ASSERT_EQ(IgnitionOccurrenceState::ChargePending, event.state);
		ASSERT_FALSE(enginePins.coils[0].getLogicValue());
		ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
		ASSERT_TRUE(event.trailingPending);
		ASSERT_TRUE(event.trailingCharging);
		ASSERT_EQ(2, ignitionCallbacks);
		ASSERT_LT(getTimeNowNt(), initialMainDeadline);
		const auto trailingDeadline = enginePins.trailingCoils[0].hardDeadline();
		ASSERT_EQ(
				enginePins.trailingCoils[0].firstHigh() + static_cast<uint32_t>(MSF2NT(1.5f)),
				trailingDeadline);
		EXPECT_LT(event.dwellStartTimer.momentX, NT2US(trailingDeadline));
		EXPECT_FALSE(event.trailingSparkFire.action);

		auto copiedMainCharge = event.dwellStartTimer.action;
		auto copiedMainFire = event.sparkEvent.scheduling.action;
		ASSERT_TRUE(copiedMainCharge);
		ASSERT_TRUE(copiedMainFire);
		ignitionCallbacks = 0;

		if (liveCut) {
			engineConfiguration->isIgnitionEnabled = false;
			eth.moveTimeForwardAndInvokeEventsUs(500);
		} else {
			engine->module<TriggerScheduler>()->flush();
		}

		EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
		EXPECT_FALSE(event.trailingPending);
		EXPECT_FALSE(event.trailingCharging);
		EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
		EXPECT_EQ(0, ignitionCallbacks);
		EXPECT_EQ(0, engine->scheduler.size());

		engineConfiguration->isIgnitionEnabled = true;
		constexpr uint32_t newerOwner = 0x123456;
		ASSERT_TRUE(enginePins.trailingCoils[0].charge(newerOwner, 1, {}));
		copiedTrailingCharge.execute();
		copiedMainCharge.execute();
		copiedMainFire.execute();

		EXPECT_TRUE(enginePins.trailingCoils[0].ownedBy(newerOwner));
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_FALSE(event.trailingPending);
		EXPECT_FALSE(event.trailingCharging);
		EXPECT_EQ(0, ignitionCallbacks);
		enginePins.trailingCoils[0].setLow();
	}
}

TEST(IgnitionTrailing, copiedCallbacksAreGenerationBoundAndKeepDwellSnapshot) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engineConfiguration->enableTrailingSparks = true;
	engine->engineState.trailingSparkAngle = 10;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	engine->engineState.multispark.count = 1;
	engine->engineState.multispark.delay = US2NT(500);
	engine->engineState.multispark.dwell = US2NT(500);

	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	auto& event = engine->ignitionEvents.elements[0];
	EXPECT_FLOAT_EQ(1, event.trailingDwell);
	EXPECT_EQ(0, event.trailingCylinder);

	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	ASSERT_TRUE(event.trailingPending);
	ASSERT_FALSE(event.trailingCharging);
	const auto firstTrailingGeneration = event.trailingGeneration;
	auto copiedTrailingCharge = event.trailingSparkCharge.action;
	ASSERT_TRUE(copiedTrailingCharge);
	const auto initialMainDeadline = enginePins.coils[0].hardDeadline();

	eth.moveTimeForwardAndInvokeEventsUs(500);
	engine->module<TriggerScheduler>()->onEnginePhase(
			1000, {getTimeNowNt(), 20, 30, 20, 30});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	ASSERT_TRUE(event.trailingCharging);
	ASSERT_FALSE(enginePins.coils[0].getLogicValue());
	ASSERT_EQ(IgnitionOccurrenceState::ChargePending, event.state);
	ASSERT_LT(getTimeNowNt(), initialMainDeadline);
	const auto firstTrailingDeadline = enginePins.trailingCoils[0].hardDeadline();
	ASSERT_EQ(
			enginePins.trailingCoils[0].firstHigh() + static_cast<uint32_t>(MSF2NT(1.5f)),
			firstTrailingDeadline);
	EXPECT_LT(event.dwellStartTimer.momentX, NT2US(firstTrailingDeadline));

	// A copied charge must not renew the high or clear the pending LOW.
	copiedTrailingCharge.execute();
	EXPECT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_TRUE(event.trailingPending);
	EXPECT_EQ(firstTrailingDeadline, enginePins.trailingCoils[0].hardDeadline());

	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	ASSERT_EQ(IgnitionOccurrenceState::Charging, event.state);
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_FALSE(enginePins.coils[0].getLogicValue());

	ASSERT_TRUE(event.trailingPending);
	EXPECT_FLOAT_EQ(1, event.trailingDwell);
	EXPECT_FLOAT_EQ(0.5f, event.occurrenceDwell);
	auto copiedTrailingFire = event.trailingSparkFire.action;
	ASSERT_TRUE(copiedTrailingFire);

	const auto counterBeforeRestart = engine->engineState.sparkCounter;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	EXPECT_EQ(counterBeforeRestart, engine->engineState.sparkCounter);

	event.cylinderNumber = 1;
	enginePins.trailingCoils[1].setHigh();
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_TRUE(enginePins.trailingCoils[1].getLogicValue());
	enginePins.trailingCoils[1].setLow();
	event.cylinderNumber = 0;
	EXPECT_FALSE(event.trailingPending);
	EXPECT_FALSE(event.trailingCharging);

	// Arm a new trailing lifecycle and prove old copied callbacks cannot touch it.
	engine->engineState.multispark.count = 0;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	ASSERT_TRUE(event.trailingPending);
	const auto secondTrailingGeneration = event.trailingGeneration;
	ASSERT_NE(firstTrailingGeneration, secondTrailingGeneration);

	eth.moveTimeForwardAndInvokeEventsUs(1000);
	ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	copiedTrailingCharge.execute();
	copiedTrailingFire.execute();
	EXPECT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_EQ(secondTrailingGeneration, event.trailingGeneration);
	EXPECT_TRUE(event.trailingPending);

	engine->module<TriggerScheduler>()->onEnginePhase(1000, {getTimeNowNt(), 20, 30, 20, 30});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_FALSE(event.trailingPending);
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionReset, pendingOccurrenceCannotReenergizeAfterProductionReset) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engineConfiguration->enableTrailingSparks = true;
	engine->engineState.trailingSparkAngle = 10;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);

	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto& event = engine->ignitionEvents.elements[0];
	ASSERT_EQ(IgnitionOccurrenceState::ChargePending, event.state);
	auto copiedCharge = event.dwellStartTimer.action;
	auto copiedFire = event.sparkEvent.scheduling.action;
	ASSERT_TRUE(copiedCharge);
	ASSERT_TRUE(copiedFire);
	ASSERT_GT(engine->scheduler.size(), 0);

	resetConfigurationExt(engineConfiguration->engineType);

	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_FALSE(event.trailingPending);
	EXPECT_FALSE(event.trailingCharging);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());

	copiedCharge.execute();
	copiedFire.execute();

	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionReset, chargingOccurrenceCannotReenergizeAfterProductionReset) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engineConfiguration->enableTrailingSparks = true;
	engine->engineState.trailingSparkAngle = 5;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);

	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto& event = engine->ignitionEvents.elements[0];
	auto copiedMainCharge = event.dwellStartTimer.action;
	ASSERT_TRUE(copiedMainCharge);
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	auto copiedMainFire = event.sparkEvent.scheduling.action;
	auto copiedTrailingCharge = event.trailingSparkCharge.action;
	ASSERT_TRUE(copiedMainFire);
	ASSERT_TRUE(copiedTrailingCharge);

	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	ASSERT_TRUE(event.trailingPending);
	ASSERT_TRUE(event.trailingCharging);
	ASSERT_GT(engine->scheduler.size(), 0);

	resetConfigurationExt(engineConfiguration->engineType);

	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_FALSE(event.trailingPending);
	EXPECT_FALSE(event.trailingCharging);
	EXPECT_FALSE(event.dwellStartTimer.action);
	EXPECT_FALSE(event.sparkEvent.scheduling.action);
	EXPECT_FALSE(event.trailingSparkCharge.action);
	EXPECT_FALSE(event.trailingSparkFire.action);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->module<TriggerScheduler>()->getQueueSizeForUnitTest());

	copiedMainCharge.execute();
	copiedMainFire.execute();
	copiedTrailingCharge.execute();

	EXPECT_EQ(IgnitionOccurrenceState::Closed, event.state);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionTrailing, liveCutBeforeChargeRejectsCopiedHigh) {
	for (int cut : {0, 1}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engineConfiguration->enableTrailingSparks = true;
		engine->engineState.trailingSparkAngle = 10;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);

		onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
		auto& event = engine->ignitionEvents.elements[0];
		eth.moveTimeForwardAndInvokeEventsUs(500);
		ASSERT_TRUE(event.trailingPending);
		auto copiedTrailingCharge = event.trailingSparkCharge.action;
		ASSERT_TRUE(copiedTrailingCharge);

		if (cut == 0) {
			engineConfiguration->isIgnitionEnabled = false;
		} else {
			engineConfiguration->enableTrailingSparks = false;
		}
		copiedTrailingCharge.execute();

		EXPECT_FALSE(event.trailingPending);
		EXPECT_FALSE(event.trailingCharging);
		EXPECT_FALSE(event.trailingSparkCharge.action);
		EXPECT_FALSE(event.trailingSparkFire.action);
		EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());

		// The main coil was already charging, so its LOW remains owed.
		eth.moveTimeForwardAndInvokeEventsUs(1500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
	}
}

TEST(IgnitionTrailing, syncLossDropsPendingHighButKeepsChargingLow) {
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engineConfiguration->enableTrailingSparks = true;
		engine->engineState.trailingSparkAngle = 10;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);

		onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
		auto& event = engine->ignitionEvents.elements[0];
		eth.moveTimeForwardAndInvokeEventsUs(500);
		ASSERT_TRUE(enginePins.coils[0].getLogicValue());
		ASSERT_TRUE(event.trailingPending);
		auto copiedTrailingCharge = event.trailingSparkCharge.action;
		ASSERT_TRUE(copiedTrailingCharge);

		engine->OnTriggerSynchronizationLost();
		EXPECT_FALSE(event.trailingPending);
		EXPECT_FALSE(event.trailingSparkCharge.action);
		EXPECT_FALSE(event.trailingSparkFire.action);
		copiedTrailingCharge.execute();
		EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());

		// The main occurrence was already charging, so sync loss retains its LOW.
		ASSERT_EQ(IgnitionOccurrenceState::Charging, event.state);
		eth.moveTimeForwardAndInvokeEventsUs(1500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
	}

	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engineConfiguration->enableTrailingSparks = true;
		engine->engineState.trailingSparkAngle = 10;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);

		onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
		auto& event = engine->ignitionEvents.elements[0];
		eth.moveTimeForwardAndInvokeEventsUs(500);
		eth.moveTimeForwardAndInvokeEventsUs(1000);
		ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
		ASSERT_TRUE(event.trailingPending);
		ASSERT_TRUE(event.trailingCharging);
		auto copiedTrailingCharge = event.trailingSparkCharge.action;
		ASSERT_FALSE(copiedTrailingCharge);

		engine->OnTriggerSynchronizationLost();
		EXPECT_TRUE(event.trailingPending);
		EXPECT_TRUE(event.trailingCharging);
		EXPECT_TRUE(enginePins.trailingCoils[0].getLogicValue());

		// The main hard guard still fires and discharges the already-charged trailing coil.
		eth.moveTimeForwardAndInvokeEventsUs(500);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
		EXPECT_FALSE(event.trailingPending);
		EXPECT_FALSE(event.trailingCharging);
		EXPECT_EQ(0, engine->scheduler.size());
	}
}

TEST(IgnitionOccurrence, GenerationWrapRejectsPreviousPendingAction) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	auto& event = engine->ignitionEvents.elements[0];
	event.generation = 0x7fffffe;
	event.chargeGeneration = 0x7fffffe;
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto oldCharge = event.dwellStartTimer.action;
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	ASSERT_EQ(1u, event.generation);
	ASSERT_EQ(1u, event.chargeGeneration);
	oldCharge.execute();
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionOccurrence, SameTickChargeAndFireOrdersAreSafe) {
	for (bool chargeFirst : {false, true}) {
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		setCylinderCount(1);
		engineConfiguration->minimumIgnitionTiming = -25;
		engine->rpmCalculator.oneDegreeUs = 100;
		engine->ignitionState.sparkDwell = 1;
		engine->ignitionState.dwellAngle = 10;
		engine->cylinders[0].setIgnitionTimingBtdc(-25);
		enginePins.coils[0].setLow();
		onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
		auto& event = engine->ignitionEvents.elements[0];
		auto charge = event.dwellStartTimer.action;
		auto fire = event.sparkEvent.scheduling.action;
		engine->scheduler.cancel(&event.dwellStartTimer);
		engine->scheduler.cancel(&event.sparkEvent.scheduling);
		if (chargeFirst) {
			charge.execute();
			EXPECT_TRUE(enginePins.coils[0].getLogicValue()) << "charge should raise the coil";
			fire.execute();
			EXPECT_TRUE(enginePins.coils[0].getLogicValue()) << "underdwell retry should retain HIGH";
			EXPECT_EQ(2, engine->scheduler.size());
			const auto physicalDeadline = enginePins.coils[0].hardDeadline();
			auto* underdwell = engine->scheduler.getForUnitTest(0);
			auto* physicalGuard = engine->scheduler.getForUnitTest(1);
			ASSERT_NE(nullptr, underdwell);
			ASSERT_NE(nullptr, physicalGuard);
			EXPECT_EQ(&event.sparkEvent.scheduling, underdwell);
			EXPECT_EQ(NT2US(physicalDeadline), physicalGuard->momentX);
			EXPECT_LT(underdwell->momentX, physicalGuard->momentX);
		} else {
			fire.execute();
			EXPECT_FALSE(enginePins.coils[0].getLogicValue()) << "fire-first should leave the coil LOW";
			charge.execute();
			EXPECT_FALSE(enginePins.coils[0].getLogicValue()) << "stale charge must not raise the coil";
		}
		eth.moveTimeForwardAndInvokeEventsUs(900);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
	}
}

TEST(IgnitionOccurrence, DwellUpdateAfterArmKeepsMinimumAndCap) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	auto& event = engine->ignitionEvents.elements[0];
	const auto armedMask = event.outputMaskSnapshot;
	engine->ignitionState.sparkDwell = 9;
	initializeIgnitionActions();
	EXPECT_FLOAT_EQ(1, event.occurrenceDwell);
	EXPECT_EQ(armedMask, event.outputMaskSnapshot);
	eth.moveTimeForwardAndInvokeEventsUs(500);
	ASSERT_TRUE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(US2NT(1500).count(), enginePins.coils[0].hardDeadline() - enginePins.coils[0].firstHigh());
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionPhysicalGuard, promotionAndRepeatedHighRetainHardDeadline) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 20, 10, 20});
	eth.moveTimeForwardAndInvokeEventsUs(500);
	auto& pin = enginePins.coils[0];
	ASSERT_TRUE(pin.getLogicValue());
	const auto first = pin.firstHigh();
	const auto deadline = pin.hardDeadline();

	// A greatly slowed next tooth promotes the normal spark beyond the hard cap.
	engine->rpmCalculator.oneDegreeUs = 1000;
	engine->module<TriggerScheduler>()->onEnginePhase(1000, {getTimeNowNt(), 20, 30, 20, 30});
	pin.setHigh();
	EXPECT_EQ(first, pin.firstHigh());
	EXPECT_EQ(deadline, pin.hardDeadline());

	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(pin.getLogicValue());
	EXPECT_EQ(1u, engine->ignitionEvents.elements[0].hardGuardCount);
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionPhysicalGuard, firstOwnerWinsAndExternalLowReleasesOwnership) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& pin = enginePins.coils[0];
	int guards = 0;
	ASSERT_TRUE(pin.charge(1, 2, {countAction, &guards}));
	const auto first = pin.firstHigh();
	const auto deadline = pin.hardDeadline();
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(pin.charge(2, 9, {countAction, &guards}));
	EXPECT_TRUE(pin.charge(1, 9, {countAction, &guards}));
	EXPECT_EQ(first, pin.firstHigh());
	EXPECT_EQ(deadline, pin.hardDeadline());
	pin.discharge(2);
	EXPECT_TRUE(pin.getLogicValue());
	pin.setLow();
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_TRUE(pin.charge(2, 2, {countAction, &guards}));
	pin.discharge(1);
	EXPECT_TRUE(pin.getLogicValue());
	pin.discharge(2);
	EXPECT_FALSE(pin.getLogicValue());
	EXPECT_EQ(0, guards);
}

TEST(IgnitionPhysicalGuard, closedOccurrenceStillForcesLowAndOldEpochIsInert) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& event = engine->ignitionEvents.elements[0];
	event.generation = 1;
	event.state = IgnitionOccurrenceState::Closed;
	IgnitionContext ctx;
	ctx.eventIndex = 0;
	ctx.generation = 1;
	ctx.isOverdwellProtect = true;
	auto& pin = enginePins.coils[0];
	ASSERT_TRUE(pin.charge(ctx.owner(), 1, {fireSparkAndPrepareNextSchedule, ctx}));
	auto oldGuard = engine->scheduler.getForUnitTest(0)->action;
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(pin.getLogicValue());

	ASSERT_TRUE(pin.charge(ctx.owner() + 16, 1, {}));
	oldGuard.execute();
	EXPECT_TRUE(pin.getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1500);
	EXPECT_FALSE(pin.getLogicValue());
}

TEST(IgnitionPhysicalGuard, pairedOutputsUseOwnAgeAndEarliestCapWins) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& event = engine->ignitionEvents.elements[0];
	event.generation = 1;
	event.state = IgnitionOccurrenceState::Charging;
	event.outputMaskSnapshot = 3;
	event.occurrenceDwell = 2;
	IgnitionContext ctx;
	ctx.eventIndex = 0;
	ctx.generation = 1;
	auto guard = ctx;
	guard.isOverdwellProtect = true;
	ASSERT_TRUE(enginePins.coils[0].charge(ctx.owner(), 1, {fireSparkAndPrepareNextSchedule, guard}));
	eth.moveTimeForwardAndInvokeEventsUs(800);
	ASSERT_TRUE(enginePins.coils[1].charge(ctx.owner(), 2, {fireSparkAndPrepareNextSchedule, guard}));
	eth.moveTimeForwardAndInvokeEventsUs(200);
	fireSparkAndPrepareNextSchedule(ctx);
	EXPECT_TRUE(enginePins.coils[0].getLogicValue());
	EXPECT_TRUE(enginePins.coils[1].getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.coils[1].getLogicValue());
	EXPECT_EQ(1u, event.hardGuardCount);
}

TEST(IgnitionPhysicalGuard, basePointerLowAndHighStartNewPhysicalEpoch) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->ignitionState.sparkDwell = 1;
	auto& pin = enginePins.coils[0];
	OutputPin* base = &pin;
	ASSERT_TRUE(pin.charge(1, 1, {}));
	const auto oldFirst = pin.firstHigh();
	eth.moveTimeForwardAndInvokeEventsUs(500);
	base->setValue(false);
	EXPECT_FALSE(pin.ownedBy(1));
	base->setValue(true);
	EXPECT_GT(pin.firstHigh(), oldFirst);
	eth.moveTimeForwardAndInvokeEventsUs(1499);
	EXPECT_TRUE(pin.getLogicValue());
	eth.moveTimeForwardAndInvokeEventsUs(1);
	EXPECT_FALSE(pin.getLogicValue());
}

TEST(IgnitionPhysicalGuard, invalidDwellAndFatalErrorNeverEnergize) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	for (float dwell : {0.0f, -1.0f, NAN, INFINITY, 0.000001f, 1001.0f}) {
		EXPECT_FALSE(enginePins.coils[0].charge(1, dwell, {}));
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
	}

	const bool priorFirmwareError = hasFirmwareErrorFlag;
	hasFirmwareErrorFlag = true;
	turnAllPinsOff();
	enginePins.coils[0].setValue(true);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
	EXPECT_FALSE(enginePins.coils[0].charge(1, 1, {}));
	hasFirmwareErrorFlag = priorFirmwareError;
}

TEST(IgnitionPhysicalGuard, stopAndDeInitInvalidatePhysicalOwner) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	auto& pin = enginePins.coils[0];
	ASSERT_TRUE(pin.charge(1, 2, {}));
	EXPECT_TRUE(pin.stop());
	EXPECT_FALSE(pin.ownedBy(1));
	EXPECT_EQ(0, engine->scheduler.size());

	ASSERT_TRUE(pin.charge(2, 2, {}));
	pin.deInit();
	EXPECT_FALSE(pin.ownedBy(2));
	EXPECT_EQ(0, engine->scheduler.size());
}

TEST(IgnitionPhysicalGuard, rawHighWithoutEngineIsRejected) {
	{
		EngineTestHelper eth(engine_type_e::TEST_ENGINE);
		enginePins.coils[0].setLow();
	}

	enginePins.coils[0].setValue(true);
	EXPECT_FALSE(enginePins.coils[0].getLogicValue());
}

TEST(IgnitionPhysicalGuard, multisparkCannotChangeTrailingSnapshot) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->minimumIgnitionTiming = -25;
	engineConfiguration->enableTrailingSparks = true;
	engine->engineState.trailingSparkAngle = 5;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	engine->engineState.multispark.count = 1;
	engine->engineState.multispark.delay = US2NT(500);
	engine->engineState.multispark.dwell = US2NT(200);
	onTriggerEventSparkLogic({getTimeNowNt(), 10, 30, 10, 30});
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	ASSERT_TRUE(enginePins.trailingCoils[0].getLogicValue());
	const auto deadline = enginePins.trailingCoils[0].hardDeadline();
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	const auto& event = engine->ignitionEvents.elements[0];
	EXPECT_FLOAT_EQ(1, event.trailingDwell);
	EXPECT_FLOAT_EQ(0.2f, event.occurrenceDwell);
	EXPECT_EQ(deadline, enginePins.trailingCoils[0].hardDeadline());
	eth.moveTimeForwardAndInvokeEventsUs(500);
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
	EXPECT_FALSE(event.trailingPending);
	eth.moveTimeForwardAndInvokeEventsUs(1000);
	EXPECT_FALSE(enginePins.trailingCoils[0].getLogicValue());
}
