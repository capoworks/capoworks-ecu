/*
 * test_startOfCrankingPrimingPulse.cpp
 *
 *  Created on: Mar 4, 2018
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

using ::testing::_;

namespace {
void configureValidPrime(MockInjectorModel2& injector) {
	engine->module<InjectorModelPrimary>().set(&injector);
	Sensor::setMockValue(SensorType::Clt, 50);
	std::fill(std::begin(engineConfiguration->primeValues), std::end(engineConfiguration->primeValues), 100);
}

class CapturingExecutor final : public Scheduler {
public:
	void schedule(const char*, scheduling_s*, efitick_t, action_s) override {
		ADD_FAILURE() << "prime callbacks must use atomic batches";
	}

	void cancel(scheduling_s*) override {}

	bool scheduleBatch(const ScheduledAction* events, size_t count) override {
		if (queuedCount + count > efi::size(queued)) {
			ADD_FAILURE() << "too many captured callbacks";
			return false;
		}

		for (size_t i = 0; i < count; i++) {
			queued[queuedCount++] = events[i];
		}
		return true;
	}

	void execute(size_t index) {
		if (index >= queuedCount) {
			ADD_FAILURE() << "callback index is not queued";
			return;
		}

		auto action = queued[index].action;
		queued[index].action = {};
		action.execute();
	}

	ScheduledAction queued[MaxScheduleBatchSize];
	size_t queuedCount = 0;
};
} // namespace

TEST(engine, testPlainCrankingWithoutAdvancedFeatures) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setTable(config->injectionPhase, -180.0f);
	engineConfiguration->isFasterEngineSpinUpEnabled = false;
	engine->tdcMarkEnabled = false;
	engineConfiguration->cranking.baseFuel = 12;

	setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth);
	ASSERT_EQ(0, Sensor::getOrZero(SensorType::Rpm)) << "RPM=0";

	eth.fireTriggerEventsWithDuration(/* durationMs */ 200);
	// still no RPM since need to cycles measure cycle duration
	ASSERT_EQ(0, Sensor::getOrZero(SensorType::Rpm)) << "start-RPM#1";

	eth.fireRise(/* delayMs */ 200);
	ASSERT_EQ(300, Sensor::getOrZero(SensorType::Rpm)) << "RPM#2";
	// two simultaneous injections
	ASSERT_EQ(4, engine->scheduler.size()) << "plain#2";

	eth.assertEvent5("sim start", 0, (void*)scheduledStartInjection, 100000 - 1625);
	// -1 because ugh floating point math
	eth.assertEvent5("sim end", 1, (void*)scheduledEndInjection, 100000 - 1);
}

TEST(priming, startScheduling) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	ASSERT_EQ(0, Sensor::getOrZero(SensorType::Rpm)) << "RPM=0";

	// Turn on the ignition switch!
	engine->module<PrimeController>()->onIgnitionStateChanged(true);

	ASSERT_EQ(1, engine->scheduler.size()) << "prime fuel";
	EXPECT_EQ(1, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, floodClearSkipsPrimeFromDriverIntent) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->isCylinderCleanupEnabled = true;
	Sensor::setMockValue(SensorType::Tps1, 0);
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 95);

	engine->module<PrimeController>()->onIgnitionStateChanged(true);

	EXPECT_EQ(0, engine->scheduler.size()) << "flood clear must not schedule prime fuel";
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, duration) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	MockInjectorModel2 injectorModel;
	engine->module<InjectorModelPrimary>().set(&injectorModel);

	for (size_t i = 0; i < efi::size(engineConfiguration->primeBins); i++) {
		engineConfiguration->primeBins[i] = i * 10;
	}

	EXPECT_CALL(injectorModel, getInjectionDuration(0.075f)).WillOnce(Return(20.0f));
	engineConfiguration->primeValues[5] = 75; // <-- We test this point

	// With coolant temp, we should see value from mock
	Sensor::setMockValue(SensorType::Clt, 50);
	EXPECT_EQ(20, engine->module<PrimeController>()->getPrimeDuration());

	// Without coolant temp, no prime
	Sensor::resetMockValue(SensorType::Clt);
	EXPECT_EQ(0, engine->module<PrimeController>()->getPrimeDuration());
}

TEST(priming, delayedSkipStillAcknowledgesAdmittedCallback) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	Sensor::setInvalidMockValue(SensorType::Clt);

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	ASSERT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(100000);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
}

TEST(priming, isIndependentFromAirmassInjectionAdmission) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	engine->module<InjectorModelPrimary>().set(&injector);
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	engineConfiguration->primingDelay = 0;
	Sensor::setMockValue(SensorType::Clt, 50);
	std::fill(std::begin(engineConfiguration->primeValues), std::end(engineConfiguration->primeValues), 100);
	EXPECT_CALL(injector, getInjectionDuration(_)).WillOnce(Return(20.0f));

	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	ASSERT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(100000);
	EXPECT_TRUE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(20000);
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, keyOffBeforeDelayedStartPreventsPulse) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	configureValidPrime(injector);
	EXPECT_CALL(injector, getInjectionDuration(_)).Times(0);

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	ASSERT_EQ(1, engine->scheduler.size());
	ASSERT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	engine->module<PrimeController>()->onIgnitionStateChanged(false);
	eth.moveTimeForwardAndInvokeEventsUs(100000);

	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, newKeyCycleDoesNotRunStaleDelayedStart) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	configureValidPrime(injector);
	EXPECT_CALL(injector, getInjectionDuration(_)).WillOnce(Return(20.0f));

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	engine->module<PrimeController>()->onIgnitionStateChanged(false);
	engineConfiguration->primingDelay = 1.0f;
	engine->module<PrimeController>()->onIgnitionStateChanged(true);

	ASSERT_EQ(2, engine->scheduler.size());
	ASSERT_EQ(2, engine->airmassInjectionState.pendingCallbacks());

	// Reach the original key cycle's delayed callback while the new one is still pending.
	eth.moveTimeForwardAndInvokeEventsUs(100000);
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(1, engine->scheduler.size()) << "new key cycle prime remains queued";
	EXPECT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(1000000);
	EXPECT_TRUE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter());

	eth.moveTimeForwardAndInvokeEventsUs(20000);
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, keyOffBetweenStartAndOpenPreventsUnmatchedPulse) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	configureValidPrime(injector);
	EXPECT_CALL(injector, getInjectionDuration(_)).WillOnce(Return(20.0f));

	CapturingExecutor executor;
	engine->scheduler.setMockExecutor(&executor);

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	ASSERT_EQ(1, executor.queuedCount);
	executor.execute(0); // delayed start: schedules open and end

	ASSERT_TRUE(engine->module<PrimeController>()->isPriming());
	ASSERT_EQ(3, executor.queuedCount);
	ASSERT_EQ(2, engine->airmassInjectionState.pendingCallbacks());

	engine->module<PrimeController>()->onIgnitionStateChanged(false);

	// An unrelated owner keeps the output count observable if the stale end
	// callback incorrectly tries to close an output the prime never opened.
	enginePins.injectors[0].open();
	ASSERT_EQ(1, enginePins.injectors[0].getOverlappingCounter());

	executor.execute(1); // open after key-off
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	executor.execute(2); // end must not close an output that was never opened
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());

	enginePins.injectors[0].close();
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());

	engine->scheduler.setMockExecutor(nullptr);
}

TEST(priming, keyOffAfterOpenStillClosesAcceptedPulse) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	configureValidPrime(injector);
	EXPECT_CALL(injector, getInjectionDuration(_)).WillOnce(Return(20.0f));

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	eth.moveTimeForwardAndInvokeEventsUs(100000);

	ASSERT_TRUE(engine->module<PrimeController>()->isPriming());
	ASSERT_EQ(1, enginePins.injectors[0].getOverlappingCounter());
	ASSERT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	engine->module<PrimeController>()->onIgnitionStateChanged(false);
	EXPECT_TRUE(engine->module<PrimeController>()->isPriming()) << "an opened pulse is allowed to finish";
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter());

	eth.moveTimeForwardAndInvokeEventsUs(20000);
	EXPECT_FALSE(engine->module<PrimeController>()->isPriming());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(priming, startSchedulesTrackedPairWithCapturedMask) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 injector;
	configureValidPrime(injector);
	engine->engineState.cylinderCount = 4;
	EXPECT_CALL(injector, getInjectionDuration(_)).WillOnce(Return(20.0f));

	engine->module<PrimeController>()->onIgnitionStateChanged(true);
	eth.moveTimeForwardAndInvokeEventsUs(100000);
	EXPECT_EQ(1, enginePins.injectors[3].getOverlappingCounter());
	EXPECT_EQ(1, engine->airmassInjectionState.pendingCallbacks());

	// Closing must use the mask captured with the pair, not the new cylinder count.
	engine->engineState.cylinderCount = 2;
	eth.moveTimeForwardAndInvokeEventsUs(20000);
	EXPECT_EQ(0, enginePins.injectors[3].getOverlappingCounter());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}
