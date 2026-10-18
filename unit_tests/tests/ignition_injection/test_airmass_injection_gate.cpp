#include "pch.h"
#include "airmass_injection_state.h"

namespace {
using Status = AirmassInjectionStatus;
using Fault = AirmassInjectionFault;

int startedCallbacks = 0;
int completedCallbacks = 0;

AirmassInjectionState& gate() {
	return engine->airmassInjectionState;
}

void selectMode(engine_load_mode_e mode) {
	engineConfiguration->fuelAlgorithm = mode;
	engine->engineState.airmassCalculationValid = false;
}

AirmassInjectionState::CalculationToken beginPositiveCalculation() {
	Sensor::setMockValue(SensorType::Rpm, 1000);
	return gate().beginCalculation(engineConfiguration->fuelAlgorithm, 1000, engine->getGlobalConfigurationVersion());
}

void completeValid(AirmassInjectionState::CalculationToken token) {
	engine->engineState.airmassCalculationValid = true;
	gate().completeCalculation(token, true);
}

void makeReady() {
	auto token = beginPositiveCalculation();
	gate().acceptCalculation();
	completeValid(token);
	ASSERT_TRUE(gate().allowInjection());
}

void trackedStartCallback(void*) {
	++startedCallbacks;
	chibios_rt::CriticalSectionLocker csl;
	gate().locked(csl).callbackCompleted();
}

void trackedEndCallback(void*) {
	++completedCallbacks;
	chibios_rt::CriticalSectionLocker csl;
	gate().locked(csl).callbackCompleted();
}

void resetTrackedCallbacks() {
	startedCallbacks = 0;
	completedCallbacks = 0;
}

bool queueTrackedPulse(int startUs, int endUs, bool prime = false) {
	ScheduledAction events[] = {
			{getTimeNowNt() + US2NT(startUs), {trackedStartCallback, nullptr}},
			{getTimeNowNt() + US2NT(endUs), {trackedEndCallback, nullptr}},
	};
	return scheduleFuelCallbacks(events, efi::size(events), prime);
}
} // namespace

TEST(airmassInjectionGate, StandaloneRequiresCompletedValidPublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	resetTrackedCallbacks();

	for (auto mode : {LM_SPEED_DENSITY, LM_ALPHA_N, LM_REAL_MAF}) {
		SCOPED_TRACE(static_cast<int>(mode));
		selectMode(mode);
		EXPECT_FALSE(gate().allowInjection());
		EXPECT_TRUE(gate().allowPrime());

		auto token = beginPositiveCalculation();
		engine->engineState.airmassCalculationValid = true;
		gate().acceptCalculation();
		EXPECT_FALSE(gate().allowInjection()) << "accepted work is not a published result";

		gate().completeCalculation(token, true);
		EXPECT_TRUE(gate().allowInjection());
		EXPECT_EQ(Status::Legacy, gate().status());
		EXPECT_EQ(Fault::None, gate().fault());
	}
}

TEST(airmassInjectionGate, FailureRequiresFreshValidPublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();

	auto failed = beginPositiveCalculation();
	gate().rejectCalculation(Fault::Sensor);
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_EQ(Status::Faulted, gate().status());
	EXPECT_EQ(Fault::Sensor, gate().fault());

	engine->engineState.airmassCalculationValid = true;
	gate().completeCalculation(failed, true);
	EXPECT_FALSE(gate().allowInjection()) << "failed generation cannot recover itself";

	auto invalid = beginPositiveCalculation();
	gate().completeCalculation(invalid, false);
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_EQ(Fault::Result, gate().fault());

	makeReady();
	EXPECT_TRUE(gate().allowInjection());
	EXPECT_EQ(Fault::None, gate().fault());
}

TEST(airmassInjectionGate, CompletedPublicationRemainsAdmittedDuringReplacement) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_ALPHA_N);
	makeReady();

	auto replacement = beginPositiveCalculation();
	engine->engineState.airmassCalculationValid = false;
	EXPECT_TRUE(gate().allowInjection()) << "in-progress validity must not revoke the last publication";
	EXPECT_EQ(Status::Legacy, gate().status());

	completeValid(replacement);
	EXPECT_TRUE(gate().allowInjection());

	gate().beginCalculation(LM_ALPHA_N, 0, engine->getGlobalConfigurationVersion());
	EXPECT_FALSE(gate().allowInjection());
}

TEST(airmassInjectionGate, StaleTokensAndInvalidationCannotChangeNewerState) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_REAL_MAF);
	makeReady();

	auto stale = beginPositiveCalculation();
	gate().acceptCalculation();
	auto current = beginPositiveCalculation();
	completeValid(current);
	ASSERT_TRUE(gate().allowInjection());

	gate().completeCalculation(stale, false);
	EXPECT_TRUE(gate().allowInjection());
	EXPECT_EQ(Status::Legacy, gate().status());

	gate().onConfigurationWrite(LM_REAL_MAF, false);
	EXPECT_FALSE(gate().allowInjection());
	gate().completeCalculation(current, true);
	EXPECT_FALSE(gate().allowInjection());
	makeReady();

	auto beforeStop = beginPositiveCalculation();
	gate().onEngineStop();
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_TRUE(gate().allowPrime());
	gate().completeCalculation(beforeStop, true);
	EXPECT_FALSE(gate().allowInjection());
	makeReady();
	EXPECT_TRUE(gate().allowInjection());
}

TEST(airmassInjectionGate, VersionChangeRequiresFreshPublication) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();

	auto oldVersion = beginPositiveCalculation();
	auto nextVersion = gate().beginCalculation(LM_SPEED_DENSITY, 1000, engine->getGlobalConfigurationVersion() + 1);
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_FALSE(gate().isCalculationCurrent(oldVersion));

	gate().completeCalculation(oldVersion, true);
	EXPECT_FALSE(gate().allowInjection());

	auto restoredVersion = gate().beginCalculation(LM_SPEED_DENSITY, 1000, engine->getGlobalConfigurationVersion());
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_NE(nextVersion, restoredVersion);
	EXPECT_TRUE(gate().isCalculationCurrent(restoredVersion));
	completeValid(restoredVersion);
	EXPECT_TRUE(gate().allowInjection());
}

TEST(airmassInjectionGate, FailedConfigurationValidationClosesAdmission) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();

	auto stale = beginPositiveCalculation();
	gate().acceptCalculation();
	ASSERT_TRUE(gate().isCalculationCurrent(stale));

	const auto generation = engine->airmassCalibration.Generation;
	const auto version = engine->getGlobalConfigurationVersion();
	config->veLoadBins[1] = config->veLoadBins[0];

	EXPECT_FATAL_ERROR(incrementGlobalConfigurationVersion());

	EXPECT_EQ(generation + 1, engine->airmassCalibration.Generation);
	EXPECT_EQ(version, engine->getGlobalConfigurationVersion());
	EXPECT_FALSE(gate().isCalculationCurrent(stale));
	EXPECT_FALSE(gate().allowInjection());

	gate().completeCalculation(stale, true);
	EXPECT_FALSE(gate().allowInjection()) << "a stale completion cannot reopen admission";
	EXPECT_EQ(Status::Legacy, gate().status());
}

TEST(airmassInjectionGate, FailedStartTimeoutClosesGateOnlyAfterTriggerTimeout) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	eth.setTriggerType(trigger_type_e::TT_ONE);
	engineConfiguration->isFasterEngineSpinUpEnabled = true;
	selectMode(LM_SPEED_DENSITY);

	eth.fireRise(1);
	ASSERT_EQ(SPINNING_UP, engine->rpmCalculator.getState());
	ASSERT_EQ(0, engine->rpmCalculator.getCachedRpm());

	makeReady();
	auto oldCalculation = beginPositiveCalculation();
	gate().acceptCalculation();
	const auto pending = gate().pendingCallbacks();

	// A recent trigger is still a start attempt, not an engine stop.
	engine->periodicSlowCallback();
	EXPECT_EQ(SPINNING_UP, engine->rpmCalculator.getState());
	EXPECT_TRUE(gate().isCalculationCurrent(oldCalculation));
	EXPECT_TRUE(gate().allowInjection());

	advanceTimeUs(10e6);
	ASSERT_FALSE(engine->triggerCentral.engineMovedRecently(getTimeNowNt()));
	engine->periodicSlowCallback();

	EXPECT_EQ(STOPPED, engine->rpmCalculator.getState());
	EXPECT_FALSE(gate().isCalculationCurrent(oldCalculation));
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_EQ(pending, gate().pendingCallbacks());

	// A fresh completed standalone calculation automatically recovers admission.
	makeReady();
	EXPECT_TRUE(gate().allowInjection());
}

TEST(airmassInjectionGate, LegacyModesAndPrimeRemainIndependentlyAdmitted) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	resetTrackedCallbacks();

	for (auto mode : {LM_LUA, LM_MOCK}) {
		SCOPED_TRACE(static_cast<int>(mode));
		selectMode(mode);
		EXPECT_TRUE(gate().allowInjection());
		EXPECT_TRUE(gate().allowPrime());
		EXPECT_EQ(Status::Legacy, gate().status());
	}

	selectMode(LM_SPEED_DENSITY);
	EXPECT_FALSE(gate().allowInjection());
	ASSERT_TRUE(queueTrackedPulse(100, 200, true));
	EXPECT_EQ(2, gate().pendingCallbacks());
	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_EQ(1, startedCallbacks);
	EXPECT_EQ(1, completedCallbacks);
	EXPECT_EQ(0, gate().pendingCallbacks());
}

TEST(airmassInjectionGate, AcceptedCallbacksDrainAfterAdmissionCloses) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();
	resetTrackedCallbacks();

	ASSERT_TRUE(queueTrackedPulse(100, 200));
	EXPECT_EQ(2, gate().pendingCallbacks());
	gate().rejectCalculation(Fault::Load);
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_FALSE(queueTrackedPulse(100, 200));
	EXPECT_EQ(2, gate().pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_EQ(1, startedCallbacks);
	EXPECT_EQ(1, completedCallbacks);
	EXPECT_EQ(0, gate().pendingCallbacks());
}

TEST(airmassInjectionGate, ScheduleBatchRejectionRollsBackAccountAndQueuesNoEvents) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();
	resetTrackedCallbacks();

	for (size_t i = 0; i < 64; i++) {
		getScheduler()->schedule("fill", nullptr, getTimeNowNt() + US2NT(10000), {+[](void*) {}, nullptr});
	}
	ASSERT_EQ(64, engine->scheduler.size());

	EXPECT_FALSE(queueTrackedPulse(100, 200));
	EXPECT_EQ(0, gate().pendingCallbacks());
	EXPECT_EQ(64, engine->scheduler.size());
	EXPECT_EQ(0, startedCallbacks);
	EXPECT_EQ(0, completedCallbacks);
	EXPECT_EQ(Status::Faulted, gate().status());
	EXPECT_EQ(Fault::Scheduling, gate().fault());

	eth.moveTimeForwardAndInvokeEventsUs(10000);
	makeReady();
	ASSERT_TRUE(queueTrackedPulse(100, 200));
	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_EQ(1, startedCallbacks);
	EXPECT_EQ(1, completedCallbacks);
	EXPECT_EQ(0, gate().pendingCallbacks());
}

TEST(airmassInjectionGate, ConfigurationWriteRetainsAcceptedCallbacks) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();
	resetTrackedCallbacks();

	ASSERT_TRUE(queueTrackedPulse(100, 200));
	ASSERT_EQ(2, gate().pendingCallbacks());

	gate().onConfigurationWrite(LM_SPEED_DENSITY, false);
	EXPECT_FALSE(gate().allowInjection());
	EXPECT_EQ(2, gate().pendingCallbacks());
	EXPECT_FALSE(queueTrackedPulse(100, 200));
	EXPECT_EQ(2, gate().pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(200);
	EXPECT_EQ(1, startedCallbacks);
	EXPECT_EQ(1, completedCallbacks);
	EXPECT_EQ(0, gate().pendingCallbacks());
}

TEST(airmassInjectionGate, ImmediateCallbacksObserveAccountedCallbacks) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();
	resetTrackedCallbacks();

	class ImmediateExecutor : public Scheduler {
	public:
		void schedule(const char*, scheduling_s*, efitick_t, action_s) override {
			ADD_FAILURE() << "fuel must use an atomic batch";
		}
		void cancel(scheduling_s*) override {}
		bool scheduleBatch(const ScheduledAction* events, size_t count) override {
			EXPECT_EQ(2u, count);
			EXPECT_EQ(2, gate().pendingCallbacks());
			auto start = events[0].action;
			start.execute();
			EXPECT_EQ(1, gate().pendingCallbacks());
			auto end = events[1].action;
			end.execute();
			EXPECT_EQ(0, gate().pendingCallbacks());
			return true;
		}
	} executor;

	engine->scheduler.setMockExecutor(&executor);
	EXPECT_TRUE(queueTrackedPulse(0, 1000));
	EXPECT_EQ(1, startedCallbacks);
	EXPECT_EQ(1, completedCallbacks);
	EXPECT_EQ(0, gate().pendingCallbacks());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(airmassInjectionGate, CallbackCounterOverflowRejectsWithoutInsertion) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	selectMode(LM_SPEED_DENSITY);
	makeReady();
	resetTrackedCallbacks();

	{
		chibios_rt::CriticalSectionLocker csl;
		ASSERT_TRUE(gate().locked(csl).callbacksAccepted(UINT16_MAX - 1));
	}

	EXPECT_FALSE(queueTrackedPulse(100, 200));
	EXPECT_EQ(UINT16_MAX - 1, gate().pendingCallbacks());
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(Fault::Scheduling, gate().fault());

	{
		chibios_rt::CriticalSectionLocker csl;
		gate().locked(csl).callbacksRejected(UINT16_MAX - 1);
	}
	EXPECT_EQ(0, gate().pendingCallbacks());
}
