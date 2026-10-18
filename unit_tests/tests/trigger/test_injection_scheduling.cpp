#include "pch.h"
#include "airmass_injection_state.h"
#include "main_trigger_callback.h"
#include "injector_model.h"

using ::testing::_;
using ::testing::InSequence;
using ::testing::StrictMock;

namespace {
struct ExpectedScheduledAction {
	efitick_t time;
	schfunc_t callback;
	void* argument;
};

void expectBatch(MockExecutor& executor, std::initializer_list<ExpectedScheduledAction> expected) {
	std::vector<ExpectedScheduledAction> copied(expected);
	EXPECT_CALL(executor, scheduleBatch(_, copied.size()))
			.WillOnce([copied](const ScheduledAction* events, size_t count) {
				EXPECT_EQ(count, copied.size());
				for (size_t i = 0; i < count; i++) {
					EXPECT_EQ(events[i].time, copied[i].time);
					EXPECT_EQ(events[i].action.getCallback(), copied[i].callback);
					EXPECT_EQ(events[i].action.getArgument(), copied[i].argument);
				}
				// The mock does not retain callbacks, so release their test accounting.
				chibios_rt::CriticalSectionLocker csl;
				engine->airmassInjectionState.locked(csl).callbacksRejected(count);
				return true;
			});
}
} // namespace

TEST(injectionScheduling, InjectionIsScheduled) {
	StrictMock<MockExecutor> mockExec;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);

	efitick_t nowNt = 1000000;

	InjectionEvent event;

	// Injection duration of 20ms
	MockInjectorModel2 im;
	EXPECT_CALL(im, getInjectionDuration(_)).WillOnce(Return(20.0f));
	engine->module<InjectorModelPrimary>().set(&im);

	engine->rpmCalculator.oneDegreeUs = 100;

	InjectorContext ctx;
	ctx.outputsMask = (1 << 0);
	ctx.eventIndex = 0;
	ctx.stage2Active = false;

	void* ctxAsPtr = bit_cast<void*>(ctx);

	// Should reserve one normal injection as an atomic batch.
	float nt5deg = USF2NT(engine->rpmCalculator.oneDegreeUs * 5);
	efitick_t startTime = nowNt + nt5deg;
	efitick_t endTime = startTime + MS2NT(20);
	expectBatch(
			mockExec,
			{{startTime, (schfunc_t)scheduledStartInjection, ctxAsPtr},
			 {endTime, (schfunc_t)scheduledEndInjection, ctxAsPtr}});

	// Event scheduled at 125 degrees
	event.injectionStartAngle = 125;

	// We are at 120 degrees now, next tooth 130
	event.onTriggerTooth({nowNt, 0, 0, 120, 130});
}

TEST(injectionScheduling, InjectionIsScheduledDualStage) {
	StrictMock<MockExecutor> mockExec;
	StrictMock<MockInjectorModel2> im;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);
	engine->module<InjectorModelPrimary>().set(&im);
	engine->module<InjectorModelSecondary>().set(&im);

	efitick_t nowNt = 1000000;

	InjectionEvent event;

	InjectorContext ctx;
	ctx.outputsMask = (1 << 0);
	ctx.eventIndex = 0;
	ctx.stage2Active = true;

	void* ctxAsPtr = bit_cast<void*>(ctx);

	engine->rpmCalculator.oneDegreeUs = 100;

	// Some nonzero fuel quantity on both stages
	engine->cylinders[0].setInjectionMass(50);
	engine->engineState.injectionStage2Fraction = 0.2;

	{
		InSequence is;

		// Primary injection duration of 20ms, secondary 10ms
		EXPECT_CALL(im, getInjectionDuration(40)).WillOnce(Return(20.0f));
		EXPECT_CALL(im, getInjectionDuration(10)).WillOnce(Return(10.0f));
	}

	// The two closing edges are sorted chronologically inside the atomic batch.
	float nt5deg = USF2NT(engine->rpmCalculator.oneDegreeUs * 5);
	efitick_t startTime = nowNt + nt5deg;
	efitick_t stage2EndTime = startTime + MS2NT(10);
	efitick_t primaryEndTime = startTime + MS2NT(20);
	expectBatch(
			mockExec,
			{{startTime, (schfunc_t)scheduledStartInjection, ctxAsPtr},
			 {stage2EndTime, (schfunc_t)scheduledEndInjectionStage2, ctxAsPtr},
			 {primaryEndTime, (schfunc_t)scheduledEndInjection, ctxAsPtr}});

	// Event scheduled at 125 degrees
	event.injectionStartAngle = 125;

	// We are at 120 degrees now, next tooth 130
	event.onTriggerTooth({nowNt, 0, 0, 120, 130});
}

TEST(injectionScheduling, InjectionIsScheduledBeforeWraparound) {
	StrictMock<MockExecutor> mockExec;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);

	efitick_t nowNt = 1000000;

	InjectionEvent event;

	InjectorContext ctx;
	ctx.outputsMask = (1 << 0);
	ctx.eventIndex = 0;
	ctx.stage2Active = false;

	void* ctxAsPtr = bit_cast<void*>(ctx);

	// Injection duration of 20ms
	MockInjectorModel2 im;
	EXPECT_CALL(im, getInjectionDuration(_)).WillOnce(Return(20.0f));
	engine->module<InjectorModelPrimary>().set(&im);

	engine->rpmCalculator.oneDegreeUs = 100;

	float nt5deg = USF2NT(engine->rpmCalculator.oneDegreeUs * 5);
	efitick_t startTime = nowNt + nt5deg;
	efitick_t endTime = startTime + MS2NT(20);
	expectBatch(
			mockExec,
			{{startTime, (schfunc_t)scheduledStartInjection, ctxAsPtr},
			 {endTime, (schfunc_t)scheduledEndInjection, ctxAsPtr}});

	// Event scheduled at 715 degrees
	event.injectionStartAngle = 715;

	// We are at 710 degrees now, next tooth 010
	event.onTriggerTooth({nowNt, 0, 0, 710, 010});
}

TEST(injectionScheduling, InjectionIsScheduledAfterWraparound) {
	StrictMock<MockExecutor> mockExec;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);

	efitick_t nowNt = 1000000;

	InjectionEvent event;

	InjectorContext ctx;
	ctx.outputsMask = (1 << 0);
	ctx.eventIndex = 0;
	ctx.stage2Active = false;

	void* ctxAsPtr = bit_cast<void*>(ctx);

	// Injection duration of 20ms
	MockInjectorModel2 im;
	EXPECT_CALL(im, getInjectionDuration(_)).WillOnce(Return(20.0f));
	engine->module<InjectorModelPrimary>().set(&im);

	engine->rpmCalculator.oneDegreeUs = 100;

	float nt5deg = USF2NT(engine->rpmCalculator.oneDegreeUs * 15);
	efitick_t startTime = nowNt + nt5deg;
	efitick_t endTime = startTime + MS2NT(20);
	expectBatch(
			mockExec,
			{{startTime, (schfunc_t)scheduledStartInjection, ctxAsPtr},
			 {endTime, (schfunc_t)scheduledEndInjection, ctxAsPtr}});

	// Event scheduled at 5 degrees
	event.injectionStartAngle = 5;

	// We are at 710 degrees now, next tooth 010
	event.onTriggerTooth({nowNt, 0, 0, 710, 010});
}

TEST(injectionScheduling, InjectionNotScheduled) {
	// StrictMock since we expect no scheduler calls!
	StrictMock<MockExecutor> mockExec;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);

	efitick_t nowNt = 1000000;

	InjectionEvent event;

	// Expect no calls to injector model
	StrictMock<MockInjectorModel2> im;
	engine->module<InjectorModelPrimary>().set(&im);

	engine->rpmCalculator.oneDegreeUs = 100;

	{
		InSequence is;

		// Expect no scheduler calls!
	}

	// Event scheduled at 125 degrees
	event.injectionStartAngle = 125;

	// We are at 130 degrees now, next tooth 140
	event.onTriggerTooth({nowNt, 0, 0, 130, 140});
}

TEST(injectionScheduling, SplitContinuationReservesBeforeEndAcknowledges) {
	StrictMock<MockExecutor> mockExec;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&mockExec);

	InjectorContext ctx;
	ctx.outputsMask = 0;
	ctx.eventIndex = 0;
	ctx.stage2Active = false;

	void* ctxAsPtr = bit_cast<void*>(ctx);

	// Split injection events should be called with no remaining split duration
	ctx.splitDurationUs = 0;

	// Reserve the original end callback before invoking its tracked wrapper.
	{
		chibios_rt::CriticalSectionLocker csl;
		ASSERT_TRUE(engine->airmassInjectionState.locked(csl).callbacksAccepted(1));
	}

	efitick_t nowNt = getTimeNowNt();
	efitick_t startTime = nowNt + MS2NT(2);
	efitick_t endTime = startTime + MS2NT(10);
	EXPECT_CALL(mockExec, scheduleBatch(_, 2)).WillOnce([&](const ScheduledAction* events, size_t count) {
		EXPECT_EQ(3, engine->airmassInjectionState.pendingCallbacks())
				<< "split pair must be reserved before the original end ACK";
		EXPECT_EQ(startTime, events[0].time);
		EXPECT_EQ((schfunc_t)scheduledStartInjection, events[0].action.getCallback());
		EXPECT_EQ(endTime, events[1].time);
		EXPECT_EQ((schfunc_t)scheduledEndInjection, events[1].action.getCallback());
		EXPECT_EQ(ctxAsPtr, events[0].action.getArgument());
		EXPECT_EQ(ctxAsPtr, events[1].action.getArgument());
		chibios_rt::CriticalSectionLocker csl;
		engine->airmassInjectionState.locked(csl).callbacksRejected(count);
		return true;
	});

	// Split injection duration of 10ms
	ctx.splitDurationUs = 10000;

	// Close and reserve the next pair before acknowledging this callback.
	scheduledEndInjection(ctx);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

namespace {
void makeStandaloneReady() {
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	Sensor::setMockValue(SensorType::Rpm, 1000);
	auto token = engine->airmassInjectionState.beginCalculation(
			LM_SPEED_DENSITY, 1000, engine->getGlobalConfigurationVersion());
	engine->engineState.airmassCalculationValid = true;
	engine->airmassInjectionState.acceptCalculation();
	engine->airmassInjectionState.completeCalculation(token, true);
	ASSERT_TRUE(engine->airmassInjectionState.allowInjection());
}

InjectorContext injectorContext(uint16_t mask = 1) {
	InjectorContext ctx;
	ctx.outputsMask = mask;
	ctx.eventIndex = 0;
	return ctx;
}

bool queuePulse(const InjectorContext& ctx, int startUs, int endUs) {
	ScheduledAction events[] = {
			{getTimeNowNt() + US2NT(startUs), {scheduledStartInjection, ctx}},
			{getTimeNowNt() + US2NT(endUs), {scheduledEndInjection, ctx}},
	};
	return scheduleFuelCallbacks(events, efi::size(events));
}

class FuelBatchExecutor : public Scheduler {
public:
	bool reject = false;
	std::vector<size_t> batches;

	void schedule(const char*, scheduling_s*, efitick_t, action_s) override {
		ADD_FAILURE() << "fuel callbacks must use an atomic batch";
	}

	void cancel(scheduling_s*) override {}

	bool scheduleBatch(const ScheduledAction* events, size_t count) override {
		batches.push_back(count);
		if (reject) {
			return false;
		}

		// Callbacks may run before scheduleBatch returns, but only after accounting.
		EXPECT_GE(engine->airmassInjectionState.pendingCallbacks(), count);
		ScheduledAction copy[MaxScheduleBatchSize];
		for (size_t i = 0; i < count; i++) {
			copy[i] = events[i];
		}
		for (size_t i = 0; i < count; i++) {
			copy[i].action.execute();
		}
		return true;
	}
};
} // namespace

TEST(injectionScheduling, InlineCallbacksObserveReservedAccounting) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	makeStandaloneReady();
	FuelBatchExecutor executor;
	engine->scheduler.setMockExecutor(&executor);

	ASSERT_TRUE(queuePulse(injectorContext(), 0, 1000));
	EXPECT_EQ((std::vector<size_t>{2}), executor.batches);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(injectionScheduling, InterleavedSplitReservesPairBeforeOriginalAck) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	makeStandaloneReady();
	FuelBatchExecutor executor;
	engine->scheduler.setMockExecutor(&executor);

	InjectorContext ctx = injectorContext();
	ctx.splitDurationUs = 1000;
	{
		chibios_rt::CriticalSectionLocker csl;
		ASSERT_TRUE(engine->airmassInjectionState.locked(csl).callbacksAccepted(1));
	}

	scheduledEndInjection(ctx);
	EXPECT_EQ((std::vector<size_t>{2}), executor.batches);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(injectionScheduling, RejectedBatchLeavesNoAccountingOrOutput) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	makeStandaloneReady();
	FuelBatchExecutor executor;
	executor.reject = true;
	engine->scheduler.setMockExecutor(&executor);

	EXPECT_FALSE(queuePulse(injectorContext(), 0, 1000));
	EXPECT_EQ((std::vector<size_t>{2}), executor.batches);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(injectionScheduling, RejectedSplitReservationDoesNotLeakAccounting) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	makeStandaloneReady();
	FuelBatchExecutor executor;
	executor.reject = true;
	engine->scheduler.setMockExecutor(&executor);

	InjectorContext ctx = injectorContext();
	ctx.splitDurationUs = 1000;
	{
		chibios_rt::CriticalSectionLocker csl;
		ASSERT_TRUE(engine->airmassInjectionState.locked(csl).callbacksAccepted(1));
	}

	scheduledEndInjection(ctx);
	EXPECT_EQ((std::vector<size_t>{2}), executor.batches);
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter());
	engine->scheduler.setMockExecutor(nullptr);
}

TEST(injectionScheduling, ClosedAdmissionDoesNotCalculateNewPulse) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->fuelAlgorithm = LM_SPEED_DENSITY;
	StrictMock<MockInjectorModel2> model;
	engine->module<InjectorModelPrimary>().set(&model);
	engine->cylinders[0].setInjectionMass(1);
	engine->rpmCalculator.oneDegreeUs = 100;
	ASSERT_FALSE(engine->airmassInjectionState.allowInjection());

	InjectionEvent event;
	event.injectionStartAngle = 5;
	event.onTriggerTooth({getTimeNowNt(), 0, 0, 0, 10});

	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}

TEST(injectionScheduling, StopKeepsAcceptedPulseObligations) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	makeStandaloneReady();
	auto ctx = injectorContext();
	ctx.splitDurationUs = 1000;

	ASSERT_TRUE(queuePulse(ctx, 100, 200));
	ASSERT_EQ(2, engine->airmassInjectionState.pendingCallbacks());
	engine->airmassInjectionState.onEngineStop();
	EXPECT_FALSE(engine->airmassInjectionState.allowInjection());
	EXPECT_EQ(2, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_FALSE(queuePulse(ctx, 300, 400));
	EXPECT_EQ(2, engine->airmassInjectionState.pendingCallbacks());

	eth.moveTimeForwardAndInvokeEventsUs(100);
	EXPECT_EQ(1, enginePins.injectors[0].getOverlappingCounter()) << "an accepted start must still open its owed pulse";
	eth.moveTimeForwardAndInvokeEventsUs(100);
	EXPECT_EQ(0, enginePins.injectors[0].getOverlappingCounter()) << "an accepted end must still close its pulse";
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	EXPECT_EQ(0, engine->scheduler.size()) << "stop must reject the split continuation";
}

TEST(injectionScheduling, InvalidDurationsAndDelaysNeverReachScheduler) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	MockInjectorModel2 model;
	engine->module<InjectorModelPrimary>().set(&model);
	engine->cylinders[0].setInjectionMass(1);
	engine->engineState.injectionStage2Fraction = 0;
	engine->rpmCalculator.oneDegreeUs = 100;

	InjectionEvent event;
	event.injectionStartAngle = 5;
	const EnginePhaseInfo phase{getTimeNowNt(), 0, 0, 0, 10};

	for (float duration : {NAN, INFINITY, -1.0f, 10000.001f}) {
		EXPECT_CALL(model, getInjectionDuration(_)).WillOnce(Return(duration));
		event.onTriggerTooth(phase);
		EXPECT_EQ(0, engine->scheduler.size());
		EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
	}

	EXPECT_CALL(model, getInjectionDuration(_)).WillOnce(Return(2.0f));
	engine->rpmCalculator.oneDegreeUs = NAN;
	event.onTriggerTooth(phase);
	EXPECT_EQ(0, engine->scheduler.size());

	EXPECT_CALL(model, getInjectionDuration(_)).WillOnce(Return(2.0f));
	engine->rpmCalculator.oneDegreeUs = MaximumScheduleDelayUs;
	event.onTriggerTooth(phase);
	EXPECT_EQ(0, engine->scheduler.size());
	EXPECT_EQ(0, engine->airmassInjectionState.pendingCallbacks());
}
