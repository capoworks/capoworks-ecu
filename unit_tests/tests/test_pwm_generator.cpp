/*
 * test_pwm_generator.cpp
 *
 *  @date Dec 8, 2018
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#define LOW_VALUE 0
#define HIGH_VALUE 1
static int expectedTimeOfNextEvent;

static void assertNextEvent(const char* msg, int expectedPinState, TestExecutor* executor, OutputPin& pin) {
	printf("PWM_test: Asserting event [%s]\r\n", msg);
	// only one action expected in queue
	ASSERT_EQ(1, executor->size()) << "PWM_test: schedulingQueue size";

	// move time to next event timestamp
	setTimeNowUs(expectedTimeOfNextEvent);

	// execute pending actions and assert that only one action was executed
	ASSERT_EQ(1, executor->executeAll(getTimeNowUs())) << msg << " executed";
	ASSERT_EQ(expectedPinState, pin.m_currentLogicValue) << msg << " pin state";

	// assert that we have one new action in queue
	ASSERT_EQ(1, executor->size()) << "PWM_test: queue.size";
}

TEST(PWM, test100dutyCycle) {
	printf("*************************************** test100dutyCycle\r\n");

	expectedTimeOfNextEvent = 0;
	setTimeNowUs(0);

	OutputPin pin;
	SimplePwm pwm("test PWM1");
	TestExecutor executor;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&executor);

	startSimplePwm(&pwm, "unit_test", &pin, 1000 /* frequency */, 1.0 /* duty cycle */);

	expectedTimeOfNextEvent += 1000;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@100", HIGH_VALUE, &executor, pin);

	expectedTimeOfNextEvent += 1000;
	assertNextEvent("exec2@100", HIGH_VALUE, &executor, pin);

	expectedTimeOfNextEvent += 1000;
	assertNextEvent("exec3@100", HIGH_VALUE, &executor, pin);
}

TEST(PWM, testSwitchToNanPeriod) {
	expectedTimeOfNextEvent = 0;
	setTimeNowUs(0);

	OutputPin pin;
	SimplePwm pwm("test PWM1");
	TestExecutor executor;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&executor);

	startSimplePwm(&pwm, "unit_test", &pin, 1000 /* frequency */, 0.60 /* duty cycle */);

	expectedTimeOfNextEvent += 600;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@70", LOW_VALUE, &executor, pin);
	ASSERT_EQ(600, getTimeNowUs()) << "time1";

	expectedTimeOfNextEvent += 400;
	assertNextEvent("exec2@70", HIGH_VALUE, &executor, pin);

	pwm.setFrequency(NAN);

	expectedTimeOfNextEvent += 600;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);
	assertNextEvent("exec2@NAN", LOW_VALUE, &executor, pin);

	expectedTimeOfNextEvent += MS2US(NAN_FREQUENCY_SLEEP_PERIOD_MS);
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);
	assertNextEvent("exec3@NAN", LOW_VALUE, &executor, pin);
}

TEST(PWM, RepeatedStopRestartBalancesCallbackNesting) {
	OutputPin pin;
	SimplePwm pwm("restart test");
	TestExecutor executor;
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&executor);

	// A stopped callback must leave no nesting behind for the next start.
	// The former leak reached the fatal nesting limit after 25 starts.
	for (int cycle = 0; cycle < 40; cycle++) {
		SCOPED_TRACE(cycle);
		startSimplePwm(&pwm, "restart test", &pin, 1000, 0.6f);
		ASSERT_EQ(0, pwm.dbgNestingLevel);
		ASSERT_EQ(1, executor.size());
		EXPECT_EQ(HIGH_VALUE, pin.m_currentLogicValue);

		setTimeNowUs(executor.getForUnitTest(0)->momentX);
		ASSERT_EQ(1, executor.executeAll(getTimeNowUs()));
		ASSERT_EQ(1, executor.size());
		EXPECT_EQ(LOW_VALUE, pin.m_currentLogicValue);

		pwm.stop();
		setTimeNowUs(executor.getForUnitTest(0)->momentX);
		ASSERT_EQ(1, executor.executeAll(getTimeNowUs()));
		ASSERT_EQ(0, executor.size());
		ASSERT_EQ(0, pwm.dbgNestingLevel);
	}
}

TEST(PWM, testPwmGenerator) {
	expectedTimeOfNextEvent = 0;
	setTimeNowUs(0);

	OutputPin pin;
	SimplePwm pwm("test PWM3");
	TestExecutor executor;

	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engine->scheduler.setMockExecutor(&executor);

	startSimplePwm(&pwm, "unit_test", &pin, 1000 /* frequency */, 0.80 /* duty cycle */);

	expectedTimeOfNextEvent += 800;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@0", LOW_VALUE, &executor, pin);
	ASSERT_EQ(800, getTimeNowUs()) << "time1";

	expectedTimeOfNextEvent += 200;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	// above we had vanilla duty cycle, now let's handle a special case
	pwm.setSimplePwmDutyCycle(0);
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@1", LOW_VALUE, &executor, pin);
	ASSERT_EQ(1000, getTimeNowUs()) << "time2";

	expectedTimeOfNextEvent += 1000;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@2", LOW_VALUE /* pin value */, &executor, pin);
	ASSERT_EQ(2000, getTimeNowUs()) << "time3";
	expectedTimeOfNextEvent += 1000;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@3", LOW_VALUE /* pin value */, &executor, pin);
	ASSERT_EQ(3000, getTimeNowUs()) << "time4";
	expectedTimeOfNextEvent += 1000;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@4", LOW_VALUE /* pin value */, &executor, pin);
	expectedTimeOfNextEvent += 1000;
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@5", LOW_VALUE /* pin value */, &executor, pin);
	expectedTimeOfNextEvent += 1000;
	ASSERT_EQ(5000, getTimeNowUs()) << "time4";
	EXPECT_EQ(expectedTimeOfNextEvent, executor.getForUnitTest(0)->momentX);

	assertNextEvent("exec@6", LOW_VALUE /* pin value */, &executor, pin);
}

TEST(PWM, ImmediateTerminalDutyPrecedesQueuedCallback) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	TestExecutor executor;
	engine->scheduler.setMockExecutor(&executor);
	OutputPin pin;
	SimplePwm pwm;
	startSimplePwm(&pwm, "immediate PWM", &pin, 800, 0.6f);
	ASSERT_TRUE(pin.getLogicValue());
	auto executeNext = [&]() {
		ASSERT_EQ(executor.size(), 1u);
		auto next = executor.getForUnitTest(0)->momentX;
		setTimeNowUs(next);
		EXPECT_EQ(executor.executeAll(next), 1u);
	};

	pwm.setDutyImmediate(0);
	EXPECT_FALSE(pin.getLogicValue());
	executeNext();
	EXPECT_FALSE(pin.getLogicValue());
	pwm.setDutyImmediate(1);
	EXPECT_TRUE(pin.getLogicValue());
	executeNext();
	EXPECT_TRUE(pin.getLogicValue());

	// A normal command can resume PWM after forcing a terminal duty.
	pwm.setSimplePwmDutyCycle(0.6f);
	executeNext();
	EXPECT_TRUE(pin.getLogicValue());
	executeNext();
	EXPECT_FALSE(pin.getLogicValue());
	executeNext();
	EXPECT_TRUE(pin.getLogicValue());

	// stop() only cancels at the queued callback. Immediate duty must still
	// reach the pin while that cancellation is pending.
	pwm.stop();
	pwm.setDutyImmediate(0);
	EXPECT_FALSE(pin.getLogicValue());
	executeNext();
	EXPECT_FALSE(pin.getLogicValue());
	EXPECT_EQ(executor.size(), 0u);
}
