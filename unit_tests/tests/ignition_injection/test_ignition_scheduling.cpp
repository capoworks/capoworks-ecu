/*
 * @file test_ignition_scheduling.cpp
 *
 * @date Nov 17, 2019
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"
#include "spark_logic.h"

using ::testing::_;

TEST(ignition, twoCoils) {
	EngineTestHelper eth(engine_type_e::FRANKENSO_BMW_M73_F);

	// let's recalculate with zero timing so that we can focus on relation advance between cylinders
	for (auto& c : engine->cylinders) {
		c.setIgnitionTimingBtdc(0);
	}
	initializeIgnitionActions();

	// first one to fire uses first coil
	EXPECT_EQ(engine->ignitionEvents.elements[0].calculateIgnitionOutputMask(), (1 << 0));
	// each subsequent event fires coil 1/6 alternating
	EXPECT_EQ(engine->ignitionEvents.elements[1].calculateIgnitionOutputMask(), (1 << 6));
	EXPECT_EQ(engine->ignitionEvents.elements[2].calculateIgnitionOutputMask(), (1 << 0));
	EXPECT_EQ(engine->ignitionEvents.elements[3].calculateIgnitionOutputMask(), (1 << 6));
	EXPECT_EQ(engine->ignitionEvents.elements[4].calculateIgnitionOutputMask(), (1 << 0));
	EXPECT_EQ(engine->ignitionEvents.elements[5].calculateIgnitionOutputMask(), (1 << 6));
	EXPECT_EQ(engine->ignitionEvents.elements[6].calculateIgnitionOutputMask(), (1 << 0));
	EXPECT_EQ(engine->ignitionEvents.elements[7].calculateIgnitionOutputMask(), (1 << 6));
	EXPECT_EQ(engine->ignitionEvents.elements[8].calculateIgnitionOutputMask(), (1 << 0));
	EXPECT_EQ(engine->ignitionEvents.elements[9].calculateIgnitionOutputMask(), (1 << 6));
	EXPECT_EQ(engine->ignitionEvents.elements[10].calculateIgnitionOutputMask(), (1 << 0));
	EXPECT_EQ(engine->ignitionEvents.elements[11].calculateIgnitionOutputMask(), (1 << 6));

	ASSERT_EQ(engine->ignitionEvents.elements[0].calculateSparkAngle(), 0);
	ASSERT_EQ(engine->ignitionEvents.elements[0].calculateIgnitionOutputMask(), (1 << 0));

	ASSERT_EQ(engine->ignitionEvents.elements[1].calculateSparkAngle(), 720 / 12);
	ASSERT_EQ(engine->ignitionEvents.elements[1].calculateIgnitionOutputMask(), (1 << 6));

	ASSERT_EQ(engine->ignitionEvents.elements[3].calculateSparkAngle(), 3 * 720 / 12);
	ASSERT_EQ(engine->ignitionEvents.elements[3].calculateIgnitionOutputMask(), (1 << 6));
}

TEST(ignition, trailingSpark) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	engineConfiguration->isFasterEngineSpinUpEnabled = false;

	EXPECT_CALL(*eth.mockAirmass, getAirmass(_, _)).WillRepeatedly(Return(AirmassResult{0.1008f, 50.0f}));

	setupSimpleTestEngineWithMafAndTT_ONE_trigger(&eth);
	setCylinderCount(1);
	engineConfiguration->firingOrder = FO_1;
	engineConfiguration->isInjectionEnabled = false;
	engineConfiguration->isIgnitionEnabled = true;

	// Fire trailing spark 10 degrees after main spark
	engine->engineState.trailingSparkAngle = 10;

	engineConfiguration->injectionMode = IM_SEQUENTIAL;

	setWholeTimingTable(0);

	eth.fireTriggerEventsWithDuration(20);
	// still no RPM since need to cycles measure cycle duration
	eth.fireTriggerEventsWithDuration(20);
	ASSERT_EQ(3000, Sensor::getOrZero(SensorType::Rpm)) << "RPM#0";

	/**
	 * Trigger up - scheduling fuel for full engine cycle
	 */
	eth.smartFireRise(20);

	// Primary coil should be high
	EXPECT_EQ(enginePins.coils[0].getLogicValue(), true);
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), false);

	// TDC callback, spark firing, and independent physical dwell guard.
	EXPECT_EQ(engine->scheduler.size(), 3);

	// execute all actions
	eth.executeActions();

	// Primary and secondary coils should be low - primary just fired
	EXPECT_EQ(enginePins.coils[0].getLogicValue(), false);
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), false);

	// Now enable trailing sparks
	engineConfiguration->enableTrailingSparks = true;

	// Fire trigger fall - should schedule ignition chargings (rising edges)
	eth.fireFall(20);
	eth.moveTimeForwardMs(18);
	eth.executeActions();

	// Primary low, scheduling trailing
	EXPECT_EQ(enginePins.coils[0].getLogicValue(), true);
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), false);

	eth.moveTimeForwardMs(2);
	eth.executeActions();

	// and secondary coils should be low
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), true);

	// Fire trigger rise - should schedule ignition firings
	eth.fireRise(0);
	eth.moveTimeForwardMs(1);
	eth.executeActions();

	// Primary goes low, scheduling trailing
	EXPECT_EQ(enginePins.coils[0].getLogicValue(), false);
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), true);

	eth.moveTimeForwardMs(1);
	eth.executeActions();
	// secondary coils should be low
	EXPECT_EQ(enginePins.trailingCoils[0].getLogicValue(), false);
}

TEST(ignition, CylinderTimingTrim) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	// Base timing 15 degrees
	setTable(config->ignitionTable, 15);

	// negative numbers retard timing, positive advance
	setTable(config->ignTrims[0].table, -4);
	setTable(config->ignTrims[1].table, -2);
	setTable(config->ignTrims[2].table, 2);
	setTable(config->ignTrims[3].table, 4);

	// run the ignition math
	engine->periodicFastCallback();

	// Check that each cylinder gets the expected timing
	float unadjusted = 15;
	EXPECT_NEAR(engine->cylinders[0].getIgnitionTimingBtdc(), unadjusted - 4, EPS4D);
	EXPECT_NEAR(engine->cylinders[1].getIgnitionTimingBtdc(), unadjusted - 2, EPS4D);
	EXPECT_NEAR(engine->cylinders[2].getIgnitionTimingBtdc(), unadjusted + 2, EPS4D);
	EXPECT_NEAR(engine->cylinders[3].getIgnitionTimingBtdc(), unadjusted + 4, EPS4D);
}

TEST(ignition, oddCylinderWastedSpark) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);
	setCylinderCount(1);
	engineConfiguration->firingOrder = FO_1;
	engineConfiguration->ignitionMode = IM_WASTED_SPARK;
	engine->rpmCalculator.oneDegreeUs = 100;
	engine->ignitionState.sparkDwell = 1;
	engine->ignitionState.dwellAngle = 10;
	engine->cylinders[0].setIgnitionTimingBtdc(-25);
	engine->engineState.useOddFireWastedSpark = true;
	engineConfiguration->minimumIgnitionTiming = -25;
	// Drain actual physical occurrences between 0 and 360 degree opportunities.
	// A mock that never executes the first LOW must not permit record reuse.
	for (float offset : {0.0f, 360.0f}) {
		onTriggerEventSparkLogic({getTimeNowNt(), 10 + offset, 30 + offset, 10 + offset, 30 + offset});
		eth.moveTimeForwardAndInvokeEventsUs(499);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		eth.moveTimeForwardAndInvokeEventsUs(1);
		EXPECT_TRUE(enginePins.coils[0].getLogicValue());
		eth.moveTimeForwardAndInvokeEventsUs(1000);
		EXPECT_FALSE(enginePins.coils[0].getLogicValue());
		EXPECT_EQ(0, engine->scheduler.size());
	}
}
