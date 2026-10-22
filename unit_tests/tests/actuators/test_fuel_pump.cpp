/**
 * @file test_logic_expression.cpp
 *
 * https://sourceforge.net/p/rusefi/tickets/102/
 *
 * @date Oct 3, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

TEST(Actuators, FuelPump) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	FuelPumpController dut;

	// Mock a fuel pump pin
	engineConfiguration->fuelPumpPin = Gpio::A0;
	engineConfiguration->startUpFuelPumpDuration = 2;
	// Re-init so it picks up the new config
	enginePins.fuelPumpRelay.init();

	// ECU just started, haven't seen trigger yet
	setTimeNowUs(0.5e6);
	dut.onIgnitionStateChanged(true);
	dut.onSlowCallback();
	// Pump should be on!
	EXPECT_TRUE(efiReadPin(Gpio::A0));

	// Prime must stop on the first slow callback after ignition turns off
	dut.onIgnitionStateChanged(false);
	dut.onSlowCallback();
	EXPECT_FALSE(efiReadPin(Gpio::A0));

	// Turning ignition back on starts a fresh priming interval
	advanceTimeUs(2.1e6);
	dut.onSlowCallback();
	EXPECT_FALSE(efiReadPin(Gpio::A0));
	dut.onIgnitionStateChanged(true);
	dut.onSlowCallback();
	EXPECT_TRUE(efiReadPin(Gpio::A0));
	advanceTimeUs(1.9e6);
	dut.onSlowCallback();
	EXPECT_TRUE(efiReadPin(Gpio::A0));
	advanceTimeUs(0.2e6);
	dut.onSlowCallback();
	EXPECT_FALSE(efiReadPin(Gpio::A0));

	// Test "force pump" mode while ignition is off
	dut.onIgnitionStateChanged(false);
	dut.forcePumpState(true);
	dut.onSlowCallback();
	EXPECT_TRUE(efiReadPin(Gpio::A0));
	dut.forcePumpState(false);
	dut.onSlowCallback();
	EXPECT_FALSE(efiReadPin(Gpio::A0));

	// Existing conservative coast policy: a recent trigger still runs the pump with ignition off.
	// FOME-004 only gates the timed startup prime, so this behavior remains unchanged.
	engine->triggerCentral.handleShaftSignal(TriggerEvent::PrimaryFalling, getTimeNowNt());
	dut.onSlowCallback();
	EXPECT_TRUE(efiReadPin(Gpio::A0));
	advanceTimeUs(10e6);
	EXPECT_FALSE(engine->triggerCentral.engineMovedRecently());
	dut.onSlowCallback();
	EXPECT_FALSE(efiReadPin(Gpio::A0));

	// Long time since ignition turned on, just saw a trigger
	dut.onIgnitionStateChanged(true);
	advanceTimeUs(10e6);
	engine->triggerCentral.handleShaftSignal(TriggerEvent::PrimaryFalling, getTimeNowNt());
	dut.onSlowCallback();
	EXPECT_TRUE(efiReadPin(Gpio::A0));
}
