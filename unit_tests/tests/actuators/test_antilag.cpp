#include "pch.h"
#include "fuel_math.h"

TEST(Actuators, AntiLag) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->antiLagEnabled = true;

	setTable(config->ALSFuelAdjustment, 50 /* percent */);
	Sensor::setMockValue(SensorType::Clt, 70);
	Sensor::setMockValue(SensorType::Rpm, 1000);
	Sensor::setMockValue(SensorType::Tps1, 1);
	Sensor::setMockValue(SensorType::AcceleratorPedal, 1);
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 1);

	engineConfiguration->antiLagActivationMode = SWITCH_INPUT_ANTILAG;
	engine->periodicFastCallback();
	float fuelWithCorrections = 1.01;
	ASSERT_NEAR(fuelWithCorrections, getRunningFuel(1), EPS4D);
	// in unit tests we pretend that physical swiych is always OFF
	ASSERT_EQ(0, engine->antilagController.ALSSwitchCondition);
	ASSERT_EQ(0.0, engine->antilagController.fuelALSCorrection);

	engineConfiguration->antiLagActivationMode = ALWAYS_ON_ANTILAG;
	engine->periodicFastCallback();
	ASSERT_EQ(1, engine->antilagController.ALSSwitchCondition);
	ASSERT_NEAR(50, engine->antilagController.fuelALSCorrection, EPS4D);

	ASSERT_EQ(1, engine->antilagController.ALSMinRPMCondition);
	ASSERT_EQ(1, engine->antilagController.ALSMaxRPMCondition);
	ASSERT_EQ(1, engine->antilagController.ALSMinCLTCondition);
	ASSERT_EQ(1, engine->antilagController.ALSMaxCLTCondition);
	ASSERT_EQ(1, engine->antilagController.ALSMaxThrottleIntentCondition);

	ASSERT_EQ(1, engine->antilagController.isAntilagCondition);
	ASSERT_NEAR(1.5 * fuelWithCorrections, getRunningFuel(1), EPS4D);
}

TEST(Actuators, AntiLagRequiresValidThrottleIntent) {
	EngineTestHelper eth(engine_type_e::TEST_ENGINE);

	engineConfiguration->antiLagEnabled = true;
	engineConfiguration->antiLagActivationMode = ALWAYS_ON_ANTILAG;
	Sensor::setMockValue(SensorType::Clt, 70);
	Sensor::setMockValue(SensorType::Rpm, 1000);

	// Missing or non-finite throttle intent must not arm anti-lag.
	Sensor::setInvalidMockValue(SensorType::DriverThrottleIntent);
	engine->antilagController.update();
	EXPECT_FALSE(engine->antilagController.ALSMaxThrottleIntentCondition);
	EXPECT_FALSE(engine->antilagController.isAntilagCondition);

	Sensor::setMockValue(SensorType::DriverThrottleIntent, NAN);
	engine->antilagController.update();
	EXPECT_FALSE(engine->antilagController.ALSMaxThrottleIntentCondition);
	EXPECT_FALSE(engine->antilagController.isAntilagCondition);

	// A valid closed throttle still arms anti-lag.
	Sensor::setMockValue(SensorType::DriverThrottleIntent, 1);
	engine->antilagController.update();
	EXPECT_TRUE(engine->antilagController.ALSMaxThrottleIntentCondition);
	EXPECT_TRUE(engine->antilagController.isAntilagCondition);

	// Losing the intent must clear the already-active condition, then recover.
	Sensor::setInvalidMockValue(SensorType::DriverThrottleIntent);
	engine->antilagController.update();
	EXPECT_FALSE(engine->antilagController.isAntilagCondition);
	engine->antilagController.update();
	EXPECT_FALSE(engine->antilagController.isAntilagCondition);

	Sensor::setMockValue(SensorType::DriverThrottleIntent, 1);
	engine->antilagController.update();
	EXPECT_TRUE(engine->antilagController.isAntilagCondition);
}
