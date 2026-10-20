/**
 * @file init_sensorss.cpp
 */

#include "pch.h"

#include "init.h"
#include "cli_registry.h"
#include "io_pins.h"

static void initSensorCli();

static void initAuxDigital() {
#ifdef MODULE_LUA
#if EFI_PROD_CODE
	for (size_t i = 0; i < efi::size(engineConfiguration->luaDigitalInputPins); i++) {
		efiSetPadMode(
				"Lua Digital",
				engineConfiguration->luaDigitalInputPins[i],
				engineConfiguration->luaDigitalInputPinModes[i]);
	}
#endif // EFI_PROD_CODE
#endif // MODULE_LUA
}

static void deInitAuxDigital() {
#ifdef MODULE_LUA
	for (size_t i = 0; i < efi::size(activeConfiguration.luaDigitalInputPins); i++) {
		brain_pin_markUnused(activeConfiguration.luaDigitalInputPins[i]);
	}
#endif // MODULE_LUA
}

void initNewSensors() {
	// First (optionally) init any sensors built in to the board that don't need config
	initBoardSensors();

	reconfigureSensors();

	initBaro();
	initAuxSpeedSensors();

	initAuxDigital();

	// Init CLI functionality for sensors (mocking)
	initSensorCli();

#if defined(HARDWARE_CI) && !defined(HW_PROTEUS)
	chThdSleepMilliseconds(100);

	if (Sensor::getOrZero(SensorType::BatteryVoltage) < 8) {
		// Fake that we have battery voltage, some tests rely on it
		Sensor::setMockValue(SensorType::BatteryVoltage, 10);
	}
#endif
}

void stopSensors() {
	deInitAuxDigital();

	deinitTps();
	deinitFluidPressure();
	deinitVbatt();
	deinitThermistors();
	deinitLambda();
	deInitFlexSensor();
#ifdef MODULE_HELLA_OPST
	deinitHellaOpsT();
#endif // MODULE_HELLA_OPST
	deinitAuxSensors();
	deInitVehicleSpeedSensor();
	deinitTurbochargerSpeedSensor();
	deinitAuxSpeedSensors();
	deinitMap();
	deinitMaf();
	deinitFuelLevel();
}

void reconfigureSensors() {
	initVbatt();
	initMap();
	initMaf();
	initFuelLevel();
	initTps();
	initFluidPressure();
	initThermistors();
	initLambda();
	initFlexSensor();
#ifdef MODULE_HELLA_OPST
	initHellaOpsT();
#endif // MODULE_HELLA_OPST
	initAuxSensors();
	initVehicleSpeedSensor();
	initTurbochargerSpeedSensor();
}

// Mocking/testing helpers
static void initSensorCli() {
	addConsoleActionSS("set_sensor_mock", [](const char* typeName, const char* valueStr) {
		SensorType type = findSensorTypeByName(typeName);

		if (type == SensorType::Invalid) {
			efiPrintf("Invalid sensor type specified: %s", typeName);
			return;
		}

		float value = atoff(valueStr);

		Sensor::setMockValue(type, value);
	});

	addConsoleAction("reset_sensor_mocks", Sensor::resetAllMocks);
	addConsoleAction("show_sensors", Sensor::showAllSensorInfo);
	addConsoleActionI("show_sensor", [](int idx) { Sensor::showInfo(static_cast<SensorType>(idx)); });
}
