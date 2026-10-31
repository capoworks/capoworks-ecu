/**
 * @file	speed_density.cpp
 *
 * See http://rusefi.com/wiki/index.php?title=Manual:Software:Fuel_Control#Speed_Density for details
 *
 * @date May 29, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#define rpmMin 500
#define rpmMax 8000

#define tpMin 0
#define tpMax 100

float IFuelComputer::getTChargeCoefficient(float rpm, float tps) {
	// First, do TPS mode since it doesn't need any of the airflow math.
	if (engineConfiguration->tChargeMode == TCHARGE_MODE_RPM_TPS) {
		float minRpmKcurrentTPS = interpolateMsg(
				"minRpm",
				tpMin,
				engineConfiguration->tChargeMinRpmMinTps,
				tpMax,
				engineConfiguration->tChargeMinRpmMaxTps,
				tps);
		float maxRpmKcurrentTPS = interpolateMsg(
				"maxRpm",
				tpMin,
				engineConfiguration->tChargeMaxRpmMinTps,
				tpMax,
				engineConfiguration->tChargeMaxRpmMaxTps,
				tps);

		const float coefficient = interpolateMsg("Kcurr", rpmMin, minRpmKcurrentTPS, rpmMax, maxRpmKcurrentTPS, rpm);
		clearThermalCoefficientInput();
		return coefficient;
	}

	constexpr floatms_t gramsPerMsToKgPerHour = (3600.0f * 1000.0f) / 1000.0f;
	// We're actually using an 'old' airMass calculated for the previous cycle, but it's ok, we're not having any
	// self-excitaton issues
	floatms_t airMassForEngine;
	uint32_t sourceGeneration;
	bool sourceValid;
	{
		// The coefficient reads the prior accepted publication. Capture its mass
		// and stamp together so diagnostics cannot pair an old mass with a newer
		// publication identity.
		chibios_rt::CriticalSectionLocker csl;
		airMassForEngine = sdAirMassInOneCylinder * engine->engineState.cylinderCount;
		sourceGeneration = thermalProvenance.AcceptedCalibrationGeneration;
		sourceValid = thermalProvenance.AcceptedValid;
	}
	// airMass is in grams per 1 cycle for 1 cyl. Convert it to airFlow in kg/h for the engine.
	// And if the engine is stopped (0 rpm), then airFlow is also zero (avoiding NaN division)
	floatms_t airFlow = (rpm == 0) ? 0 : airMassForEngine * gramsPerMsToKgPerHour / getEngineCycleDuration(rpm);

	if (engineConfiguration->tChargeMode == TCHARGE_MODE_AIR_INTERP) {
		// just interpolate between user-specified min and max coefs, based on the max airFlow value
		const float coefficient = interpolateClamped(
				0.0,
				engineConfiguration->tChargeAirCoefMin,
				engineConfiguration->tChargeAirFlowMax,
				engineConfiguration->tChargeAirCoefMax,
				airFlow);
		recordThermalCoefficientInput(
				airFlow, sourceGeneration, sourceValid && std::isfinite(airFlow), TCHARGE_MODE_AIR_INTERP);
		return coefficient;
	} else if (engineConfiguration->tChargeMode == TCHARGE_MODE_AIR_INTERP_TABLE) {
		const float coefficient =
				interpolate2d(airFlow, engineConfiguration->tchargeBins, engineConfiguration->tchargeValues);
		recordThermalCoefficientInput(
				airFlow, sourceGeneration, sourceValid && std::isfinite(airFlow), TCHARGE_MODE_AIR_INTERP_TABLE);
		return coefficient;
	} else {
		clearThermalCoefficientInput();
		firmwareError("Unexpected tChargeMode: %d", engineConfiguration->tChargeMode);
		return 0;
	}
}

//  http://rusefi.com/math/t_charge.html
/***panel:Charge Temperature*/
temperature_t IFuelComputer::getTCharge(float rpm, float tps) {
	const auto clt = Sensor::get(SensorType::Clt);
	const auto iat = Sensor::get(SensorType::Iat);

	float airTemp;

	// Without either valid, return 0C.  It's wrong, but it'll pretend to be nice and dense, so at least you won't go
	// lean.
	if (!iat && !clt) {
		clearThermalCoefficientInput();
		return 0;
	} else if (!clt && iat) {
		// Intake temperature will almost always be colder (richer) than CLT - use that
		clearThermalCoefficientInput();
		return iat.Value;
	} else if (!iat && clt) {
		// Without valid intake temperature, assume intake temp is 0C, and interpolate anyway
		airTemp = 0;
	} else {
		// All is well - use real air temp
		airTemp = iat.Value;
	}

	float coolantTemp = clt.Value;

	sdTcharge_coff = getTChargeCoefficient(rpm, tps);

	if (std::isnan(sdTcharge_coff)) {
		warning(ObdCode::CUSTOM_ERR_T2_CHARGE, "t2-getTCharge NaN");
		return coolantTemp;
	}

	// Interpolate between CLT and IAT:
	// 0.0 coefficient -> use CLT (full heat transfer)
	// 1.0 coefficient -> use IAT (no heat transfer)
	float Tcharge = interpolateClamped(0.0f, coolantTemp, 1.0f, airTemp, sdTcharge_coff);

	if (std::isnan(Tcharge)) {
		// we can probably end up here while resetting engine state - interpolation would fail
		warning(ObdCode::CUSTOM_ERR_TCHARGE_NOT_READY, "getTCharge NaN");
		return coolantTemp;
	}

	return Tcharge;
}

void IFuelComputer::clearThermalCoefficientInput() {
	chibios_rt::CriticalSectionLocker csl;
	thermalProvenance.CoefficientAirflowKgPerHour = 0;
	thermalProvenance.CoefficientSourceGeneration = 0;
	thermalProvenance.CoefficientAirflowValid = false;
	thermalProvenance.CoefficientMode = static_cast<uint8_t>(engineConfiguration->tChargeMode);
}

void IFuelComputer::recordThermalCoefficientInput(
		float airflowKgPerHour, uint32_t sourceGeneration, bool airflowValid, tChargeMode_e mode) {
	chibios_rt::CriticalSectionLocker csl;
	thermalProvenance.CoefficientAirflowKgPerHour = std::isfinite(airflowKgPerHour) ? airflowKgPerHour : 0;
	thermalProvenance.CoefficientSourceGeneration = sourceGeneration;
	thermalProvenance.CoefficientAirflowValid = airflowValid;
	thermalProvenance.CoefficientMode = static_cast<uint8_t>(mode);
}
