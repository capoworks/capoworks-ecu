/**
 * @file fuel_computer.h
 */

#pragma once

class ValueProvider3D;
struct output_channels_s;

#include "rusefi_types.h"
#include "fuel_computer_generated.h"

struct IFuelComputer : public fuel_computer_s {
	virtual mass_t getCycleFuel(mass_t airmass, float rpm, float load) = 0;
	temperature_t getTCharge(float rpm, float tps);
	float getLoadOverride(float defaultLoad, load_override_e overrideMode) const;

	// Diagnostic-only snapshots for the charge-temperature calculation. The
	// accepted fields are updated only with a completed fuel publication, while
	// the coefficient fields describe the preceding accepted publication used by
	// the current charge-temperature evaluation.
	void clearThermalProvenance();
	void acceptThermalProvenance(uint32_t calibrationGeneration, float fuelTemperatureK, bool fuelTemperatureValid);
	void exportThermalProvenance(output_channels_s& output) const;

private:
	float getTChargeCoefficient(float rpm, float tps);
	void clearThermalCoefficientInput();
	void recordThermalCoefficientInput(
			float airflowKgPerHour, uint32_t sourceGeneration, bool airflowValid, tChargeMode_e mode);

	struct ThermalProvenanceRuntime {
		uint32_t AcceptedCalibrationGeneration = 0;
		bool AcceptedValid = false;

		float CoefficientAirflowKgPerHour = 0;
		uint32_t CoefficientSourceGeneration = 0;
		bool CoefficientAirflowValid = false;
		uint8_t CoefficientMode = static_cast<uint8_t>(TCHARGE_MODE_RPM_TPS);

		float FuelTemperatureK = 0;
		uint32_t FuelTemperatureCalibrationGeneration = 0;
		bool FuelTemperatureValid = false;
	};

	ThermalProvenanceRuntime thermalProvenance;
};

// This contains the math of the fuel model, but doesn't actually read any configuration
class FuelComputerBase : public IFuelComputer {
public:
	mass_t getCycleFuel(mass_t airmass, float rpm, float load) override;
	// The owning calculation has already resolved and validated this coordinate.
	// Do not re-read afrOverrideMode or live MAP/TPS inside a physical candidate.
	mass_t getCycleFuelWithResolvedLoad(mass_t airmass, float rpm, float load);
	float getResolvedLambdaLoad() const {
		return m_resolvedLambdaLoad;
	}
	void publishResolvedLambdaLoad(float load) {
		m_resolvedLambdaLoad = load;
	}

	virtual float getStoichiometricRatio() const = 0;
	virtual float getTargetLambda(float rpm, float load) const = 0;
	virtual float getTargetLambdaLoadAxis(float defaultLoad) const = 0;

private:
	mass_t calculateCycleFuel(mass_t airmass, float rpm, float load, bool strict);
	float m_resolvedLambdaLoad = 0;
};

// This class is a usable implementation of a fuel model that reads real configuration
class FuelComputer final : public FuelComputerBase {
public:
	float getStoichiometricRatio() const override;
	float getTargetLambda(float rpm, float load) const override;
	float getTargetLambdaLoadAxis(float defaultLoad) const override;
};

float getLoadOverride(float defaultLoad, load_override_e overrideMode);

// Last valid flex fuel sensor reading stored in backup RAM, if there is one
expected<float> getStoredFlexEthanolPercent();

// Best ethanol content we know of: the live sensor if it's valid, otherwise the last stored
// reading, otherwise the configured default
float getFlexEthanolPercent();

// Store the current flex fuel sensor reading in backup RAM, if it's valid
void updateStoredFlexEthanolPercent();

constexpr float fuelDensity = 0.72; // g/cc
