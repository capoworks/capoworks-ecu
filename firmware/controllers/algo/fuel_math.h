/**
 * @file	fuel_math.h
 *
 * @date May 27, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "airmass.h"
#include "airmass_injection_state.h"
#include "airmass_loads.h"
#include "blended_airmass.h"
#include "rusefi_types.h"

template <unsigned TRowCount, unsigned TColumnCount>
class PreparedTable3DInterpolation;

// Identity captured before any live standalone-model evaluation. The final
// publication compares these captured values again before changing live outputs.
struct FuelAirmassCapture {
	AirmassInjectionState::CalculationToken Token = 0;
	AirmassInputs Inputs;
	engine_load_mode_e Strategy = LM_SPEED_DENSITY;
	int ConfigurationVersion = 0;
	uint32_t CalibrationGeneration = 0;
	float Displacement = 0;
	uint8_t CylinderCount = 0;
	bool Standalone = false;
	bool Current = false;
};

// Every field here is a candidate only. Live engine state is not changed until
// calculateFuelPublication() completes and the caller commits the publication.
struct FuelPublicationCandidate {
	AirmassDiagnostics Diagnostics;
	BlendedAirmassDiagnostics BlendedDiagnostics;
	AirmassConsumerLoadCandidate ConsumerLoads;
	AirmassInjectionFault Fault = AirmassInjectionFault::Result;
	bool Valid = false;
	bool Stale = false;
	bool Degraded = false;
	bool HasBlendedDiagnostics = false;
	uint8_t VeAnalyzeEndpoint = 0;

	float BaseFuelMass = 0;
	float CycleInjectionMass = 0;
	float TpsAccelEnrich = 0;
	float SdAirMassInOneCylinder = 0;
	float NormalizedCylinderFilling = 0;
	float FuelingLoad = 0;
	float IgnitionLoad = 0;
	float AirflowEstimate = 0;

	float RunningBaseFuel = 0;
	float RunningFuel = 0;
	float TotalFuelCorrection = 0;
	float ResolvedLambdaLoad = 0;
	float AfrTableYAxis = 0;
	float TargetLambda = 0;
	float TargetAFR = 0;
	float StoichiometricRatio = 0;

	float CrankingCoolantTemperatureCoefficient = 0;
	float CrankingTpsCoefficient = 0;
	float CrankingDurationCoefficient = 0;
	float CrankingFuel = 0;

	float Stage2Fraction = 0;
	float InjectionDuration = 0;
	float InjectionDurationStage2 = 0;
	float InjectionOffset = 0;
	bool ShouldUpdateInjectionTiming = false;
	float CylinderFuelMasses[MAX_CYLINDER_COUNT] = {};
};

void initFuelMap();

/**
 * @return baseFuel with CLT and IAT corrections
 */
float getRunningFuel(float baseFuel);

float getBaroCorrection();
percent_t getFuelALSCorrection(float rpm);
int getNumberOfInjections(injection_mode_e mode);
angle_t getInjectionOffset(float rpm, float load);
float getIatFuelCorrection();

float getCltFuelCorrection();
float getCrankingFuel(float baseFuel);
float getCrankingFuel3(float baseFuel, uint32_t revolutionCounterSinceStart);
float getCycleInjectionMass(float rpm, bool isCranking);
float getInjectionModeDurationMultiplier(injection_mode_e mode);
int getNumberOfInjections(injection_mode_e mode, size_t cylinderCount);
float getInjectionModeDurationMultiplier(injection_mode_e mode, size_t cylinderCount);
percent_t getInjectorDutyCycle(float rpm);
percent_t getInjectorDutyCycleStage2(float rpm);
float getStage2InjectionFraction(float rpm, float fuelLoad);

bool captureFuelAirmassInputs(
		float rpm,
		AirmassInjectionState::CalculationToken token,
		FuelAirmassCapture& capture,
		bool requireCurrent = true);
bool isFuelCaptureCurrent(const FuelAirmassCapture& capture);
bool isFuelCaptureCurrentLocked(const FuelAirmassCapture& capture, AirmassInjectionState::LockedAccess& admission);
FuelPublicationCandidate
calculateFuelPublication(float rpm, bool isCranking, const FuelAirmassCapture& capture, bool requireCurrent = true);
void publishBlendedAirmassDiagnostics(const FuelPublicationCandidate& candidate);

float getStandardAirCharge();
float getCylinderFuelTrim(
		size_t cylinderNumber, const PreparedTable3DInterpolation<TRIM_SIZE, TRIM_SIZE>& interpolation);

struct AirmassModelBase;
AirmassModelBase* getAirmassModel(engine_load_mode_e mode);
