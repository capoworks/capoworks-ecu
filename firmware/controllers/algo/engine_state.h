/**
 * @file engine_state.h
 * @brief One header which acts as gateway to current engine state
 *
 * @date Dec 20, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "global.h"
#include "engine_parts.h"
#include "engine_state_generated.h"
#include "airmass_loads.h"
#include "ve_analyze_gate.h"
#include "ve_analyze_fuel_events.h"

struct output_channels_s;

// Diagnostic-only evidence for the existing VE Analyze qualification latch.
// This does not add admission or recovery policy.
struct VeAnalyzeLatchDiagnostics {
	uint16_t FirstRejectReason = 0;
	uint16_t CurrentRejectReasons = 0;
	uint8_t CandidateEndpoint = 0;
	uint32_t SessionGeneration = 0;
	uint32_t CalibrationGeneration = 0;
	uint32_t CalculationEpoch = 0;
	int64_t FirstLatchTimeSeconds = 0;
};

// Diagnostic-only evidence for composite calculations discarded at publication.
// This domain never participates in qualification, admission, or fuel output.
struct VeAnalyzeDiscardedCalculationDiagnostics {
	uint16_t Count = 0;
	uint16_t LastReason = 0;
	uint32_t LastCalibrationGeneration = 0;
};

// Operating exclusions are separate from the sampler's TPS/history reasons.
// These values begin above the complete VeAnalyzeGateReason enum.
enum class VeAnalyzeOperatingReason : uint32_t {
	None = 0,
	InvalidRpm = 1u << 13,
	Cranking = 1u << 14,
	Launch = 1u << 15,
	Antilag = 1u << 16,
	InvalidSensor = 1u << 17,
	InvalidFuelCalculation = 1u << 18,
	FuelCut = 1u << 19,
	SparkCut = 1u << 20,
	Dfco = 1u << 21,
};

static_assert(
		static_cast<uint32_t>(VeAnalyzeOperatingReason::InvalidRpm) >
		static_cast<uint32_t>(VeAnalyzeGateReason::ResetRecovery));

constexpr uint32_t operator|(VeAnalyzeOperatingReason lhs, VeAnalyzeOperatingReason rhs) {
	return static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs);
}

class EngineState : public engine_state_s {
public:
	EngineState();
	void periodicFastCallback();
	void updateVeAnalyzeSampleGate(efitick_t nowNt);
	void resetVeAnalyzeSampleGate(efitick_t nowNt);
	void updateTChargeK(float rpm, float tps);
	void resetVeAnalyzeDiagnostics(uint32_t calibrationGeneration = 0, uint32_t calculationEpoch = 0);
	void latchVeAnalyzeDiagnostics(uint16_t reasonMask, uint32_t calibrationGeneration, uint32_t calculationEpoch);
	void observeVeAnalyzeCandidate(
			uint8_t candidateEndpoint,
			uint16_t rejectReasonMask,
			uint32_t calibrationGeneration,
			uint32_t calculationEpoch);
	// Caller must hold the publication state's critical section.
	void recordVeAnalyzeDiscardedCalculationLocked(uint32_t capturedCalibrationGeneration);
	void resetVeAnalyzeDiscardedCalculationDiagnostics();
	void exportVeAnalyzeDiagnostics(output_channels_s& output) const;

	// The event collector has no internal lock. Schedule observation and the
	// slow telemetry snapshot with the caller-owned critical section held.
	void resetVeAnalyzeFuelEventsLocked(efitick_t nowNt);
	bool observeVeAnalyzeFuelEventLocked(VeAnalyzeFuelEventInput input);
	// The returned reference remains valid until the next snapshot while the
	// caller continues to hold the critical section.
	const VeAnalyzeFuelEventSnapshot& snapshotVeAnalyzeFuelEventsLocked(efitick_t nowNt) const;
	void exportVeAnalyzeFuelEventsLocked(output_channels_s& output) const;

	/**
	 * always 360 or 720, never zero
	 */
	angle_t engineCycle;

	/**
	 * Number of cylinders the engine has, cached from the configured firing order.
	 * Recomputed in prepareOutputSignals() on any configuration change.
	 * @see getFiringOrderLength()
	 */
	uint8_t cylinderCount = 1;

	bool useOddFireWastedSpark = false;

	float injectionStage2Fraction = 0;

	Timer crankingTimer;

	WarningCodeState warnings;

	/**
	 * MAP averaging angle start, in relation to 'mapAveragingSchedulingAtIndex' trigger index index
	 */
	angle_t mapAveragingStart[MAX_CYLINDER_COUNT];

	// Angle between firing the main (primary) spark and the secondary (trailing) spark
	angle_t trailingSparkAngle = 0;

	Timer timeSinceLastTChargeK;

	float currentVe = 0;

	bool airmassCalculationValid = false;
	AirmassLoadSnapshot airmassLoads;
	uint32_t airmassLoadsInvalidationEpoch = 0;
	uint32_t airmassLoadsPublicationSerial = 0;
	bool veAnalyzeSessionStarted = false;
	bool veAnalyzeSessionInvalid = false;
	uint8_t veAnalyzeEndpoint = 0;
	VeAnalyzeLatchDiagnostics veAnalyzeDiagnostics;
	VeAnalyzeDiscardedCalculationDiagnostics veAnalyzeDiscardedCalculationDiagnostics;
	VeAnalyzeFuelEvents veAnalyzeFuelEvents;

	/**
	 * Raw fuel injection duration produced by current fuel algorithm, without any correction
	 */
	floatms_t baseFuel = 0;

	/**
	 * TPS acceleration: extra fuel amount
	 */
	floatms_t tpsAccelEnrich = 0;

	/**
	 * Each individual fuel injection duration for current engine cycle, without wall wetting
	 * including everything including injector lag, both cranking and running
	 * @see getInjectionDuration()
	 */
	floatms_t injectionDuration = 0;
	floatms_t injectionDurationStage2 = 0;

	angle_t injectionOffset = 0;

	multispark_state multispark;

	bool shouldUpdateInjectionTiming = true;

	void updateSplitInjection();

	Timer splitInjectionTimer;
	bool requestSplitInjection = false;

	void updateMapCylinderOffsets();

	/**
	 * Per-cylinder MAP window averages at full precision.
	 * The mapPerCylinder[] live data channel is a uint8_t for display only - it saturates at 255 kPa,
	 * so it must never be read back in to the cylinder balance math.
	 */
	float mapPerCylinderFloat[MAX_CYLINDER_COUNT] = {0};
	float mapCylinderBalance[MAX_CYLINDER_COUNT] = {0};

private:
	VeAnalyzeGate veAnalyzeGate;
	efitick_t lastTpsObservationTick;
	uint32_t veAnalyzeOperatingReasons = 0;
	// ECU fast-callback observations, not accepted TunerStudio table cells.
	uint32_t veAnalyzeAcceptedObservations = 0;
	uint32_t veAnalyzeRejectedObservations = 0;
	uint64_t veAnalyzeFuelEventSerial = 0;
	bool hasVeAnalyzeFuelAttempt = false;
	uint64_t veAnalyzeFuelLastAttemptSerial = 0;
	uint8_t veAnalyzeFuelLastAttemptOwner = 0;
	efitick_t veAnalyzeFuelLastAttemptTimestamp;
	uint16_t veAnalyzeFuelLastAttemptCylinderMask = 0;
	uint8_t veAnalyzeFuelLastAttemptBankMask = 0;
	bool veAnalyzeFuelLastAttemptWallClamp = false;
	float veAnalyzeFuelLastAttemptBaseGrams = 0;
	float veAnalyzeFuelLastAttemptAeGrams = 0;
	float veAnalyzeFuelLastAttemptFilmGrams = 0;
	float veAnalyzeFuelLastAttemptFinalGrams = 0;
	mutable VeAnalyzeFuelEventSnapshot veAnalyzeFuelEventsSnapshot;
};

EngineState* getEngineState();
