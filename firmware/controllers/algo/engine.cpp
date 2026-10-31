/**
 * @file	engine.cpp
 *
 *
 * This might be a http://en.wikipedia.org/wiki/God_object but that's best way I can
 * express myself in C/C++. I am open for suggestions :)
 *
 * @date May 21, 2014
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "trigger_central.h"
#include "fuel_math.h"
#ifdef MODULE_CYLINDER_FUEL_TRIM
#include "cylinder_fuel_trim.h"
#endif // MODULE_CYLINDER_FUEL_TRIM
#include "speed_density.h"
#include "init.h"

#include <algorithm>
#include <cmath>

#include "perf_trace.h"
#include "backup_ram.h"
#include "idle_thread.h"
#include "idle_hardware.h"
#include "gppwm.h"
#include "speedometer.h"
#ifdef MODULE_BOOST
#include "boost_control.h"
#endif // MODULE_BOOST
#include "ac_control.h"
#include "vr_pwm.h"
#include "can_vss.h"
#if EFI_MC33816
#include "mc33816.h"
#endif // EFI_MC33816

#if EFI_PROD_CODE
#include "trigger_emulator_algo.h"
#include "bench_test.h"
#else
#define isRunningBenchTest() true
#endif /* EFI_PROD_CODE */

#if (BOARD_TLE8888_COUNT > 0)
#include "gpio/tle8888.h"
#endif

#if EFI_ENGINE_SNIFFER
#include "engine_sniffer.h"
extern WaveChart waveChart;
#endif /* EFI_ENGINE_SNIFFER */

#if EFI_ENGINE_SNIFFER
void Engine::resetEngineSnifferIfInTestMode() {
	if (isFunctionalTestMode) {
		// TODO: what is the exact reasoning for the exact engine sniffer pause time I wonder
		waveChart.pauseEngineSnifferUntilNt = getTimeNowNt() + MS2NT(300);
		waveChart.reset();
	}
}
#endif /* EFI_ENGINE_SNIFFER */

void Engine::updateTriggerWaveform() {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	// we have a confusing threading model so some synchronization would not hurt
	chibios_rt::CriticalSectionLocker csl;

	engine->triggerCentral.updateWaveform();

	if (!engine->triggerCentral.triggerShape.shapeDefinitionError) {
		prepareOutputSignals();
	}
#endif /* EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT */
}

void Engine::periodicSlowCallback() {
#if EFI_SHAFT_POSITION_INPUT
	// Re-read config in case it's changed
	triggerCentral.primaryTriggerConfiguration.update();
	for (int camIndex = 0; camIndex < CAMS_PER_BANK; camIndex++) {
		triggerCentral.vvtTriggerConfiguration[camIndex].update();
	}

	{
		// Keep a new trigger event from arriving between the timeout check and reset.
		chibios_rt::CriticalSectionLocker csl;
		// isStopped() also treats SPINNING_UP with zero RPM as stopped for engine
		// math. A timed-out start still needs the actual STOPPED state and resets.
		if (rpmCalculator.getState() != STOPPED && !triggerCentral.engineMovedRecently(getTimeNowNt())) {
			OnTriggerSynchronizationLost();
		}
	}
#endif // EFI_SHAFT_POSITION_INPUT

	efiWatchdog();
	updateSlowSensors();
	updateWidebandAliveTimers();
	checkShutdown();

	module<TpsAccelEnrichment>()->onNewValue(Sensor::getOrZero(SensorType::Tps1));

	updateVrPwm();

	// Remember the current ethanol content so we have something sane to use before the flex
	// sensor wakes up on the next start
	updateStoredFlexEthanolPercent();

	enginePins.o2heater.setValue(engineConfiguration->forceO2Heating || engine->rpmCalculator.isRunning());
	enginePins.starterRelayDisable.setValue(Sensor::getOrZero(SensorType::Rpm) < engineConfiguration->cranking.rpm);

	updateGppwm();

	engine->engineModules.apply_all([](auto& m) { m.onSlowCallback(); });

#if (BOARD_TLE8888_COUNT > 0)
	tle8888startup();
#endif

#if EFI_PROD_CODE
	void baroLps25Update();
	baroLps25Update();
#endif // EFI_PROD_CODE

	engineState.updateSplitInjection();
}

/**
 * We are executing these heavy (logarithm) methods from outside the trigger callbacks for performance reasons.
 * See also periodicFastCallback
 */
void Engine::updateSlowSensors() {
	updateSwitchInputs();

#if EFI_SHAFT_POSITION_INPUT
	float rpm = Sensor::getOrZero(SensorType::Rpm);
	triggerCentral.isEngineSnifferEnabled = rpm < engineConfiguration->engineSnifferRpmThreshold;
#endif // EFI_SHAFT_POSITION_INPUT
}

static bool getClutchUpState() {
	if (isBrainPinValid(engineConfiguration->clutchUpPin)) {
		return engineConfiguration->clutchUpPinInverted ^ efiReadPin(engineConfiguration->clutchUpPin);
	}
	return engine->engineState.lua.clutchUpState;
}

static bool getBrakePedalState() {
	if (isBrainPinValid(engineConfiguration->brakePedalPin)) {
		return engineConfiguration->brakePedalPinInverted ^ efiReadPin(engineConfiguration->brakePedalPin);
	}

#if EFI_CAN_SUPPORT
	// Some vehicles broadcast the brake switch on CAN, use that if we have it
	if (auto canBrake = getCanBrakePedalState()) {
		return canBrake.Value;
	}
#endif // EFI_CAN_SUPPORT

	return engine->engineState.lua.brakePedalState;
}

void Engine::updateSwitchInputs() {
	// this value is not used yet
	if (isBrainPinValid(engineConfiguration->clutchDownPin)) {
		engine->engineState.clutchDownState =
				engineConfiguration->clutchDownPinInverted ^ efiReadPin(engineConfiguration->clutchDownPin);
	}
	{
		bool currentState;
		if (hasAcToggle()) {
			currentState = getAcToggle();
		} else {
			currentState = engine->engineState.lua.acRequestState;
		}
		AcController& acController = engine->module<AcController>().unmock();
		if (acController.acButtonState != currentState) {
			acController.acButtonState = currentState;
			acController.timeSinceStateChange.reset();
		}
		if (hasAcPressure()) {
			acController.acPressureSwitchState = getAcPressure();
		}
	}

	engine->engineState.clutchUpState = getClutchUpState();
	engine->engineState.brakePedalState = getBrakePedalState();
}

static uint16_t selectFirstVeAnalyzeReason(uint16_t reasons) {
	constexpr AirmassVeAnalyzeRejectReason priority[] = {
			AirmassVeAnalyzeRejectReason::StrategyChange,
			AirmassVeAnalyzeRejectReason::CalibrationWrite,
			AirmassVeAnalyzeRejectReason::NonUniformAuthority,
			AirmassVeAnalyzeRejectReason::Configuration,
			AirmassVeAnalyzeRejectReason::Sensor,
			AirmassVeAnalyzeRejectReason::IdleVe,
			AirmassVeAnalyzeRejectReason::Fallback,
			AirmassVeAnalyzeRejectReason::CalculationResult,
			AirmassVeAnalyzeRejectReason::StaleCalculation,
			AirmassVeAnalyzeRejectReason::SessionAlreadyInvalid,
			AirmassVeAnalyzeRejectReason::Stabilization,
	};

	for (const auto reason : priority) {
		const auto bit = airmassVeAnalyzeReason(reason);
		if (reasons & bit) {
			return bit;
		}
	}
	return airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::None);
}

static void saturatingIncrement(uint32_t& value) {
	if (value < UINT32_MAX) {
		++value;
	}
}

static float saturatedSeconds(uint32_t milliseconds) {
	constexpr float maximumSeconds = 655.35f;
	const float seconds = static_cast<float>(milliseconds) / 1000.0f;
	return std::isfinite(seconds) ? std::min(seconds, maximumSeconds) : maximumSeconds;
}

void EngineState::updateVeAnalyzeSampleGate(efitick_t nowNt) {
#if EFI_ENGINE_CONTROL
	bool tpsFresh = false;
	const SensorResult tps = [&]() {
		// Keep the sampled value, its publication marker, and the ECU observation
		// marker coherent when an ADC update or gate reset runs on another thread.
		chibios_rt::CriticalSectionLocker csl;
		const auto reading = Sensor::get(SensorType::Tps1);
		tpsFresh = Sensor::hasUpdatedAfter(SensorType::Tps1, lastTpsObservationTick);
		lastTpsObservationTick = nowNt;
		return reading;
	}();
	const auto rpm = Sensor::get(SensorType::Rpm);

	uint32_t operatingReasons = 0;
	if (!rpm.Valid || !std::isfinite(rpm.Value) || rpm.Value <= 0) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::InvalidRpm);
	}
	if (engine->rpmCalculator.isCranking()) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::Cranking);
	}
	if (!tps.Valid) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::InvalidSensor);
	}
	if (!airmassCalculationValid || !std::isfinite(tpsAccelEnrich)) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::InvalidFuelCalculation);
	}

	auto& limpManager = engine->module<LimpManager>().unmock();
	const bool fuelCut = !limpManager.allowInjection().value;
	const bool sparkCut = !limpManager.allowIgnition().value;
	const bool dfcoActive = engine->module<DfcoController>()->cutFuel();
	if (fuelCut) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::FuelCut);
	}
	if (sparkCut) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::SparkCut);
	}
	if (dfcoActive) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::Dfco);
	}

	bool launchActive = false;
#if EFI_LAUNCH_CONTROL
	launchActive = engine->launchController.isLaunchCondition;
#endif
	if (launchActive) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::Launch);
	}

	bool antilagActive = false;
#if EFI_ANTILAG_SYSTEM
	antilagActive = engine->antilagController.isAntilagCondition;
#endif
	if (antilagActive) {
		operatingReasons |= static_cast<uint32_t>(VeAnalyzeOperatingReason::Antilag);
	}

	const bool correctionActive = std::isfinite(tpsAccelEnrich) && std::fabs(tpsAccelEnrich) > 0.01f;
	const bool cutActive = fuelCut || sparkCut || dfcoActive;
	const float tpsPercent = tps.Valid ? tps.Value : NAN;

	{
		chibios_rt::CriticalSectionLocker csl;
		(void)veAnalyzeGate.updateTps(nowNt, tpsPercent, tps.Valid, tpsFresh);
		veAnalyzeGate.observeTransient(nowNt, VeAnalyzeTransient::FuelCorrection, correctionActive);
		veAnalyzeGate.observeTransient(nowNt, VeAnalyzeTransient::FuelCut, cutActive);
		veAnalyzeOperatingReasons = operatingReasons;
		const auto gateSnapshot = veAnalyzeGate.snapshot(nowNt);
		if (gateSnapshot.Ready && operatingReasons == 0) {
			saturatingIncrement(veAnalyzeAcceptedObservations);
		} else {
			saturatingIncrement(veAnalyzeRejectedObservations);
		}
	}
#else
	(void)nowNt;
#endif // EFI_ENGINE_CONTROL
}

void EngineState::resetVeAnalyzeSampleGate(efitick_t nowNt) {
	chibios_rt::CriticalSectionLocker csl;
	veAnalyzeGate.reset(nowNt);
	lastTpsObservationTick = nowNt;
	veAnalyzeOperatingReasons = 0;
	veAnalyzeAcceptedObservations = 0;
	veAnalyzeRejectedObservations = 0;
}

void EngineState::resetVeAnalyzeDiagnostics(uint32_t calibrationGeneration, uint32_t calculationEpoch) {
	const uint32_t nextSessionGeneration = veAnalyzeDiagnostics.SessionGeneration + 1;
	veAnalyzeDiagnostics = {};
	veAnalyzeDiagnostics.SessionGeneration = nextSessionGeneration;
	veAnalyzeDiagnostics.CalibrationGeneration = calibrationGeneration;
	veAnalyzeDiagnostics.CalculationEpoch = calculationEpoch;
	veAnalyzeSessionStarted = false;
	veAnalyzeSessionInvalid = false;
	veAnalyzeEndpoint = 0;
}

void EngineState::latchVeAnalyzeDiagnostics(
		uint16_t reasonMask, uint32_t calibrationGeneration, uint32_t calculationEpoch) {
	const auto alreadyInvalid = airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::SessionAlreadyInvalid);
	veAnalyzeDiagnostics.CurrentRejectReasons = reasonMask | (veAnalyzeSessionInvalid ? alreadyInvalid : 0);
	veAnalyzeDiagnostics.CalibrationGeneration = calibrationGeneration;
	veAnalyzeDiagnostics.CalculationEpoch = calculationEpoch;
	veAnalyzeSessionInvalid = true;
	veAnalyzeEndpoint = 0;

	if (veAnalyzeDiagnostics.FirstRejectReason == 0) {
		veAnalyzeDiagnostics.FirstRejectReason = selectFirstVeAnalyzeReason(reasonMask);
		veAnalyzeDiagnostics.FirstLatchTimeSeconds = getTimeNowS();
	}
}

void EngineState::observeVeAnalyzeCandidate(
		uint8_t candidateEndpoint,
		uint16_t rejectReasonMask,
		uint32_t calibrationGeneration,
		uint32_t calculationEpoch) {
	veAnalyzeDiagnostics.CandidateEndpoint = candidateEndpoint;
	veAnalyzeDiagnostics.CalibrationGeneration = calibrationGeneration;
	veAnalyzeDiagnostics.CalculationEpoch = calculationEpoch;

	if (rejectReasonMask != 0) {
		latchVeAnalyzeDiagnostics(rejectReasonMask, calibrationGeneration, calculationEpoch);
	} else if (veAnalyzeSessionInvalid) {
		veAnalyzeDiagnostics.CurrentRejectReasons =
				airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::SessionAlreadyInvalid);
	} else {
		veAnalyzeDiagnostics.CurrentRejectReasons = 0;
	}
}

void EngineState::recordVeAnalyzeDiscardedCalculationLocked(uint32_t capturedCalibrationGeneration) {
	auto& diagnostics = veAnalyzeDiscardedCalculationDiagnostics;
	if (diagnostics.Count < UINT16_MAX) {
		++diagnostics.Count;
	}
	diagnostics.LastReason = airmassVeAnalyzeReason(AirmassVeAnalyzeRejectReason::StaleCalculation);
	diagnostics.LastCalibrationGeneration = capturedCalibrationGeneration;
}

void EngineState::resetVeAnalyzeDiscardedCalculationDiagnostics() {
	veAnalyzeDiscardedCalculationDiagnostics = {};
}

void EngineState::exportVeAnalyzeDiagnostics(output_channels_s& output) const {
	const VeAnalyzeGateSnapshot gateSnapshot = veAnalyzeGate.snapshot(getTimeNowNt());
	const uint32_t gateReasons = gateSnapshot.Reasons | veAnalyzeOperatingReasons;

	output.blendedVeAnalyzeCandidateEndpoint = veAnalyzeDiagnostics.CandidateEndpoint;
	output.blendedVeAnalyzeFirstReason = veAnalyzeDiagnostics.FirstRejectReason;
	output.blendedVeAnalyzeRejectReasons = veAnalyzeDiagnostics.CurrentRejectReasons;

	uint16_t elapsedSeconds = 0;
	if (veAnalyzeDiagnostics.FirstRejectReason != 0) {
		const int64_t elapsed = getTimeNowS() - veAnalyzeDiagnostics.FirstLatchTimeSeconds;
		elapsedSeconds = elapsed <= 0 ? 0 : elapsed > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(elapsed);
	}
	output.blendedVeAnalyzeLatchedTime = elapsedSeconds;
	output.blendedVeAnalyzeSessionGeneration = veAnalyzeDiagnostics.SessionGeneration;
	output.blendedVeAnalyzeCalibrationGeneration = veAnalyzeDiagnostics.CalibrationGeneration;
	output.blendedVeAnalyzeCalculationEpoch = veAnalyzeDiagnostics.CalculationEpoch;
	output.blendedVeAnalyzeEndpoint = veAnalyzeSessionInvalid ? 0 : veAnalyzeEndpoint;

	const auto& discarded = veAnalyzeDiscardedCalculationDiagnostics;
	output.blendedVeAnalyzeDiscardedCount = discarded.Count;
	output.blendedVeAnalyzeLastDiscardedReason = discarded.LastReason;
	output.blendedVeAnalyzeLastDiscardedGeneration = discarded.LastCalibrationGeneration;

	output.veAnalyzeTpsRateAbs =
			std::isfinite(gateSnapshot.AbsTpsRatePercentPerSecond) ? gateSnapshot.AbsTpsRatePercentPerSecond : 0;
	output.veAnalyzeRateValid = gateSnapshot.RateValid;
	output.veAnalyzeSampleReady = gateSnapshot.Ready && veAnalyzeOperatingReasons == 0;
	output.veAnalyzeSampleRejectReasons = gateReasons;
	output.veAnalyzeTimeSinceMovement = saturatedSeconds(gateSnapshot.MillisecondsSinceMovement);
	output.veAnalyzeTimeSinceCorrection = saturatedSeconds(gateSnapshot.MillisecondsSinceCorrection);
	output.veAnalyzeRecoveryRemaining = saturatedSeconds(gateSnapshot.HoldRemainingMilliseconds);
	output.veAnalyzeAcceptedObservations = veAnalyzeAcceptedObservations;
	output.veAnalyzeRejectedObservations = veAnalyzeRejectedObservations;
}

void EngineState::resetVeAnalyzeFuelEventsLocked(efitick_t nowNt) {
	veAnalyzeFuelEvents.reset(nowNt);
	hasVeAnalyzeFuelAttempt = false;
}

bool EngineState::observeVeAnalyzeFuelEventLocked(VeAnalyzeFuelEventInput input) {
	// Keep a producer-local serial ahead of collector filtering so every actual
	// phase-relevant attempt remains distinguishable in exported evidence.
	input.Serial = ++veAnalyzeFuelEventSerial;
	hasVeAnalyzeFuelAttempt = true;
	veAnalyzeFuelLastAttemptSerial = input.Serial;
	veAnalyzeFuelLastAttemptOwner = input.OwnerCylinder;
	veAnalyzeFuelLastAttemptTimestamp = input.Timestamp;
	veAnalyzeFuelLastAttemptCylinderMask = input.AffectedCylinderMask;
	veAnalyzeFuelLastAttemptBankMask = input.AffectedBankMask;
	veAnalyzeFuelLastAttemptWallClamp = input.WallModelZeroClamp;
	veAnalyzeFuelLastAttemptBaseGrams = input.BasePreAeMassGrams;
	veAnalyzeFuelLastAttemptAeGrams = input.AppliedAeMassGrams;
	veAnalyzeFuelLastAttemptFilmGrams = input.FilmCorrectionMassGrams;
	veAnalyzeFuelLastAttemptFinalGrams = input.FinalCommandedMassGrams;
	return veAnalyzeFuelEvents.observeEvent(input);
}

VeAnalyzeFuelEventSnapshot EngineState::snapshotVeAnalyzeFuelEventsLocked(efitick_t nowNt) const {
	return veAnalyzeFuelEvents.snapshot(nowNt);
}

void EngineState::exportVeAnalyzeFuelEventsLocked(output_channels_s& output) const {
	// The live-data schema fixes per-cylinder capacity with [12 iterate]; keep the export loop in bounds.
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderBaseMg) / sizeof(output.veAnalyzeFuelCylinderBaseMg[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderAeMg) / sizeof(output.veAnalyzeFuelCylinderAeMg[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderFilmMg) / sizeof(output.veAnalyzeFuelCylinderFilmMg[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderFinalMg) / sizeof(output.veAnalyzeFuelCylinderFinalMg[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderScheduledMg) / sizeof(output.veAnalyzeFuelCylinderScheduledMg[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderReasonMask) / sizeof(output.veAnalyzeFuelCylinderReasonMask[0]) ==
			MAX_CYLINDER_COUNT);
	static_assert(
			sizeof(output.veAnalyzeFuelCylinderAge) / sizeof(output.veAnalyzeFuelCylinderAge[0]) == MAX_CYLINDER_COUNT);

	const efitick_t nowNt = getTimeNowNt();
	const VeAnalyzeFuelEventSnapshot snapshot = snapshotVeAnalyzeFuelEventsLocked(nowNt);
	output.veAnalyzeFuelEventReady = snapshot.Ready;
	output.veAnalyzeFuelEventRejectReasons = snapshot.Reasons;
	output.veAnalyzeFuelEventAttemptedCount = snapshot.AttemptedEventCount;
	output.veAnalyzeFuelEventAcceptedCount = snapshot.AcceptedEventCount;
	output.veAnalyzeFuelEventRejectedCount = snapshot.RejectedEventCount;
	output.veAnalyzeFuelEventInvalidCount = snapshot.InvalidEvidenceEventCount;
	output.veAnalyzeFuelEventObservedCylinderMask = snapshot.ObservedCylinderMask;
	output.veAnalyzeFuelEventObservedBankMask = snapshot.ObservedBankMask;
	output.veAnalyzeFuelEventExpectedCylinderMask = snapshot.ExpectedCylinderMask;
	output.veAnalyzeFuelEventExpectedBankMask = snapshot.ExpectedBankMask;
	output.veAnalyzeFuelEventFreshCylinderMask = snapshot.FreshCylinderMask;
	output.veAnalyzeFuelEventFreshBankMask = snapshot.FreshBankMask;
	output.veAnalyzeFuelEventCoverageComplete = snapshot.CoverageComplete;
	output.veAnalyzeFuelEventPublicationGeneration = snapshot.PublicationGeneration;

	const auto& last = snapshot.LastEvent;
	const bool lastAttemptMatchesRetainedEvent =
			hasVeAnalyzeFuelAttempt && last.HasEvent && last.Serial == veAnalyzeFuelLastAttemptSerial;
	const uint64_t lastAttemptSerial = hasVeAnalyzeFuelAttempt ? veAnalyzeFuelLastAttemptSerial : 0;
	output.veAnalyzeFuelEventLastOwner = hasVeAnalyzeFuelAttempt ? veAnalyzeFuelLastAttemptOwner : 0;
	output.veAnalyzeFuelEventLastAge = UINT16_MAX;
	if (hasVeAnalyzeFuelAttempt && nowNt.count > veAnalyzeFuelLastAttemptTimestamp.count) {
		const uint64_t elapsedNt =
				static_cast<uint64_t>(nowNt.count) - static_cast<uint64_t>(veAnalyzeFuelLastAttemptTimestamp.count);
		const uint64_t elapsedMilliseconds = NT2US(elapsedNt) / 1000;
		output.veAnalyzeFuelEventLastAge =
				elapsedMilliseconds > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(elapsedMilliseconds);
	} else if (hasVeAnalyzeFuelAttempt) {
		output.veAnalyzeFuelEventLastAge = 0;
	}
	output.veAnalyzeFuelEventLastSerialLow = static_cast<uint32_t>(lastAttemptSerial);
	output.veAnalyzeFuelEventLastSerialHigh = static_cast<uint32_t>(lastAttemptSerial >> 32);
	output.veAnalyzeFuelEventLastWallClamp = hasVeAnalyzeFuelAttempt ? veAnalyzeFuelLastAttemptWallClamp : 0;
	output.veAnalyzeFuelEventLastCylinderMask = hasVeAnalyzeFuelAttempt ? veAnalyzeFuelLastAttemptCylinderMask : 0;
	output.veAnalyzeFuelEventLastBankMask = hasVeAnalyzeFuelAttempt ? veAnalyzeFuelLastAttemptBankMask : 0;
	output.veAnalyzeFuelEventLastBaseMg =
			hasVeAnalyzeFuelAttempt && std::isfinite(veAnalyzeFuelLastAttemptBaseGrams)
					? ve_analyze_fuel_events::gramsToMilligrams(veAnalyzeFuelLastAttemptBaseGrams)
					: 0;
	output.veAnalyzeFuelEventLastAeMg =
			hasVeAnalyzeFuelAttempt && std::isfinite(veAnalyzeFuelLastAttemptAeGrams)
					? ve_analyze_fuel_events::gramsToMilligrams(veAnalyzeFuelLastAttemptAeGrams)
					: 0;
	output.veAnalyzeFuelEventLastFilmMg =
			hasVeAnalyzeFuelAttempt && std::isfinite(veAnalyzeFuelLastAttemptFilmGrams)
					? ve_analyze_fuel_events::gramsToMilligrams(veAnalyzeFuelLastAttemptFilmGrams)
					: 0;
	output.veAnalyzeFuelEventLastFinalMg =
			hasVeAnalyzeFuelAttempt && std::isfinite(veAnalyzeFuelLastAttemptFinalGrams)
					? ve_analyze_fuel_events::gramsToMilligrams(veAnalyzeFuelLastAttemptFinalGrams)
					: 0;
	output.veAnalyzeFuelEventLastScheduledMg =
			lastAttemptMatchesRetainedEvent && last.Fresh && !last.SupersededByRejection
					? last.ScheduledCommandMassMilligrams
					: 0;

	for (size_t bankIndex = 0; bankIndex < STFT_BANK_COUNT; bankIndex++) {
		const auto& bank = snapshot.Banks[bankIndex];
		const auto& record = bank.LatestEvent;
		output.veAnalyzeFuelBankBaseMg[bankIndex] = record.CalculatedBaseMassMilligrams;
		output.veAnalyzeFuelBankAeMg[bankIndex] = record.CalculatedAppliedAeMassMilligrams;
		output.veAnalyzeFuelBankFilmMg[bankIndex] = record.CalculatedFilmCorrectionMassMilligrams;
		output.veAnalyzeFuelBankFinalMg[bankIndex] = record.AttemptedFinalCommandedMassMilligrams;
		output.veAnalyzeFuelBankScheduledMg[bankIndex] =
				record.Fresh && !record.SupersededByRejection ? record.ScheduledCommandMassMilligrams : 0;
		output.veAnalyzeFuelBankPeakAePercent[bankIndex] = bank.PeakAePercentOfBase;
		output.veAnalyzeFuelBankPeakFilmPercent[bankIndex] = bank.PeakFilmPercentOfBase;
		output.veAnalyzeFuelBankPeakSumAbsPercent[bankIndex] = bank.PeakCombinedPercentOfBase;
		output.veAnalyzeFuelBankReasonMask[bankIndex] = bank.Reasons;
	}

	for (size_t cylinderIndex = 0; cylinderIndex < MAX_CYLINDER_COUNT; cylinderIndex++) {
		const auto& cylinder = snapshot.Cylinders[cylinderIndex];
		output.veAnalyzeFuelCylinderBaseMg[cylinderIndex] = cylinder.CalculatedBaseMassMilligrams;
		output.veAnalyzeFuelCylinderAeMg[cylinderIndex] = cylinder.CalculatedAppliedAeMassMilligrams;
		output.veAnalyzeFuelCylinderFilmMg[cylinderIndex] = cylinder.CalculatedFilmCorrectionMassMilligrams;
		output.veAnalyzeFuelCylinderFinalMg[cylinderIndex] = cylinder.AttemptedFinalCommandedMassMilligrams;
		output.veAnalyzeFuelCylinderScheduledMg[cylinderIndex] =
				cylinder.Fresh && !cylinder.SupersededByRejection ? cylinder.ScheduledCommandMassMilligrams : 0;
		output.veAnalyzeFuelCylinderReasonMask[cylinderIndex] = cylinder.Reasons;
		output.veAnalyzeFuelCylinderAge[cylinderIndex] =
				cylinder.HasEvent && cylinder.AgeMilliseconds <= UINT16_MAX ? cylinder.AgeMilliseconds : UINT16_MAX;
	}

	for (size_t reasonIndex = 0; reasonIndex < ve_analyze_fuel_events::FuelEventReasonCount && reasonIndex < 25;
		 reasonIndex++) {
		output.veAnalyzeFuelEventRejectCountByReason[reasonIndex] = snapshot.RejectedEventCountsByReason[reasonIndex];
	}
}

static void resetOwnedAirmassRuntimeState(Engine& owner) {
	owner.airmassInjectionState.onEngineStop();

#if EFI_ENGINE_CONTROL
	{
		chibios_rt::CriticalSectionLocker csl;
		for (auto& cylinder : owner.cylinders) {
			cylinder.invalidateFuelProvenance();
		}
	}
#endif // EFI_ENGINE_CONTROL

	auto& state = owner.engineState;
	const efitick_t nowNt = getTimeNowNt();
	state.resetVeAnalyzeSampleGate(getTimeNowNt());
	{
		chibios_rt::CriticalSectionLocker csl;
		state.resetVeAnalyzeFuelEventsLocked(nowNt);
	}
	state.resetVeAnalyzeDiagnostics(
			owner.airmassCalibration.Generation, owner.airmassInjectionState.publicationEpoch());
	state.resetVeAnalyzeDiscardedCalculationDiagnostics();

	auto& output = owner.outputChannels;
	state.exportVeAnalyzeDiagnostics(output);
	output.blendedStatus = static_cast<uint8_t>(owner.airmassInjectionState.status());
	output.blendedFault = static_cast<uint8_t>(owner.airmassInjectionState.fault());
}

Engine::Engine() {
	reset();
}

int Engine::getGlobalConfigurationVersion() const {
	return globalConfigurationVersion;
}

void Engine::reset() {
	++airmassCalibration.Generation;
	airmassCalibration.Known = 0;
	airmassCalibration.ConfigurationVersion = -1;
	airmassCalibration.AuthorityEndpoint = AirmassVeAnalyzeEndpoint::None;
	airmassCalibration.AuthorityEndpointKnown = false;
#ifdef MODULE_CYLINDER_FUEL_TRIM
	cylinderFuelTrimCalibration.reset();
	resetCylinderFuelTrim(*this);
#endif // MODULE_CYLINDER_FUEL_TRIM
	resetOwnedAirmassRuntimeState(*this);
	engineState.airmassLoads = {};
	engineState.airmassLoadsInvalidationEpoch++;
	engineState.airmassLoadsPublicationSerial++;
	clearAirmassConsumerCursorFields(outputChannels);
	/**
	 * it's important for wrapAngle() that engineCycle field never has zero
	 */
	engineState.engineCycle = getEngineCycle(FOUR_STROKE_CRANK_SENSOR);
	resetLua();
}

void Engine::resetLua() {
	// todo: https://github.com/rusefi/rusefi/issues/4308
	engineState.lua = {};
	engineState.lua.fuelAdd = 0;
	engineState.lua.fuelMult = 1;
	engineState.lua.luaDisableEtb = false;
	engineState.lua.luaIgnCut = false;
#ifdef MODULE_BOOST
	module<BoostController>().unmock().resetLua();
#endif // MODULE_BOOST
	ignitionState.luaTimingAdd = 0;
	ignitionState.luaTimingMult = 1;
#if EFI_IDLE_CONTROL
	module<IdleController>().unmock().luaAdd = 0;
	module<IdleTargetController>().unmock().luaAddRpm = 0;
#endif // EFI_IDLE_CONTROL
}

#if EFI_SHAFT_POSITION_INPUT
void Engine::OnTriggerStateProperState(efitick_t nowNt) {
	rpmCalculator.setSpinningUp(nowNt);
}

void Engine::OnTriggerSynchronizationLost() {
	efiPrintf("engine stopped");

	rpmCalculator.setStopSpinning();
	resetOwnedAirmassRuntimeState(*this);
	invalidateAirmassConsumerLoads();

	triggerCentral.triggerState.resetState();
	triggerCentral.instantRpm.resetInstantRpm();

	for (size_t i = 0; i < efi::size(triggerCentral.vvtState); i++) {
		for (size_t j = 0; j < efi::size(triggerCentral.vvtState[0]); j++) {
			triggerCentral.vvtState[i][j].resetState();
		}
	}

	// Reset injector & ignition scheduling to avoid wrong mode or dwell during restart
	injectionEvents.invalidate();
	engine->ignitionEvents.isReady = false;

	// Notify modules that the engine has stopped
	engineModules.apply_all([](auto& m) { m.onEngineStop(); });
}

void Engine::OnTriggerSyncronization(bool wasSynchronized, bool isDecodingError) {
	// TODO: this logic probably shouldn't be part of engine.cpp

	// We only care about trigger shape once we have synchronized trigger. Anything could happen
	// during first revolution and it's fine
	if (wasSynchronized) {
		// 'triggerStateListener is not null' means we are running a real engine and now just preparing trigger shape
		// that's a bit of a hack, a sweet OOP solution would be a real callback or at least 'needDecodingErrorLogic'
		// method?
		if (isDecodingError) {
#if EFI_PROD_CODE
			if (engineConfiguration->verboseTriggerSynchDetails ||
				(triggerCentral.triggerState.someSortOfTriggerError() && !engineConfiguration->silentTriggerError)) {
				efiPrintf(
						"error: synchronizationPoint @ index %lu expected %d/%d got %d/%d",
						triggerCentral.triggerState.currentCycle.current_index,
						triggerCentral.triggerShape.getExpectedEventCount(TriggerWheel::T_PRIMARY),
						triggerCentral.triggerShape.getExpectedEventCount(TriggerWheel::T_SECONDARY),
						triggerCentral.triggerState.currentCycle.eventCount[0],
						triggerCentral.triggerState.currentCycle.eventCount[1]);
			}
#endif /* EFI_PROD_CODE */
		}

		engine->triggerCentral.triggerErrorDetection.add(isDecodingError);
	}
}
#endif

void Engine::injectEngineReferences() {
#if EFI_SHAFT_POSITION_INPUT
	triggerCentral.primaryTriggerConfiguration.update();
	for (int camIndex = 0; camIndex < CAMS_PER_BANK; camIndex++) {
		triggerCentral.vvtTriggerConfiguration[camIndex].update();
	}
#endif // EFI_SHAFT_POSITION_INPUT
}

void Engine::setConfig() {
	efi::clear(config);

	injectEngineReferences();
}

void Engine::efiWatchdog() {
#if EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
	if (isRunningPwmTest) {
		return;
	}

	if (module<PrimeController>()->isPriming()) {
		return;
	}

	if (!getTriggerCentral()->isSpinningJustForWatchdog) {
		if (!isRunningBenchTest() && enginePins.stopPins()) {
			// todo: make this a firmwareError assuming functional tests would run
			warning(ObdCode::CUSTOM_ERR_2ND_WATCHDOG, "Some pins were turned off by 2nd pass watchdog");
		}
		return;
	}

	if (engine->triggerCentral.engineMovedRecently()) {
		// Engine moved recently, no need to safe pins.
		return;
	}

	getTriggerCentral()->isSpinningJustForWatchdog = false;
	ignitionEvents.isReady = false;

	efiPrintf("Engine stopped, safing pins");

	enginePins.stopPins();
#endif // EFI_ENGINE_CONTROL && EFI_SHAFT_POSITION_INPUT
}

void Engine::checkShutdown() {
#if EFI_MAIN_RELAY_CONTROL
	// if we are already in the "ignition_on" mode, then do nothing
	/* this logic is not alive
		if (ignitionOnTimeNt > 0) {
			return;
		}
	todo: move to shutdown_controller.cpp
	*/

	// here we are in the shutdown (the ignition is off) or initial mode (after the firmware fresh start)
	// const efitick_t engineStopWaitTimeoutUs = 500000LL;	// 0.5 sec
	// in shutdown mode, we need a small cooldown time between the ignition off and on
/* this needs work or tests
todo: move to shutdown_controller.cpp
	if (stopEngineRequestTimeNt == 0 || (getTimeNowNt() - stopEngineRequestTimeNt) > US2NT(engineStopWaitTimeoutUs)) {
		// if the ignition key is turned on again,
		// we cancel the shutdown mode, but only if all shutdown procedures are complete
		const float vBattThresholdOn = 8.0f;
		// we fallback into zero instead of VBAT_FALLBACK_VALUE because it's not safe to false-trigger the "ignition on"
event,
		// and we want to turn on the main relay only when 100% sure.
		if ((Sensor::getOrZero(SensorType::BatteryVoltage) > vBattThresholdOn) && !isInShutdownMode()) {
			ignitionOnTimeNt = getTimeNowNt();
			efiPrintf("Ignition voltage detected!");
			if (stopEngineRequestTimeNt != 0) {
				efiPrintf("Cancel the engine shutdown!");
				stopEngineRequestTimeNt = 0;
			}
		}
	}
*/
#endif /* EFI_MAIN_RELAY_CONTROL */
}

bool Engine::isInShutdownMode() const {
	// TODO: this logic is currently broken
#if 0 && EFI_MAIN_RELAY_CONTROL && EFI_PROD_CODE
	// if we are in "ignition_on" mode and not in shutdown mode
	if (stopEngineRequestTimeNt == 0 && ignitionOnTimeNt > 0) {
		const float vBattThresholdOff = 5.0f;
		// start the shutdown process if the ignition voltage dropped low
		if (Sensor::get(SensorType::BatteryVoltage).value_or(VBAT_FALLBACK_VALUE) <= vBattThresholdOff) {
			scheduleStopEngine();
		}
	}

	// we are not in the shutdown mode?
	if (stopEngineRequestTimeNt == 0) {
		return false;
	}

	const efitick_t turnOffWaitTimeoutNt = NT_PER_SECOND;
	// We don't want any transients to step in, so we wait at least 1 second whatever happens.
	// Also it's good to give the stepper motor some time to start moving to the initial position (or parking)
	if ((getTimeNowNt() - stopEngineRequestTimeNt) < turnOffWaitTimeoutNt)
		return true;

	const efitick_t engineSpinningWaitTimeoutNt = 5 * NT_PER_SECOND;
	// The engine is still spinning! Give it some time to stop (but wait no more than 5 secs)
	if (isSpinning && (getTimeNowNt() - stopEngineRequestTimeNt) < engineSpinningWaitTimeoutNt)
		return true;

	// The idle motor valve is still moving! Give it some time to park (but wait no more than 10 secs)
	// Usually it can move to the initial 'cranking' position or zero 'parking' position.
	const efitick_t idleMotorWaitTimeoutNt = 10 * NT_PER_SECOND;
	if (isIdleMotorBusy() && (getTimeNowNt() - stopEngineRequestTimeNt) < idleMotorWaitTimeoutNt)
		return true;
#endif /* EFI_MAIN_RELAY_CONTROL */
	return false;
}

bool Engine::isMainRelayEnabled() const {
#if EFI_MAIN_RELAY_CONTROL
	return enginePins.mainRelay.getLogicValue();
#else
	// if no main relay control, we assume it's always turned on
	return true;
#endif /* EFI_MAIN_RELAY_CONTROL */
}

injection_mode_e getCurrentInjectionMode() {
	if (getEngineRotationState()->isCranking()) {
		return engineConfiguration->crankingInjectionMode;
	}

	auto runningMode = engineConfiguration->injectionMode;

#if EFI_SHAFT_POSITION_INPUT
	if (runningMode == IM_SEQUENTIAL) {
		bool missingPhaseInfoForSequential = !engine->triggerCentral.triggerState.hasSynchronizedPhase();

		bool willGetSequentialInfoLater = engine->triggerCentral.triggerState.expectDisambiguation();

		// IF
		// - We do not currently have full sync
		// - AND we expect to get it later (ie, once the cam syncs)
		// THEN hold off on sequential, and stay in batch fueling for now
		if (missingPhaseInfoForSequential && willGetSequentialInfoLater) {
			return IM_BATCH;
		}
	}
#endif /* EFI_SHAFT_POSITION_INPUT */

	return runningMode;
}

/**
 * The idea of this method is to execute all heavy calculations in a lower-priority thread,
 * so that trigger event handler/IO scheduler tasks are faster.
 */
void Engine::periodicFastCallback() {
	ScopePerf pc(PE::EnginePeriodicFastCallback);

	engineState.periodicFastCallback();

	speedoUpdate();

	engineModules.apply_all([](auto& m) { m.onFastCallback(); });

	// LimpManager's current cut state is updated by the module pass above.
	engineState.updateVeAnalyzeSampleGate(getTimeNowNt());
}

EngineRotationState* getEngineRotationState() {
	return &engine->rpmCalculator;
}

EngineState* getEngineState() {
	return &engine->engineState;
}

TunerStudioOutputChannels* getTunerStudioOutputChannels() {
	return &engine->outputChannels;
}

Scheduler* getScheduler() {
	return &engine->scheduler;
}

#if EFI_SHAFT_POSITION_INPUT
TriggerCentral* getTriggerCentral() {
	return &engine->triggerCentral;
}
#endif // EFI_SHAFT_POSITION_INPUT

LimpManager* getLimpManager() {
	return &engine->module<LimpManager>().unmock();
}

#if EFI_ENGINE_CONTROL
FuelSchedule* getFuelSchedule() {
	return &engine->injectionEvents;
}

IgnitionEventList* getIgnitionEvents() {
	return &engine->ignitionEvents;
}
#endif // EFI_ENGINE_CONTROL
