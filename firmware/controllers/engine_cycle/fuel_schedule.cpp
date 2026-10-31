/**
 * @file fuel_schedule.cpp
 *
 * Handles injection scheduling
 */

#include "pch.h"
#include "fuel_math.h"

#if EFI_ENGINE_CONTROL

extern bool printFuelDebug;

void startInjection(InjectorContext ctx) {
	forEachSetBit(ctx.outputsMask, [&ctx](size_t idx) {
		enginePins.injectors[idx].open();

		if (ctx.stage2Active) {
			enginePins.injectorsStage2[idx].open();
		}
	});
}

void endInjection(InjectorContext ctx) {
	forEachSetBit(ctx.outputsMask, [](size_t idx) { enginePins.injectors[idx].close(); });

	if (ctx.splitDurationUs > 0) {
		efitick_t openTime = getTimeNowNt() + MS2NT(2);
		efitick_t closeTime = openTime + US2NT(ctx.splitDurationUs);

		// Zero out the split duration so it doesn't repeat
		ctx.splitDurationUs = 0;

		ScheduledAction events[] = {
				{openTime, {scheduledStartInjection, ctx}},
				{closeTime, {scheduledEndInjection, ctx}},
		};
		// Split continuation is new work and must pass admission as one pair.
		scheduleFuelCallbacks(events, efi::size(events));
	} else {
		// No splits remaining, prepare for next cycle
		if (ctx.eventIndex < efi::size(getFuelSchedule()->elements) && engine->airmassInjectionState.allowInjection()) {
			getFuelSchedule()->elements[ctx.eventIndex].update();
		}
	}
}

void endInjectionStage2(InjectorContext ctx) {
	forEachSetBit(ctx.outputsMask, [](size_t idx) { enginePins.injectorsStage2[idx].close(); });
}

void scheduledStartInjection(InjectorContext ctx) {
	// Accepted work still owns its output even if admission closes before it runs.
	startInjection(ctx);
	engine->airmassInjectionState.callbackCompleted();
}

void scheduledEndInjection(InjectorContext ctx) {
	// endInjection reserves any split continuation before this callback acknowledges.
	endInjection(ctx);
	engine->airmassInjectionState.callbackCompleted();
}

void scheduledEndInjectionStage2(InjectorContext ctx) {
	endInjectionStage2(ctx);
	engine->airmassInjectionState.callbackCompleted();
}

uint16_t InjectionEvent::calculateInjectorOutputMask() const {
	uint16_t mask = 0;

	switch (m_injectionMode) {
		case IM_SIMULTANEOUS:
			// Simultaneous mode fires all injectors
			mask = (1 << engine->engineState.cylinderCount) - 1;
			break;
		case IM_SINGLE_POINT:
			// Single point only fires injector 1
			mask = 1;
			break;
		case IM_BATCH:
			// In batch mode, also fire the cylinder 360 degrees out to support "two-wire batch" mode

			// Compute the position of this cylinder's twin in the firing order
			// Each injector gets fired as a primary (the same as sequential), but also
			// fires the injector 360 degrees later in the firing order.
			mask |=
					(1 << getCylinderNumberAtIndex(
							 (ownIndex + (engine->engineState.cylinderCount / 2)) % engine->engineState.cylinderCount));

			// falls through
		case IM_SEQUENTIAL:
			// In batch+sequential, fire this cylinder's injector
			mask |= 1 << cylinderNumber;
			break;
	}

	return mask;
}

static uint16_t getConfiguredCylinderMask() {
	const uint8_t cylinderCount = engine->engineState.cylinderCount;
	if (cylinderCount == 0 || cylinderCount > MAX_CYLINDER_COUNT) {
		return 0;
	}
	return static_cast<uint16_t>((1u << cylinderCount) - 1u);
}

static uint8_t getAffectedBankMask(uint16_t cylinderMask) {
	uint8_t bankMask = 0;
	for (size_t cylinderIndex = 0; cylinderIndex < MAX_CYLINDER_COUNT; cylinderIndex++) {
		if ((cylinderMask & (1u << cylinderIndex)) == 0) {
			continue;
		}

		const uint8_t bank = engineConfiguration->cylinderBankSelect[cylinderIndex];
		if (bank < STFT_BANK_COUNT) {
			bankMask |= static_cast<uint8_t>(1u << bank);
		}
	}
	return bankMask;
}

void InjectionEvent::onTriggerTooth(const EnginePhaseInfo& phase) {
	auto eventAngle = injectionStartAngle;

	// Determine whether our angle is going to happen before (or near) the next tooth
	if (!isPhaseInRange(EngPhase{eventAngle}, phase) || !engine->airmassInjectionState.allowInjection()) {
		return;
	}

	mass_t cycleMassGrams;
	CylinderFuelProvenance provenance;
	uint32_t currentCalibrationGeneration;
	{
		// The publication owner replaces these two values together. Capture that
		// exact pair so this event cannot combine a new mass with old provenance.
		chibios_rt::CriticalSectionLocker csl;
		cycleMassGrams = engine->cylinders[this->cylinderNumber].getInjectionMass();
		provenance = engine->cylinders[this->cylinderNumber].getFuelProvenance();
		currentCalibrationGeneration = engine->airmassCalibration.Generation;
	}

	// don't allow split inj in simultaneous mode
	bool doSplitInjection = engine->engineState.doSplitInjection && !isSimultaneous;

	// Select fuel mass from the correct cylinder
	const float modeMultiplier = getInjectionModeDurationMultiplier(m_injectionMode);
	const float inputEventMassGrams = cycleMassGrams * modeMultiplier;
	float injectionMassGrams = inputEventMassGrams;

	// Perform wall wetting adjustment on fuel mass, not duration, so that
	// it's correct during fuel pressure (injector flow) or battery voltage (deadtime) transients
	// TODO: is it correct to wall wet on both pulses?
	injectionMassGrams = wallFuel.adjust(injectionMassGrams);
	const bool wallModelZeroClamp = wallFuel.wasZeroClamped();

	// Disable staging in simultaneous mode or split injection mode
	float stage2Fraction = (isSimultaneous || doSplitInjection) ? 0 : getEngineState()->injectionStage2Fraction;

	// Compute fraction of fuel on stage 2, remainder goes on stage 1
	const float injectionMassStage2 = stage2Fraction * injectionMassGrams;
	float injectionMassStage1 = injectionMassGrams - injectionMassStage2;

	{
		// Log this fuel as consumed

#ifdef MODULE_TRIP_ODO
		int numberOfInjections = getNumberOfInjections(m_injectionMode);

		float actualInjectedMass = numberOfInjections * (injectionMassStage1 + injectionMassStage2);

		engine->module<TripOdometer>()->consumeFuel(actualInjectedMass, phase.timestamp);
#endif // MODULE_TRIP_ODO
	}

	if (doSplitInjection) {
		// If in split mode, do the injection in two halves
		injectionMassStage1 = injectionMassStage1 / 2;
	}

	const uint16_t affectedCylinderMask = calculateInjectorOutputMask();
	const uint16_t expectedCylinderMask = getConfiguredCylinderMask();
	const uint8_t affectedBankMask = getAffectedBankMask(affectedCylinderMask);
	const uint8_t expectedBankMask = getAffectedBankMask(expectedCylinderMask);
	const bool provenanceCurrent = provenance.Valid && provenance.CalibrationGeneration != 0 &&
								   provenance.CalibrationGeneration == currentCalibrationGeneration;

	VeAnalyzeFuelEventInput fuelEventEvidence;
	fuelEventEvidence.OwnerCylinder = this->cylinderNumber;
	fuelEventEvidence.AffectedCylinderMask = affectedCylinderMask;
	fuelEventEvidence.AffectedBankMask = affectedBankMask;
	fuelEventEvidence.ExpectedCylinderMask = expectedCylinderMask;
	fuelEventEvidence.ExpectedBankMask = expectedBankMask;
	// Unknown or stale provenance deliberately publishes the reserved zero
	// generation. That closes the evidence gate without touching injection.
	fuelEventEvidence.PublicationGeneration = provenanceCurrent ? provenance.CalibrationGeneration : 0;
	fuelEventEvidence.Timestamp = phase.timestamp;
	fuelEventEvidence.BasePreAeMassGrams = provenanceCurrent ? provenance.PreAeBaseFuelMass * modeMultiplier : 0;
	fuelEventEvidence.AppliedAeMassGrams = provenanceCurrent ? provenance.AppliedAeFuelMass * modeMultiplier : 0;
	fuelEventEvidence.FilmCorrectionMassGrams = injectionMassGrams - inputEventMassGrams;
	fuelEventEvidence.FinalCommandedMassGrams = injectionMassGrams;
	fuelEventEvidence.HasStage1Mass = true;
	fuelEventEvidence.Stage1MassGrams = injectionMassStage1;
	fuelEventEvidence.HasStage2Mass = injectionMassStage2 > 0;
	fuelEventEvidence.Stage2MassGrams = injectionMassStage2;
	fuelEventEvidence.WallModelZeroClamp = wallModelZeroClamp;
	fuelEventEvidence.SchedulerAccepted = false;
	fuelEventEvidence.IntegrationUnsupported = doSplitInjection || m_injectionMode == IM_SINGLE_POINT;

	auto observeFuelAttempt = [&fuelEventEvidence](bool schedulerAccepted) {
		fuelEventEvidence.SchedulerAccepted = schedulerAccepted;
		chibios_rt::CriticalSectionLocker csl;
		engine->engineState.observeVeAnalyzeFuelEventLocked(fuelEventEvidence);
	};

	const floatms_t injectionDurationStage1 =
			engine->module<InjectorModelPrimary>()->getInjectionDuration(injectionMassStage1);
	const floatms_t injectionDurationStage2 =
			injectionMassStage2 > 0
					? engine->module<InjectorModelSecondary>()->getInjectionDuration(injectionMassStage2)
					: 0;

#if EFI_PRINTF_FUEL_DETAILS
	if (printFuelDebug) {
		printf("fuel injectionDuration=%.2fms adjusted=%.2fms\n",
			   getEngineState()->injectionDuration,
			   injectionDurationStage1);
	}
#endif /*EFI_PRINTF_FUEL_DETAILS */

	if (this->cylinderNumber == 0) {
		engine->outputChannels.actualLastInjection = injectionDurationStage1;
		engine->outputChannels.actualLastInjectionStage2 = injectionDurationStage2;
	}

	if (!std::isfinite(injectionDurationStage1) || !std::isfinite(injectionDurationStage2)) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		warning(ObdCode::CUSTOM_OBD_NAN_INJECTION, "Nonfinite injection pulse");
		observeFuelAttempt(false);
		return;
	}
	if (injectionDurationStage1 < 0 || injectionDurationStage2 < 0) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		warning(ObdCode::CUSTOM_OBD_NEG_INJECTION, "Negative injection pulse %.2f", injectionDurationStage1);
		observeFuelAttempt(false);
		return;
	}

	// If somebody commanded an impossibly short injection, do nothing.
	// Durations under 50us-ish aren't safe for the scheduler
	// as their order may be swapped, resulting in a stuck open injector
	// see https://github.com/rusefi/rusefi/pull/596 for more details
	if (injectionDurationStage1 < 0.050f) {
		observeFuelAttempt(false);
		return;
	}

	floatus_t durationUsStage1 = MS2US(injectionDurationStage1);
	floatus_t durationUsStage2 = MS2US(injectionDurationStage2);

	// Bound every float before conversion. Split duration has only 15 bits in InjectorContext.
	if (!std::isfinite(durationUsStage1) || !std::isfinite(durationUsStage2) ||
		durationUsStage1 >= MaximumScheduleDelayUs || durationUsStage2 >= MaximumScheduleDelayUs ||
		(doSplitInjection && durationUsStage1 > 32767)) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		observeFuelAttempt(false);
		return;
	}

	int32_t durationUsStage1Int = static_cast<int32_t>(durationUsStage1);
	int32_t durationUsStage2Int = static_cast<int32_t>(durationUsStage2);

	// Only bother with the second stage if it's long enough to be relevant
	bool hasStage2Injection = durationUsStage2 > 50;
	// A real first-stage pulse cannot make a suppressed second-stage pulse
	// disappear from the command identity.
	const bool stage2Suppressed = injectionMassStage2 > 0 && !hasStage2Injection;
	fuelEventEvidence.IntegrationUnsupported |= stage2Suppressed;

	InjectorContext ctx;
	ctx.eventIndex = ownIndex;
	ctx.stage2Active = hasStage2Injection;
	ctx.outputsMask = affectedCylinderMask;

	if (doSplitInjection) {
		ctx.splitDurationUs = durationUsStage1;
	}

#if EFI_PRINTF_FUEL_DETAILS
	if (printFuelDebug) {
		printf("handleFuelInjectionEvent fuelout %06x injection_duration %dus engineCycleDuration=%.1fms\t\n",
			   ctx.outputsMask,
			   (int)durationUsStage1,
			   (int)MS2US(getCrankshaftRevolutionTimeMs(Sensor::getOrZero(SensorType::Rpm))) / 1000.0);
	}
#endif /*EFI_PRINTF_FUEL_DETAILS */

	// Correctly wrap injection start angle
	float angleFromNow = eventAngle - phase.currentEngPhase.angle;
	if (angleFromNow < 0) {
		angleFromNow += getEngineState()->engineCycle;
	}

	float delayUs = engine->rpmCalculator.oneDegreeUs * angleFromNow;
	if (!std::isfinite(delayUs) || delayUs < 0 || delayUs >= MaximumScheduleDelayUs) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		observeFuelAttempt(false);
		return;
	}

	// Preserve scheduleByAngle's fractional-microsecond conversion after bounding it.
	static_assert(static_cast<int64_t>(MaximumScheduleDelayUs) * US_TO_NT_MULTIPLIER < INT32_MAX);
	float delayNtFloat = USF2NT(delayUs);
	if (!std::isfinite(delayNtFloat) || delayNtFloat < 0 || delayNtFloat > INT32_MAX) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		observeFuelAttempt(false);
		return;
	}
	int32_t delayNt = static_cast<int32_t>(delayNtFloat);
	int64_t longestDurationUs = durationUsStage1Int;
	if (hasStage2Injection) {
		longestDurationUs = std::max<int64_t>(longestDurationUs, durationUsStage2Int);
	}
	int64_t longestDurationNt = US2NT(longestDurationUs);
	if (phase.timestamp.count > INT64_MAX - delayNt - longestDurationNt) {
		engine->airmassInjectionState.rejectCalculation(AirmassInjectionFault::Scheduling);
		observeFuelAttempt(false);
		return;
	}

	efitick_t startTime = phase.timestamp + efidur_t{delayNt};
	ScheduledAction events[MaxScheduleBatchSize] = {
			{startTime, {scheduledStartInjection, ctx}},
			{startTime + US2NT(durationUsStage1Int), {scheduledEndInjection, ctx}},
	};
	size_t count = 2;
	if (hasStage2Injection) {
		events[count++] = {
				startTime + US2NT(durationUsStage2Int),
				{scheduledEndInjectionStage2, ctx},
		};
		// The executor requires chronological order when stage 2 ends first.
		if (events[2].time < events[1].time) {
			std::swap(events[1], events[2]);
		}
	}
	const bool schedulerAccepted = scheduleFuelCallbacks(events, count);
	observeFuelAttempt(schedulerAccepted);

#if EFI_UNIT_TEST
	printf("scheduling injection angle=%.2f/delay=%d injectionDuration=%d %d\r\n",
		   angleFromNow,
		   (int)NT2US(startTime - phase.timestamp),
		   (int)durationUsStage1,
		   (int)durationUsStage2);
#endif
}

FuelSchedule::FuelSchedule() {
	for (int cylinderIndex = 0; cylinderIndex < MAX_CYLINDER_COUNT; cylinderIndex++) {
		elements[cylinderIndex].setIndex(cylinderIndex);
	}
}

WallFuel& InjectionEvent::getWallFuel() {
	return wallFuel;
}

void FuelSchedule::invalidate() {
	isReady = false;
}

// Determines how much to adjust injection opening angle based on the injection's duration and the current phasing mode
static float getInjectionAngleCorrection(float fuelMs, float oneDegreeUs) {
	auto mode = engineConfiguration->injectionTimingMode;
	if (mode == InjectionTimingMode::Start) {
		// Start of injection gets no correction for duration
		return 0;
	}

	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(fuelMs), "NaN fuelMs", false);

	angle_t injectionDurationAngle = MS2US(fuelMs) / oneDegreeUs;
	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(injectionDurationAngle), "NaN injectionDurationAngle", false);
	assertAngleRange(injectionDurationAngle, "injectionDuration_r", ObdCode::CUSTOM_INJ_DURATION);

	if (mode == InjectionTimingMode::Center) {
		// Center of injection is half-corrected for duration
		return injectionDurationAngle * 0.5f;
	} else {
		// End of injection gets "full correction" so we advance opening by the full duration
		return injectionDurationAngle;
	}
}

// Returns the start angle of this injector in engine coordinates (0-720 for a 4 stroke),
// or unexpected if unable to calculate the start angle due to missing information.
expected<angle_t> OneCylinder::computeInjectionAngle(injection_mode_e mode) const {
	floatus_t oneDegreeUs = getEngineRotationState()->getOneDegreeUs();
	if (std::isnan(oneDegreeUs)) {
		// in order to have fuel schedule we need to have current RPM
		return unexpected;
	}

	// injection phase may be scheduled by injection end, so we need to step the angle back
	// for the duration of the injection
	angle_t injectionDurationAngle = getInjectionAngleCorrection(getEngineState()->injectionDuration, oneDegreeUs);

	// User configured offset - degrees after TDC combustion
	floatus_t injectionOffset = getEngineState()->injectionOffset;
	if (std::isnan(injectionOffset)) {
		// injection offset map not ready - we are not ready to schedule fuel events
		return unexpected;
	}

	angle_t openingAngle = injectionOffset - injectionDurationAngle;
	assertAngleRange(openingAngle, "openingAngle_r", ObdCode::CUSTOM_ERR_6554);
	wrapAngle(openingAngle, "addFuel#1", ObdCode::CUSTOM_ERR_6555);
	// TODO: should we log per-cylinder injection timing? #76
	getTunerStudioOutputChannels()->injectionOffset = openingAngle;

	// Convert from cylinder-relative to cylinder-1-relative
	openingAngle += getAngleOffset();

	efiAssert(ObdCode::CUSTOM_ERR_ASSERT, !std::isnan(openingAngle), "findAngle#3", false);
	assertAngleRange(openingAngle, "findAngle#a33", ObdCode::CUSTOM_ERR_6544);

	wrapAngle(openingAngle, "addFuel#2", ObdCode::CUSTOM_ERR_6555);

#if EFI_UNIT_TEST
	printf("registerInjectionEvent openingAngle=%.2f inj %d\r\n", openingAngle, m_cylinderNumber);
#endif

	return openingAngle;
}

bool InjectionEvent::updateInjectionAngle(injection_mode_e mode) {
	if (auto result = engine->cylinders[cylinderNumber].computeInjectionAngle(mode)) {
		// If injector duty cycle is high, lock injection SOI so that we
		// don't miss injections at or above 100% duty
		if (getEngineState()->shouldUpdateInjectionTiming) {
			injectionStartAngle = result.Value;
		}

		return true;
	} else {
		return false;
	}
}

/**
 * @returns false in case of error, true if success
 */
bool InjectionEvent::update() {
	cylinderNumber = getCylinderNumberAtIndex(ownIndex);

	injection_mode_e mode = getCurrentInjectionMode();

	if (updateInjectionAngle(mode)) {
		m_injectionMode = mode;

		engine->outputChannels.currentInjectionMode = static_cast<uint8_t>(mode);

		return true;
	} else {
		return false;
	}
}

void FuelSchedule::addFuelEvents() {
	for (size_t cylinderIndex = 0; cylinderIndex < engine->engineState.cylinderCount; cylinderIndex++) {
		bool result = elements[cylinderIndex].update();

		if (!result) {
			invalidate();
			return;
		}
	}

	// We made it through all cylinders, mark the schedule as ready so it can be used
	isReady = true;
}

void FuelSchedule::onTriggerTooth(const EnginePhaseInfo& phase) {
	// Wait for schedule to be built - this happens the first time we get RPM
	if (!isReady) {
		return;
	}

	for (size_t i = 0; i < engine->engineState.cylinderCount; i++) {
		elements[i].onTriggerTooth(phase);
	}
}

#endif // EFI_ENGINE_CONTROL
