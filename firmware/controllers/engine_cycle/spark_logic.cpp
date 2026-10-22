/*
 * @file spark_logic.cpp
 *
 * @date Sep 15, 2016
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "spark_logic.h"
#include "ignition_retarget.h"

#include "event_queue.h"

#include "knock_logic.h"

#if EFI_ENGINE_CONTROL

/**
 * @param cylinderIndex from 0 to cylinderCount, not cylinder number
 */
static int getIgnitionPinForIndex(int cylinderIndex, ignition_mode_e ignitionMode) {
	switch (ignitionMode) {
		case IM_ONE_COIL:
			return 0;
		case IM_WASTED_SPARK: {
			if (engine->engineState.cylinderCount == 1) {
				// we do not want to divide by zero
				return 0;
			}
			return cylinderIndex % (engine->engineState.cylinderCount / 2);
		}
		case IM_INDIVIDUAL_COILS:
			return cylinderIndex;
		case IM_TWO_COILS:
			return cylinderIndex % 2;

		default:
			firmwareError(
					ObdCode::CUSTOM_OBD_IGNITION_MODE,
					"Invalid ignition mode getIgnitionPinForIndex(): %d",
					engineConfiguration->ignitionMode);
			return 0;
	}
}

angle_t OneCylinder::getSparkAngle(angle_t lateAdjustment) const {
	// Compute the final ignition timing including all "late" adjustments
	angle_t finalIgnitionTiming = m_timingAdvance + lateAdjustment;

	// 10 ATDC ends up as 710, convert it to -10 so we can log and clamp correctly
	if (finalIgnitionTiming > 360) {
		finalIgnitionTiming -= 720;
	}

	// Clamp the final ignition timing to the configured limits
	// finalIgnitionTiming is deg BTDC
	// minimumIgnitionTiming limits maximium retard
	// maximumIgnitionTiming limits maximum advance
	finalIgnitionTiming = std::clamp<float>(
			finalIgnitionTiming,
			engineConfiguration->minimumIgnitionTiming,
			engineConfiguration->maximumIgnitionTiming);

	engine->outputChannels.ignitionAdvanceCyl[m_cylinderNumber] = finalIgnitionTiming;

	return
			// Negate because timing *before* TDC, and we schedule *after* TDC
			-finalIgnitionTiming
			// Offset by this cylinder's position in the cycle
			+ getAngleOffset();
}

uint16_t IgnitionEvent::calculateIgnitionOutputMask() const {
	const int index = getIgnitionPinForIndex(cylinderIndex, m_ignitionMode);
	const int coilIndex = getCylinderNumberAtIndex(index);

	uint16_t outputsMask = 1 << coilIndex;

	// If wasted spark, find the paired coil in addition to "main" output for this cylinder.
	// Skip in pairedOddFireWastedSpark mode: a single physical coil is shared between companion cylinders, so only the
	// first-half (lower firing-order index) cylinder slot gets a pin assigned and we drive only that one OutputPin from
	// both events.
	if (m_ignitionMode == IM_WASTED_SPARK && !engineConfiguration->pairedOddFireWastedSpark) {
		int secondIndex = index + engine->engineState.cylinderCount / 2;
		int secondCoilIndex = getCylinderNumberAtIndex(secondIndex);
		outputsMask |= 1 << secondCoilIndex;
	}

	return outputsMask;
}

angle_t IgnitionEvent::calculateSparkAngle() const {
	angle_t sparkAngle = engine->cylinders[cylinderNumber].getSparkAngle(
			// Pull any extra timing for knock retard
			-engine->module<KnockController>()->getKnockRetard());

	efiAssert(ObdCode::CUSTOM_SPARK_ANGLE_1, !std::isnan(sparkAngle), "sparkAngle#1", 0);
	wrapAngle(sparkAngle, "findAngle#2", ObdCode::CUSTOM_ERR_6550);

	return sparkAngle;
}

static void prepareCylinderIgnitionSchedule(angle_t dwellAngleDuration, floatms_t sparkDwell, IgnitionEvent& event) {
	// todo: clean up this implementation? does not look too nice as is.

	const int realCylinderNumber = getCylinderNumberAtIndex(event.cylinderIndex);

	// let's save planned duration so that we can later compare it with reality
	event.sparkDwell = sparkDwell;

	// Stash which cylinder we're scheduling so that knock sensing knows which
	// cylinder just fired
	event.cylinderNumber = realCylinderNumber;

	auto sparkAngle = event.calculateSparkAngle();

	auto ignitionMode = getCurrentIgnitionMode();

	// On an odd cylinder (or odd fire) wasted spark engine, map outputs as if in sequential.
	// During actual scheduling, the events just get scheduled every 360 deg instead
	// of every 720 deg.
	if (ignitionMode == IM_WASTED_SPARK && engine->engineState.useOddFireWastedSpark) {
		ignitionMode = IM_INDIVIDUAL_COILS;
	}

	angle_t dwellStartAngle = sparkAngle - dwellAngleDuration;
	efiAssertVoid(ObdCode::CUSTOM_ERR_6590, !std::isnan(dwellStartAngle), "findAngle#5");

	assertAngleRange(dwellStartAngle, "findAngle dwellStartAngle", ObdCode::CUSTOM_ERR_6550);
	wrapAngle(dwellStartAngle, "findAngle#7", ObdCode::CUSTOM_ERR_6550);

	event.m_ignitionMode = ignitionMode;
	event.dwellAngle = dwellStartAngle;

	engine->outputChannels.currentIgnitionMode = static_cast<uint8_t>(ignitionMode);
}

static bool newIgnitionChargeAllowed() {
	return engineConfiguration->isIgnitionEnabled && !hasFirmwareError() && getLimpManager()->allowIgnition().value;
}

static void fireTrailingSpark(IgnitionContext ctx) {
	chibios_rt::CriticalSectionLocker csl;
	if (ctx.eventIndex >= MAX_CYLINDER_COUNT) {
		return;
	}
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
	if (!event.trailingPending || ctx.generation != event.trailingGeneration) {
		return;
	}

	// Invalidate before cancellation so an extracted duplicate cannot revive this cycle.
	event.trailingPending = false;
	event.trailingCharging = false;
	engine->scheduler.cancel(&event.trailingSparkCharge);
	engine->scheduler.cancel(&event.trailingSparkFire);
	enginePins.trailingCoils[event.trailingCylinder].discharge(ctx.owner());
}

static void chargeTrailingSpark(IgnitionContext ctx) {
	chibios_rt::CriticalSectionLocker csl;
	if (ctx.eventIndex >= MAX_CYLINDER_COUNT) {
		return;
	}
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
	if (!event.trailingPending || ctx.generation != event.trailingGeneration) {
		return;
	}

	auto& pin = enginePins.trailingCoils[event.trailingCylinder];
	// A copied callback may execute after the original already charged this cycle.
	if (event.trailingCharging || pin.ownedBy(ctx.owner())) {
		return;
	}

	if (!newIgnitionChargeAllowed() || !engineConfiguration->enableTrailingSparks) {
		fireTrailingSpark(ctx);
		return;
	}

	if (!pin.charge(ctx.owner(), event.trailingDwell, {fireTrailingSpark, ctx})) {
		event.contentionCount++;
		fireTrailingSpark(ctx);
		return;
	}

	event.trailingCharging = true;
}

uint16_t IgnitionContext::outputsMask() const {
	return eventIndex < MAX_CYLINDER_COUNT ? engine->ignitionEvents.elements[eventIndex].outputMaskSnapshot : 0;
}

static bool isCurrent(IgnitionContext ctx, const IgnitionEvent& event) {
	return ctx.generation == event.generation && event.state != IgnitionOccurrenceState::Closed;
}

static void nextChargeGeneration(IgnitionEvent& event) {
	event.chargeGeneration = (event.chargeGeneration + 1) & 0x7ffffff;
	if (!event.chargeGeneration) {
		event.chargeGeneration = 1;
	}
}

static IgnitionContext chargeContext(IgnitionContext ctx, const IgnitionEvent& event) {
	ctx.generation = event.chargeGeneration;
	return ctx;
}

static void nextFireGeneration(IgnitionEvent& event) {
	event.fireGeneration = (event.fireGeneration + 1) & 0x7ffffff;
	if (!event.fireGeneration) {
		event.fireGeneration = 1;
	}
}

static IgnitionContext beginOccurrence(IgnitionEvent& event) {
	nextChargeGeneration(event);
	nextFireGeneration(event);
	event.generation = (event.generation + 1) & 0x7ffffff;
	if (!event.generation) {
		event.generation = 1;
	}
	IgnitionContext ctx;
	ctx.eventIndex = event.cylinderIndex;
	ctx.generation = event.generation;
	event.state = IgnitionOccurrenceState::ChargePending;
	return ctx;
}

static IgnitionContext trailingContext(const IgnitionEvent& event) {
	IgnitionContext ctx;
	ctx.eventIndex = event.cylinderIndex;
	ctx.generation = event.trailingGeneration;
	return ctx;
}

static void closeOccurrence(IgnitionEvent& event) {
	// Invalidate BEFORE cancellation: an executor may already have copied action.
	const auto priorState = event.state;
	event.state = IgnitionOccurrenceState::Closed;
	event.candidateValid = false;

	if (priorState == IgnitionOccurrenceState::ChargePending) {
		// A multispark handoff can leave the trailing coil HIGH while the next main
		// occurrence is still pending. Retire that lifecycle with this occurrence.
		const bool wasTrailingCharging = event.trailingCharging;
		event.trailingPending = false;
		event.trailingCharging = false;
		engine->scheduler.cancel(&event.trailingSparkCharge);
		engine->scheduler.cancel(&event.trailingSparkFire);
		if (wasTrailingCharging) {
			enginePins.trailingCoils[event.trailingCylinder].discharge(trailingContext(event).owner());
		}
	}

	engine->scheduler.cancel(&event.dwellStartTimer);
	engine->module<TriggerScheduler>()->cancel(&event.sparkEvent);
	engine->scheduler.cancel(&event.sparkEvent.scheduling);
	event.sparkEvent.fallbackIsCurrent = false;
}

static void fireRetargetedSpark(IgnitionContext ctx) {
	chibios_rt::CriticalSectionLocker csl;
	if (ctx.eventIndex >= MAX_CYLINDER_COUNT) {
		return;
	}
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
	if (event.state == IgnitionOccurrenceState::Closed || ctx.generation != event.fireGeneration) {
		return;
	}
	ctx.generation = event.generation;
	fireSparkAndPrepareNextSchedule(ctx);
}

static action_s firingAction(IgnitionEvent& event, IgnitionContext ctx) {
	if (event.plannedByTime && event.candidateValid) {
		ctx.generation = event.fireGeneration;
		return {fireRetargetedSpark, ctx};
	}
	return {fireSparkAndPrepareNextSchedule, ctx};
}

void fireSparkAndPrepareNextSchedule(IgnitionContext ctx) {
	chibios_rt::CriticalSectionLocker csl;
	if (ctx.eventIndex >= MAX_CYLINDER_COUNT) {
		return;
	}
	efitick_t nowNt = getTimeNowNt();
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
	if (!isCurrent(ctx, event)) {
		return;
	}

	const auto owner = ctx.owner();
	float actualDwellMs = 0;
	efitick_t minimumFire = nowNt;
	efitick_t hardDeadline = nowNt + US2NT(1000000);
	bool anyCharged = false;
	forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) {
		auto& pin = enginePins.coils[idx];
		if (pin.ownedBy(owner)) {
			anyCharged = true;
			actualDwellMs = std::max(actualDwellMs, NT2USF(nowNt - pin.firstHigh()) * 0.001f);
			minimumFire = std::max<efitick_t>(
					minimumFire, pin.firstHigh() + static_cast<uint32_t>(MSF2NT(0.8f * event.occurrenceDwell)));
			hardDeadline = std::min(hardDeadline, pin.hardDeadline());
		}
	});

	// Any physical hard deadline wins over every output's minimum dwell.
	if (!ctx.isOverdwellProtect && anyCharged && nowNt < hardDeadline && nowNt < minimumFire) {
		event.sparksRemaining = 0;
		const auto retry = std::min(hardDeadline, std::max<efitick_t>(minimumFire, nowNt + US2NT(10)));
		engine->scheduler.schedule("minimum dwell", &event.sparkEvent.scheduling, retry, firingAction(event, ctx));
		return;
	}

	if (ctx.isOverdwellProtect && anyCharged) {
		event.hardGuardCount++;
	}
	if (!anyCharged && !event.wasSparkLimited) {
		event.missedChargeCount++;
	}
	closeOccurrence(event);

#if EFI_UNIT_TEST
	if (engine->onIgnitionEvent) {
		engine->onIgnitionEvent(ctx, false);
	}
#endif

	forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) { enginePins.coils[idx].discharge(owner); });

#if EFI_TUNER_STUDIO
	// ratio of desired dwell duration to actual dwell duration gives us some idea of how good is input trigger jitter
	engine->outputChannels.dwellAccuracyRatio = actualDwellMs / event.sparkDwell;
#endif

	// now that we've just fired a coil let's prepare the new schedule for the next engine revolution

	angle_t dwellAngleDuration = engine->ignitionState.dwellAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(dwellAngleDuration) || std::isnan(sparkDwell)) {
		// we are here if engine has just stopped
		return;
	}

	// If there are more sparks to fire, schedule them
	if (event.sparksRemaining > 0 && !ctx.isOverdwellProtect && anyCharged && newIgnitionChargeAllowed()) {
		event.sparksRemaining--;
		ctx = beginOccurrence(event);
		event.occurrenceDwell = NT2USF(engine->engineState.multispark.dwell) * 0.001f;

		efitick_t nextDwellStart = nowNt + engine->engineState.multispark.delay;
		efitick_t nextFiring = nextDwellStart + engine->engineState.multispark.dwell;

		// We can schedule both of these right away, since we're going for "asap" not "particular angle"
		engine->scheduler.schedule(
				"dwell", &event.dwellStartTimer, nextDwellStart, {&turnSparkPinHigh, chargeContext(ctx, event)});
		engine->scheduler.schedule(
				"firing", &event.sparkEvent.scheduling, nextFiring, {fireSparkAndPrepareNextSchedule, ctx});
	} else {
		if (event.trailingPending && !ctx.isOverdwellProtect) {
			IgnitionContext trailingCtx = ctx;
			trailingCtx.generation = event.trailingGeneration;
			// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
			scheduleByAngle(
					&event.trailingSparkFire,
					nowNt,
					engine->engineState.trailingSparkAngle,
					{&fireTrailingSpark, trailingCtx});
		}

		if (event.trailingPending && (ctx.isOverdwellProtect || !anyCharged)) {
			fireTrailingSpark(trailingContext(event));
		}

		// If all events have been scheduled, prepare for next time.
		prepareCylinderIgnitionSchedule(dwellAngleDuration, sparkDwell, event);
	}

	engine->onSparkFireKnockSense(event.occurrenceCylinder);
}

void turnSparkPinHigh(IgnitionContext ctx) {
	chibios_rt::CriticalSectionLocker csl;
	if (ctx.eventIndex >= MAX_CYLINDER_COUNT) {
		return;
	}
	auto& event = engine->ignitionEvents.elements[ctx.eventIndex];
	if (ctx.generation != event.chargeGeneration || event.state != IgnitionOccurrenceState::ChargePending ||
		event.wasSparkLimited) {
		return;
	}
	ctx.generation = event.generation;
	if (!newIgnitionChargeAllowed()) {
		event.wasSparkLimited = true;
		closeOccurrence(event);
		return;
	}
	efitick_t nowNt = getTimeNowNt();
	const auto owner = ctx.owner();
	bool available = true;
	forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) { available &= enginePins.coils[idx].canCharge(owner); });
	if (!available) {
		event.contentionCount++;
		closeOccurrence(event);
		return;
	}

	event.state = IgnitionOccurrenceState::Charging;
	auto guardCtx = ctx;
	guardCtx.isOverdwellProtect = true;
	bool charged = true;
	forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) {
		charged &=
				enginePins.coils[idx].charge(owner, event.occurrenceDwell, {fireSparkAndPrepareNextSchedule, guardCtx});
	});
	if (!charged || !isCurrent(ctx, event)) {
		forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) { enginePins.coils[idx].discharge(owner); });
		if (isCurrent(ctx, event)) {
			closeOccurrence(event);
		}
		return;
	}

	event.actualDwellTimer.reset(nowNt);

#if EFI_UNIT_TEST
	if (engine->onIgnitionEvent) {
		engine->onIgnitionEvent(ctx, true);
	}
#endif

	if (event.trailingEnabled && !event.trailingPending) {
		event.trailingPending = true;
		event.trailingCharging = false;
		event.trailingGeneration = (event.trailingGeneration + 1) & 0x7ffffff;
		if (!event.trailingGeneration) {
			event.trailingGeneration = 1;
		}
		IgnitionContext trailingCtx = ctx;
		trailingCtx.generation = event.trailingGeneration;
		// Trailing sparks are enabled - schedule an event for the corresponding trailing coil
		scheduleByAngle(
				&event.trailingSparkCharge,
				nowNt,
				engine->engineState.trailingSparkAngle,
				{&chargeTrailingSpark, trailingCtx});
	}
}

static void scheduleSparkEvent(
		bool limitedSpark,
		IgnitionEvent& event,
		float dwellMs,
		EngPhase dwellAngle,
		EngPhase sparkAngle,
		const EnginePhaseInfo& phase,
		expected<float> chargeDelayNt = unexpected) {
	chibios_rt::CriticalSectionLocker csl;
	if (event.state != IgnitionOccurrenceState::Closed || event.trailingPending || event.sparkEvent.scheduling.action) {
		// A previous charge still owns this timer, possibly across sync loss or
		// configuration reset. Let it discharge its outputs before reusing the
		// event: replacing it could strand an old coil, while retaining it could
		// leave the new charge without a guard if the old timer fires first.
		return;
	}

	float angleOffset = dwellAngle - phase.currentEngPhase;
	if (angleOffset < 0) {
		angleOffset += engine->engineState.engineCycle;
	}

	engine->engineState.sparkCounter++;
	event.wasSparkLimited = limitedSpark;

	auto ctx = beginOccurrence(event);
	event.outputMaskSnapshot = event.calculateIgnitionOutputMask();
	event.occurrenceDwell = dwellMs;
	event.occurrenceCylinder = event.cylinderNumber;
	event.trailingEnabled = engineConfiguration->enableTrailingSparks;
	event.trailingDwell = dwellMs;
	event.trailingCylinder = event.cylinderNumber;
	event.sparksRemaining = limitedSpark ? 0 : engine->engineState.multispark.count;

	efitick_t chargeTime = phase.timestamp;

	/**
	 * The start of charge is always within the current trigger event range, so just plain time-based scheduling
	 */
	if (!limitedSpark) {
		/**
		 * Note how we do not check if spark is limited or not while scheduling 'spark down'
		 * This way we make sure that coil dwell started while spark was enabled would fire and not burn
		 * the coil.
		 */
		if (chargeDelayNt) {
			chargeTime = phase.timestamp + static_cast<uint32_t>(chargeDelayNt.Value);
			engine->scheduler.schedule(
					"time budget dwell",
					&event.dwellStartTimer,
					chargeTime,
					{turnSparkPinHigh, chargeContext(ctx, event)});
		} else {
			chargeTime = scheduleByAngleInPhase(
					&event.dwellStartTimer,
					phase,
					angleOffset,
					{&turnSparkPinHigh, chargeContext(ctx, event)},
					AngleTimingPolicy::Ignition);
		}
	}

	/**
	 * Spark event is often happening during a later trigger event timeframe
	 */

	efiAssertVoid(ObdCode::CUSTOM_ERR_6591, !std::isnan(sparkAngle.angle), "findAngle#4");
	assertAngleRange(sparkAngle.angle, "findAngle#a5", ObdCode::CUSTOM_ERR_6549);

	// Registration of the charge, spark and fallback is protected by the same lock.
	if (!isCurrent(ctx, event)) {
		return;
	}
	bool scheduled = engine->module<TriggerScheduler>()->scheduleOrQueue(
			&event.sparkEvent, sparkAngle, firingAction(event, ctx), phase, AngleTimingPolicy::Ignition);

	(void)scheduled;
	(void)chargeTime;
}

// A sync/configuration reset must never let an old pending HIGH occur later.
// Main charging occurrences retain their armed LOW. A trailing pending charge is
// dropped only before HIGH; once HIGH has happened, its matched LOW is retained.
void cancelPendingIgnition() {
	chibios_rt::CriticalSectionLocker csl;
#if EFI_UNIT_TEST
	if (!engine) {
		return;
	}
#endif // EFI_UNIT_TEST
	engine->ignitionEvents.plannerPhaseValid = false;

	for (auto& event : engine->ignitionEvents.elements) {
		event.plannedCycleValid = false;
		event.candidateValid = false;
		if (event.state == IgnitionOccurrenceState::ChargePending) {
			closeOccurrence(event);
		}
		event.sparksRemaining = 0;
		event.trailingEnabled = false;

		engine->scheduler.cancel(&event.trailingSparkCharge);
		if (event.trailingPending && !enginePins.trailingCoils[event.trailingCylinder].getLogicValue()) {
			event.trailingPending = false;
			event.trailingCharging = false;
			engine->scheduler.cancel(&event.trailingSparkFire);
		}
	}
}

void stopIgnition() {
	chibios_rt::CriticalSectionLocker csl;
#if EFI_UNIT_TEST
	if (!engine) {
		return;
	}
#endif // EFI_UNIT_TEST

	for (auto& event : engine->ignitionEvents.elements) {
		closeOccurrence(event);
		event.trailingPending = false;
		event.trailingCharging = false;
		event.trailingEnabled = false;
		engine->scheduler.cancel(&event.trailingSparkCharge);
		engine->scheduler.cancel(&event.trailingSparkFire);
	}
}

void initializeIgnitionActions() {
	IgnitionEventList& list = engine->ignitionEvents;
	angle_t dwellAngle = engine->ignitionState.dwellAngle;
	floatms_t sparkDwell = engine->ignitionState.getDwell();
	if (std::isnan(engine->cylinders[0].getIgnitionTimingBtdc()) || std::isnan(dwellAngle)) {
		// error should already be reported
		// need to invalidate previous ignition schedule
		list.isReady = false;
		return;
	}

	for (size_t cylinderIndex = 0; cylinderIndex < engine->engineState.cylinderCount; cylinderIndex++) {
		list.elements[cylinderIndex].cylinderIndex = cylinderIndex;
		prepareCylinderIgnitionSchedule(dwellAngle, sparkDwell, list.elements[cylinderIndex]);
	}
	list.isReady = true;
}

static void prepareIgnitionSchedule() {
	ScopePerf perf(PE::PrepareIgnitionSchedule);

	/**
	 * TODO: warning. there is a bit of a hack here, todo: improve.
	 * currently output signals/times dwellStartTimer from the previous revolutions could be
	 * still used because they have crossed the revolution boundary
	 * but we are already re-purposing the output signals, but everything works because we
	 * are not affecting that space in memory. todo: use two instances of 'ignitionSignals'
	 */
	operation_mode_e operationMode = getEngineRotationState()->getOperationMode();
	float maxAllowedDwellAngle = (int)(getEngineCycle(operationMode) / 2); // the cast is about making Coverity happy

	if (getCurrentIgnitionMode() == IM_ONE_COIL) {
		maxAllowedDwellAngle = getEngineCycle(operationMode) / engine->engineState.cylinderCount / 1.1;
	}

	if (engine->ignitionState.dwellAngle == 0) {
		warning(ObdCode::CUSTOM_ZERO_DWELL, "dwell is zero?");
	}
	if (engine->ignitionState.dwellAngle > maxAllowedDwellAngle) {
		warning(ObdCode::CUSTOM_DWELL_TOO_LONG, "dwell angle too long: %.2f", engine->ignitionState.dwellAngle);
	}

	// todo: add some check for dwell overflow? like 4 times 6 ms while engine cycle is less then that

	initializeIgnitionActions();
}

static bool timeBudgetSupported() {
	return engineConfiguration->ignitionTimeBudget && getCurrentIgnitionMode() == IM_INDIVIDUAL_COILS &&
		   getTriggerCentral()->triggerState.hasSynchronizedPhase() && !getTriggerCentral()->directSelfStimulation &&
		   getTriggerCentral()->instantRpm.ignitionProfile.toothCount() > 0;
}

static expected<float> ignitionEta(const EnginePhaseInfo& phase, float distance) {
	if (!std::isfinite(distance) || distance < 0 || distance > 180) {
		return unexpected;
	}
	auto eta = getTriggerCentral()->instantRpm.ignitionProfile.getTimeToAngleNt(phase, distance);
	if (eta) {
		return eta;
	}
	const float fallback = USF2NT(engine->rpmCalculator.oneDegreeUs) * distance;
	return std::isfinite(fallback) && fallback >= 0 ? expected<float>(fallback) : unexpected;
}

// This is a planning/service reserve, not a proven interrupt-latency bound.
static constexpr float ignitionServiceReserveNt = USF2NT(100);

static uint32_t plannerCycleAt(const EnginePhaseInfo& phase) {
	const auto& list = engine->ignitionEvents;
	return list.plannerCycle + (list.plannerPhaseValid && phase.currentEngPhase.angle < list.plannerLastPhase);
}

static float candidateDistance(const IgnitionEvent& event, const EnginePhaseInfo& phase) {
	const int32_t cycles = static_cast<int32_t>(event.candidateTdcCycle - plannerCycleAt(phase));
	return cycles * 720.0f + event.candidateSparkAngle - phase.currentEngPhase.angle;
}

static expected<float> ignitionAngleForTime(const EnginePhaseInfo& phase, float ticks) {
	auto angle = getTriggerCentral()->instantRpm.ignitionProfile.getAngleForTimeNt(phase, ticks);
	if (angle) {
		return angle;
	}
	const float rate = USF2NT(engine->rpmCalculator.oneDegreeUs);
	const float fallback = ticks / rate;
	return std::isfinite(fallback) && rate > 0 && fallback >= 0 && fallback <= 180 ? expected<float>(fallback)
																				   : unexpected;
}

static void observeRetarget(
		IgnitionEvent& event,
		float requested,
		float applied,
		float earliest,
		float latest,
		IgnitionRetargetStatus status) {
	event.requestedSparkAngle = requested;
	event.retargetStatus = static_cast<uint8_t>(status);
	(void)applied;
	(void)earliest;
	(void)latest;
#if EFI_UNIT_TEST
	if (engine->onIgnitionRetarget) {
		engine->onIgnitionRetarget(event.cylinderIndex, requested, applied, earliest, latest, static_cast<int>(status));
	}
#endif
}

// Update a candidate for the SAME TDC. Nothing is cancelled here: if computation
// consumed the available reserve, an armed plan remains executable unchanged.
static bool updateCandidate(IgnitionEvent& event, const EnginePhaseInfo& phase, float dwellMs) {
	const float requested =
			engine->cylinders[event.cylinderNumber].getSparkAngle(-engine->module<KnockController>()->getKnockRetard());
	const float previous = candidateDistance(event, phase);
	const float wanted = previous + requested - event.candidateSparkAngle;
	const bool changed = std::abs(requested - event.requestedSparkAngle) > .0001f;
	const bool charging = event.state == IgnitionOccurrenceState::Charging;
	const float minimum = MSF2NT(.8f * dwellMs);
	// This path corrects a NEW command. Do not impose a new registration
	// reserve or change the original admission/minimum-dwell behavior merely
	// because a tooth changed the estimated time to an unchanged target.
	if (!changed) {
		return false;
	}
	const auto now = getTimeNowNt();
	float earliestNt = static_cast<float>((now - phase.timestamp)) + ignitionServiceReserveNt;
	float latestNt = INFINITY;
	if (charging) {
		IgnitionContext owner;
		owner.eventIndex = event.cylinderIndex;
		owner.generation = event.generation;
		bool anyOwned = false;
		forEachSetBit(event.outputMaskSnapshot, [&](size_t idx) {
			const auto& pin = enginePins.coils[idx];
			if (pin.ownedBy(owner.owner())) {
				anyOwned = true;
				earliestNt = std::max(earliestNt, static_cast<float>((pin.firstHigh() - phase.timestamp)) + minimum);
				latestNt = std::min(
						latestNt,
						static_cast<float>((pin.hardDeadline() - phase.timestamp)) - ignitionServiceReserveNt);
			}
		});
		if (!anyOwned) {
			observeRetarget(
					event, requested, event.candidateSparkAngle, previous, previous, IgnitionRetargetStatus::Rejected);
			return false;
		}
	} else {
		earliestNt += minimum;
	}
	auto earliest = ignitionAngleForTime(phase, earliestNt + 1); // round toward feasibility
	float latest = 180;
	if (charging) {
		auto end = ignitionAngleForTime(phase, std::max(0.0f, latestNt - 1));
		if (end) {
			latest = end.Value;
		} else {
			auto horizon = ignitionEta(phase, 180);
			if (!horizon || horizon.Value > latestNt) {
				earliest = unexpected;
			}
		}
	}
	const float origin = event.candidateSparkAngle - previous;
	if (!earliest || earliestNt > latestNt || !std::isfinite(wanted)) {
		observeRetarget(
				event,
				requested,
				event.candidateSparkAngle,
				previous + origin,
				previous + origin,
				IgnitionRetargetStatus::Rejected);
		return false;
	}
	const float tdc = engine->cylinders[event.cylinderNumber].getAngleOffset();
	earliest.Value = std::max(earliest.Value, tdc - engineConfiguration->maximumIgnitionTiming - origin);
	latest = std::min(latest, tdc - engineConfiguration->minimumIgnitionTiming - origin);
	auto decision = selectIgnitionTarget(
			previous, wanted, earliest.Value, latest, event.state != IgnitionOccurrenceState::Closed);
	auto eta = ignitionEta(phase, decision.distance);
	// Recheck the real clock near commit, not just the ISR's captured timestamp.
	const float elapsed = static_cast<float>((getTimeNowNt() - phase.timestamp));
	if (!eta || eta.Value < elapsed + (charging ? 0 : minimum) || eta.Value > latestNt) {
		decision = {previous, IgnitionRetargetStatus::Rejected};
	}
	observeRetarget(
			event, requested, decision.distance + origin, earliest.Value + origin, latest + origin, decision.status);
	if (decision.status == IgnitionRetargetStatus::Rejected || decision.status == IgnitionRetargetStatus::Unchanged) {
		return false;
	}
	event.candidateSparkAngle = decision.distance + origin;
	return true;
}

void revisePendingIgnition(const EnginePhaseInfo& phase) {
	if (!timeBudgetSupported()) {
		return;
	}
	chibios_rt::CriticalSectionLocker csl;
	for (auto& event : engine->ignitionEvents.elements) {
		if (!event.plannedByTime || !event.candidateValid || event.state == IgnitionOccurrenceState::Closed ||
			event.wasSparkLimited) {
			continue;
		}
		if (!newIgnitionChargeAllowed()) {
			// An existing charge keeps its current LOW and independent physical guard.
			if (event.state == IgnitionOccurrenceState::ChargePending) {
				closeOccurrence(event);
			}
			continue;
		}
		const bool retargeted = updateCandidate(event, phase, event.occurrenceDwell);
		IgnitionContext ctx;
		ctx.eventIndex = event.cylinderIndex;
		ctx.generation = event.generation;
		auto eta = event.state == IgnitionOccurrenceState::ChargePending
						 ? ignitionEta(phase, candidateDistance(event, phase))
						 : expected<float>(unexpected);
		if (eta) {
			// Invalidate an extracted old HIGH before registering any replacement
			// LOW: registration itself may execute another due action inline.
			nextChargeGeneration(event);
			engine->scheduler.cancel(&event.dwellStartTimer);
		}
		if (retargeted) {
			float target = event.candidateSparkAngle;
			wrapAngle(target, "retarget spark", ObdCode::CUSTOM_ERR_6550);
			event.plannedSparkAngle = target;
			nextFireGeneration(event);
			engine->module<TriggerScheduler>()->cancel(&event.sparkEvent);
			engine->scheduler.cancel(&event.sparkEvent.scheduling);
			engine->module<TriggerScheduler>()->scheduleOrQueue(
					&event.sparkEvent, {target}, firingAction(event, ctx), phase, AngleTimingPolicy::Ignition);
		}
		if (!eta || !isCurrent(ctx, event) || event.state != IgnitionOccurrenceState::ChargePending) {
			continue;
		}
		const float delay = std::max(0.0f, eta.Value - MSF2NT(event.occurrenceDwell));
		engine->scheduler.schedule(
				"revised dwell",
				&event.dwellStartTimer,
				phase.timestamp + static_cast<uint32_t>(delay),
				{turnSparkPinHigh, chargeContext(ctx, event)});
	}
}

void onTriggerEventSparkLogic(const EnginePhaseInfo& phase) {
	ScopePerf perf(PE::OnTriggerEventSparkLogic);

	if (!engineConfiguration->isIgnitionEnabled) {
		return;
	}

	bool limitedSpark = !getLimpManager()->allowIgnition().value;

	const floatms_t dwellMs = engine->ignitionState.getDwell();
	if (!IgnitionOutputPin::validDwell(dwellMs)) {
		warning(ObdCode::CUSTOM_DWELL, "invalid dwell to handle: %.2f", dwellMs);
		return;
	}

	if (!engine->ignitionEvents.isReady) {
		prepareIgnitionSchedule();
	}

	/**
	 * Ignition schedule is defined once per revolution
	 * See initializeIgnitionActions()
	 */

	// Only apply odd cylinder count wasted logic if:
	// - odd cyl count
	// - current mode is wasted spark
	// - four stroke
	bool enableOddCylinderWastedSpark =
			engine->engineState.useOddFireWastedSpark && getCurrentIgnitionMode() == IM_WASTED_SPARK;

	const bool useBudget = timeBudgetSupported();
	auto& list = engine->ignitionEvents;

#if EFI_ANTILAG_SYSTEM && EFI_LAUNCH_CONTROL
	const float ALSSkipRatio = engineConfiguration->antiLagEnabled && engine->antilagController.isAntilagCondition
									 ? static_cast<float>(engineConfiguration->ALSSkipRatio)
									 : 0.0f;
	engine->ALSsoftSparkLimiter.setTargetSkipRatio(ALSSkipRatio);
#endif // EFI_ANTILAG_SYSTEM

	if (useBudget) {
		if (list.plannerPhaseValid && phase.currentEngPhase.angle < list.plannerLastPhase) {
			list.plannerCycle++;
		}
		list.plannerLastPhase = phase.currentEngPhase.angle;
		list.plannerPhaseValid = true;
	}

	if (engine->ignitionEvents.isReady) {
		for (size_t i = 0; i < engine->engineState.cylinderCount; i++) {
			auto& event = engine->ignitionEvents.elements[i];

			if (useBudget) {
				chibios_rt::CriticalSectionLocker csl;
				if (event.state != IgnitionOccurrenceState::Closed || event.trailingPending) {
					continue;
				}
				if (!event.candidateValid) {
					// Seed a future target only. A later command cannot wrap this
					// retained candidate into a different TDC occurrence.
					const float rawTarget = engine->cylinders[event.cylinderNumber].getSparkAngle(
							-engine->module<KnockController>()->getKnockRetard());
					if (!std::isfinite(rawTarget)) {
						continue;
					}
					float target = rawTarget;
					wrapAngle(target, "budget target", ObdCode::CUSTOM_ERR_6550);
					float distance = target - phase.currentEngPhase.angle;
					uint32_t occurrence = list.plannerCycle;
					if (distance < 0) {
						distance += 720;
						occurrence++;
					}
					if (rawTarget < 0) {
						occurrence++;
					}
					if (rawTarget >= 720) {
						occurrence--;
					}
					if (distance > 540) {
						const uint32_t expired = occurrence - 1;
						if (!event.plannedCycleValid || event.plannedTdcCycle != expired) {
							event.expiredTargetCount++;
							event.plannedTdcCycle = expired;
							event.plannedCycleValid = true;
						}
						continue;
					}
					if (distance > 180 || (event.plannedCycleValid && event.plannedTdcCycle == occurrence)) {
						continue;
					}
					event.candidateTdcCycle = occurrence;
					event.candidateSparkAngle = rawTarget;
					event.requestedSparkAngle = rawTarget;
					event.retargetStatus = static_cast<uint8_t>(IgnitionRetargetStatus::Unchanged);
					event.candidateValid = true;
				}
				if (candidateDistance(event, phase) < -180) {
					event.expiredTargetCount++;
					event.plannedTdcCycle = event.candidateTdcCycle;
					event.plannedCycleValid = true;
					event.candidateValid = false;
					continue;
				}
				const bool corrected = updateCandidate(event, phase, dwellMs);
				auto eta = ignitionEta(phase, candidateDistance(event, phase));
				float span = phase.nextEngPhase - phase.currentEngPhase;
				if (span < 0) {
					span += 720;
				}
				auto interval = ignitionEta(phase, span);
				if (!eta || !interval) {
					if (candidateDistance(event, phase) < 0) {
						event.expiredTargetCount++;
						event.plannedTdcCycle = event.candidateTdcCycle;
						event.plannedCycleValid = true;
						event.candidateValid = false;
					}
					continue;
				}
				const float delay = std::max(0.0f, eta.Value - MSF2NT(dwellMs));
				if (delay >= interval.Value) {
					continue;
				}
				const float tdc = engine->cylinders[event.cylinderNumber].getAngleOffset();
				const float elapsed = static_cast<float>(getTimeNowNt() - phase.timestamp);
				if ((corrected && eta.Value < elapsed + MSF2NT(.8f * dwellMs)) ||
					event.candidateSparkAngle < tdc - engineConfiguration->maximumIgnitionTiming ||
					event.candidateSparkAngle > tdc - engineConfiguration->minimumIgnitionTiming) {
					// There is no safe candidate inside the configured timing range.
					// Consume this TDC without energizing; a later command cannot revive it.
					event.expiredTargetCount++;
					event.plannedTdcCycle = event.candidateTdcCycle;
					event.plannedCycleValid = true;
					event.candidateValid = false;
					continue;
				}
				event.plannedTdcCycle = event.candidateTdcCycle;
				event.plannedCycleValid = true;
				event.plannedByTime = true;
				float target = event.candidateSparkAngle;
				wrapAngle(target, "candidate spark", ObdCode::CUSTOM_ERR_6550);
				event.plannedSparkAngle = target;
				bool skip = limitedSpark;
#if EFI_LAUNCH_CONTROL
				skip |= engine->softSparkLimiter.shouldSkip() || engine->torqueReductionSparkLimiter.shouldSkip();
#endif
#if EFI_ANTILAG_SYSTEM && EFI_LAUNCH_CONTROL
				skip |= engine->ALSsoftSparkLimiter.shouldSkip();
#endif
				scheduleSparkEvent(skip, event, dwellMs, phase.currentEngPhase, {target}, phase, delay);
				continue;
			}
			event.plannedByTime = false;
			if (event.state == IgnitionOccurrenceState::Closed) {
				event.candidateValid = false;
			}
			angle_t dwellAngle = event.dwellAngle;

			angle_t sparkAngleAdjust = 0;

			bool isOddCylWastedEvent = false;
			if (enableOddCylinderWastedSpark) {
				auto dwellAngleWastedEvent = dwellAngle + 360;
				if (dwellAngleWastedEvent > 720) {
					dwellAngleWastedEvent -= 720;
				}

				// Check whether this event hits 360 degrees out from now (ie, wasted spark),
				// and if so, twiddle the dwell and spark angles so it happens now instead
				isOddCylWastedEvent = isPhaseInRange(EngPhase{dwellAngleWastedEvent}, phase);

				if (isOddCylWastedEvent) {
					dwellAngle = dwellAngleWastedEvent;

					sparkAngleAdjust = 360;
				}
			}

			if (!isOddCylWastedEvent && !isPhaseInRange(EngPhase{dwellAngle}, phase)) {
				continue;
			}

			angle_t sparkAngle = sparkAngleAdjust + event.calculateSparkAngle();
			if (sparkAngle > 720) {
				sparkAngle -= 720;
			}
			if (std::isnan(sparkAngle)) {
				warning(ObdCode::CUSTOM_ADVANCE_SPARK, "NaN advance");
				continue;
			}

#if EFI_LAUNCH_CONTROL
			if (engine->softSparkLimiter.shouldSkip()) {
				continue;
			}

			if (engine->torqueReductionSparkLimiter.shouldSkip()) {
				continue;
			}
#endif // EFI_LAUNCH_CONTROL

#if EFI_ANTILAG_SYSTEM && EFI_LAUNCH_CONTROL
			if (engine->ALSsoftSparkLimiter.shouldSkip()) {
				continue;
			}
#endif // EFI_ANTILAG_SYSTEM

			scheduleSparkEvent(limitedSpark, event, dwellMs, {dwellAngle}, {sparkAngle}, phase);
		}
	}
}

/**
 * Number of sparks per physical coil
 * @see getNumberOfInjections
 */
int getNumberOfSparks(ignition_mode_e mode) {
	switch (mode) {
		case IM_ONE_COIL:
			return engine->engineState.cylinderCount;
		case IM_TWO_COILS:
			return engine->engineState.cylinderCount / 2;
		case IM_INDIVIDUAL_COILS:
			return 1;
		case IM_WASTED_SPARK:
			return 2;
		default:
			firmwareError(ObdCode::CUSTOM_ERR_IGNITION_MODE, "Unexpected ignition_mode_e %d", mode);
			return 1;
	}
}

/**
 * @see getInjectorDutyCycle
 */
percent_t getCoilDutyCycle(float rpm) {
	floatms_t totalPerCycle = engine->ignitionState.getDwell() * getNumberOfSparks(getCurrentIgnitionMode());
	floatms_t engineCycleDuration =
			getCrankshaftRevolutionTimeMs(rpm) * (getEngineRotationState()->getOperationMode() == TWO_STROKE ? 1 : 2);
	return 100 * totalPerCycle / engineCycleDuration;
}

#endif // EFI_ENGINE_CONTROL
