/*
 * @file prime_injection.cpp
 */

#include "pch.h"
#include "prime_injection.h"
#include "injection_gpio.h"
#include "sensor.h"
#include "backup_ram.h"

floatms_t PrimeController::getPrimeDuration() const {
	auto clt = Sensor::get(SensorType::Clt);

	// If the coolant sensor is dead, skip the prime. The engine will still start fine, but may take a little longer.
	if (!clt) {
		return 0;
	}

	auto primeMass = 0.001f * // convert milligram to gram
					 interpolate2d(clt.Value, engineConfiguration->primeBins, engineConfiguration->primeValues);

	efiPrintf("Priming pulse mass: %.4f g", primeMass);

	return engine->module<InjectorModelPrimary>()->getInjectionDuration(primeMass);
}

// Check if the engine is not stopped or cylinder cleanup is activated
static bool isPrimeInjectionPulseSkipped() {
	// Skip if the engine is already spinning
	if (!getEngineRotationState()->isStopped()) {
		return true;
	}

	// Skip if cylinder cleanup is active
	return engineConfiguration->isCylinderCleanupEnabled &&
		   (Sensor::getOrZero(SensorType::DriverThrottleIntent) > CLEANUP_MODE_TPS);
}

void PrimeController::onIgnitionStateChanged(bool ignitionOn) {
	uintptr_t generation;

	{
		chibios_rt::CriticalSectionLocker csl;

		if (ignitionOn == m_isIgnitionOn) {
			return;
		}

		m_isIgnitionOn = ignitionOn;
		++m_keyCycleGeneration;
		if (!m_keyCycleGeneration) {
			++m_keyCycleGeneration;
		}
		generation = m_keyCycleGeneration;
	}

	if (!ignitionOn) {
		// Delayed starts and opens carry this generation, so they become stale
		// immediately. An already-opened pulse still gets its matching close.
		return;
	}

	// First, we need a protection against 'fake' ignition switch on and off (i.e. no engine started), to avoid repeated
	// prime pulses. So we check and update the ignition switch counter in non-volatile backup-RAM
	uint32_t ignSwitchCounter = getKeyCycleCounter();

	// if we're just toying with the ignition switch, give it another chance eventually...
	if (ignSwitchCounter > 10) {
		ignSwitchCounter = 0;
	}

	// If we're going to skip this pulse, then save the counter as 0.
	// That's because we'll definitely need the prime pulse next time (either due to the cylinder cleanup or the engine
	// spinning)
	if (isPrimeInjectionPulseSkipped()) {
		ignSwitchCounter = -1;
	}

	// start prime injection if this is a 'fresh start'
	if (ignSwitchCounter == 0) {
		// Give sensors long enough to wake up before priming
		constexpr float minimumPrimeDelayMs = 100;
		float delayMs = engineConfiguration->primingDelay * 1000 + minimumPrimeDelayMs;
		float delayUs = MS2US(delayMs);
		if (std::isfinite(delayUs) && delayUs >= 0 && delayUs < MaximumScheduleDelayUs) {
			auto startTime = getTimeNowNt() + US2NT(static_cast<int>(delayUs));
			ScheduledAction request{startTime, {PrimeController::onPrimeStartAdapter, generation}};
			scheduleFuelCallbacks(&request, 1, true);
		}
	} else {
		efiPrintf("Skipped priming pulse since ignSwitchCounter = %lu", ignSwitchCounter);
	}

	// we'll reset it later when the engine starts
	setKeyCycleCounter(ignSwitchCounter + 1);
}

#if EFI_PROD_CODE
uint32_t PrimeController::getKeyCycleCounter() const {
	return getBackupSram()->IgnCounter;
}

void PrimeController::setKeyCycleCounter(uint32_t count) {
	getBackupSram()->IgnCounter = count;
}
#else // not EFI_PROD_CODE
uint32_t PrimeController::getKeyCycleCounter() const {
	return 0;
}

void PrimeController::setKeyCycleCounter(uint32_t) {}
#endif

bool PrimeController::isPriming() const {
	chibios_rt::CriticalSectionLocker csl;
	return m_isPriming;
}

void PrimeController::onPrimeStartAdapter(uintptr_t generation) {
	engine->module<PrimeController>()->onPrimeStart(generation);
	// A delayed prime still completes its accepted callback when skipped.
	engine->airmassInjectionState.callbackCompleted();
}

void PrimeController::onPrimeOpenAdapter(uintptr_t generation) {
	engine->module<PrimeController>()->onPrimeOpen(generation);
	engine->airmassInjectionState.callbackCompleted();
}

void PrimeController::onPrimeEndAdapter(uintptr_t generation) {
	engine->module<PrimeController>()->onPrimeEnd(generation);
	engine->airmassInjectionState.callbackCompleted();
}

void PrimeController::onPrimeStart(uintptr_t generation) {
	{
		chibios_rt::CriticalSectionLocker csl;
		if (!m_isIgnitionOn || generation != m_keyCycleGeneration || m_isPriming) {
			return;
		}
	}

	if (!engine->airmassInjectionState.allowPrime() || isPrimeInjectionPulseSkipped()) {
		return;
	}

	auto durationMs = getPrimeDuration();

	// Don't prime a zero-duration pulse
	if (!std::isfinite(durationMs) || durationMs < 0.050f || durationMs >= MaximumScheduleDelayUs / 1000) {
		efiPrintf("Skipped zero-duration priming pulse.");
		return;
	}

	efiPrintf("Firing priming pulse of %.2f ms", durationMs);

	if (engine->engineState.cylinderCount == 0 || engine->engineState.cylinderCount > MAX_CYLINDER_COUNT) {
		return;
	}

	uint16_t outputsMask = (1 << engine->engineState.cylinderCount) - 1;

	{
		chibios_rt::CriticalSectionLocker csl;

		// Recheck after the slower sensor, fuel, and configuration work above.
		if (!m_isIgnitionOn || generation != m_keyCycleGeneration || m_isPriming) {
			return;
		}

		// Capture the mask with the paired callbacks so a later tune change cannot
		// leave injectors opened by a wider mask.
		m_primeOutputsMask = outputsMask;
		m_pulseGeneration = generation;
		m_primeOpened = false;
		m_isPriming = true;
	}

	auto startTime = getTimeNowNt();
	auto endTime = startTime + US2NT(static_cast<int>(MS2US(durationMs)));

	ScheduledAction events[] = {
			{startTime, {onPrimeOpenAdapter, generation}},
			{endTime, {onPrimeEndAdapter, generation}},
	};

	if (!scheduleFuelCallbacks(events, efi::size(events), true)) {
		chibios_rt::CriticalSectionLocker csl;
		if (m_isPriming && m_pulseGeneration == generation) {
			m_isPriming = false;
			m_pulseGeneration = 0;
		}
	}
}

void PrimeController::onPrimeOpen(uintptr_t generation) {
	chibios_rt::CriticalSectionLocker csl;

	if (!m_isPriming || m_pulseGeneration != generation || !m_isIgnitionOn ||
		generation != m_keyCycleGeneration || m_primeOpened) {
		return;
	}

	InjectorContext ctx;
	ctx.outputsMask = m_primeOutputsMask;
	m_primeOpened = true;
	startInjection(ctx);
}

void PrimeController::onPrimeEnd(uintptr_t generation) {
	bool wasOpened;
	InjectorContext ctx;

	{
		chibios_rt::CriticalSectionLocker csl;

		if (!m_isPriming || m_pulseGeneration != generation) {
			return;
		}

		wasOpened = m_primeOpened;
		ctx.outputsMask = m_primeOutputsMask;
		m_primeOpened = false;

		// Keep m_isPriming set through an opened pulse's close so a replacement
		// pulse cannot start while the matching output is being released.
		if (!wasOpened) {
			m_isPriming = false;
			m_pulseGeneration = 0;
		}
	}

	if (!wasOpened) {
		return;
	}

	endInjection(ctx);

	{
		chibios_rt::CriticalSectionLocker csl;
		if (m_isPriming && m_pulseGeneration == generation) {
			m_isPriming = false;
			m_pulseGeneration = 0;
		}
	}
}

void PrimeController::onSlowCallback() {
	if (!getEngineRotationState()->isStopped()) {
#if EFI_PROD_CODE
		getBackupSram()->IgnCounter = 0;
#endif /* EFI_PROD_CODE */
	}
}
