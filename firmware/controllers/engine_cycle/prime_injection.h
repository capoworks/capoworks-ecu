/*
 * @file prime_injection.h

 */

#pragma once

#include "engine_module.h"
#include "rusefi_types.h"
#include "scheduler.h"

class PrimeController : public EngineModule {
public:
	void onIgnitionStateChanged(bool ignitionOn) override;
	void onSlowCallback() override;

	floatms_t getPrimeDuration() const;

	bool isPriming() const;

private:
	bool m_isPriming = false;
	bool m_isIgnitionOn = false;
	bool m_primeOpened = false;
	uintptr_t m_keyCycleGeneration = 0;
	uintptr_t m_pulseGeneration = 0;
	uint16_t m_primeOutputsMask = 0;
	void onPrimeStart(uintptr_t generation);
	void onPrimeOpen(uintptr_t generation);
	void onPrimeEnd(uintptr_t generation);

	static void onPrimeStartAdapter(uintptr_t generation);
	static void onPrimeOpenAdapter(uintptr_t generation);
	static void onPrimeEndAdapter(uintptr_t generation);

	uint32_t getKeyCycleCounter() const;
	void setKeyCycleCounter(uint32_t count);
};
