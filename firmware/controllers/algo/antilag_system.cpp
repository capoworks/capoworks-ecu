/*
 * @file antilag_system.cpp
 *
 *  @date 26. nov. 2022
 *      Author: Turbo Marian
 */

#include "pch.h"

#if EFI_ANTILAG_SYSTEM
#include "antilag_system.h"
#include "engine_state.h"
#include "fuel_math.h"

bool AntilagSystemBase::isInsideALSSwitchCondition() {
	isALSSwitchActivated = engineConfiguration->antiLagActivationMode == SWITCH_INPUT_ANTILAG;

	if (isALSSwitchActivated) {
		if (isBrainPinValid(engineConfiguration->ALSActivatePin)) {
#if EFI_PROD_CODE
			ALSActivatePinState =
					engineConfiguration->ALSActivateInverted ^ efiReadPin(engineConfiguration->ALSActivatePin);
#else
			ALSActivatePinState = false;
#endif
		}
		return ALSActivatePinState;
	} else {
		// ALWAYS_ACTIVE_ANTILAG
		return true;
	}
}

bool AntilagSystemBase::isALSMinRPMCondition(float rpm) const {
	return engineConfiguration->ALSMinRPM < rpm;
}

bool AntilagSystemBase::isALSMaxRPMCondition(float rpm) const {
	return engineConfiguration->ALSMaxRPM > rpm;
}

bool AntilagSystemBase::isALSMinCLTCondition() const {
	int clt = Sensor::getOrZero(SensorType::Clt);

	return engineConfiguration->ALSMinCLT < clt;
}

bool AntilagSystemBase::isALSMaxCLTCondition() const {
	int clt = Sensor::getOrZero(SensorType::Clt);

	return engineConfiguration->ALSMaxCLT > clt;
}

bool AntilagSystemBase::isALSMaxThrottleIntentCondition() const {
	auto throttleIntent = Sensor::get(SensorType::DriverThrottleIntent);
	if (!throttleIntent || !std::isfinite(throttleIntent.Value)) {
		return false;
	}

	return static_cast<float>(engineConfiguration->ALSMaxTPS) > throttleIntent.Value;
}

bool AntilagSystemBase::isAntilagConditionMet(float rpm) {
	ALSMinRPMCondition = isALSMinRPMCondition(rpm);
	ALSMaxRPMCondition = isALSMaxRPMCondition(rpm);
	ALSMinCLTCondition = isALSMinCLTCondition();
	ALSMaxCLTCondition = isALSMaxCLTCondition();
	ALSMaxThrottleIntentCondition = isALSMaxThrottleIntentCondition();
	ALSSwitchCondition = isInsideALSSwitchCondition();

	return ALSMinRPMCondition && ALSMaxRPMCondition && ALSMinCLTCondition && ALSMaxCLTCondition &&
		   ALSMaxThrottleIntentCondition && ALSSwitchCondition;
}

void AntilagSystemBase::update() {
	float rpm = Sensor::getOrZero(SensorType::Rpm);
	bool durationValid = engineConfiguration->ALSMaxDuration > 0;
	bool eligible = engineConfiguration->antiLagEnabled && durationValid && isAntilagConditionMet(rpm);

	// A timeout does not clear eligibility, so only a genuine false-to-true transition re-arms.
	if (eligible && !m_wasEligible) {
		m_maxDurationTimer.reset();
	}
	m_wasEligible = eligible;

	ALSTimerCondition = eligible && m_maxDurationTimer.getElapsedSeconds() < engineConfiguration->ALSMaxDuration;
	isAntilagCondition = eligible && ALSTimerCondition;

#if EFI_ANTILAG_SYSTEM
	fuelALSCorrection = getFuelALSCorrection(rpm);
#endif // EFI_ANTILAG_SYSTEM
}

#endif /* EFI_ANTILAG_SYSTEM */
