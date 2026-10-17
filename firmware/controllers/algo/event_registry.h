/**
 * @file	event_registry.h
 *
 * @date Nov 27, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "global.h"
#include "efi_gpio.h"
#include "scheduler.h"
#include "trigger_structure.h"

enum class TriggerQueueMembership : uint8_t {
	None,
	Waiting,
	Due
};

struct AngleBasedEvent {
	scheduling_s scheduling;
	action_s action;
	/**
	 * Trigger-based scheduler maintains a linked list of all pending tooth-based events.
	 */
	AngleBasedEvent* next = nullptr;
	// An armed fallback can outlive the queued event. Reusing the event breaks that association.
	bool fallbackIsCurrent = false;
	TriggerQueueMembership queueMembership = TriggerQueueMembership::None;

	TrgPhase eventPhase;
	AngleTimingPolicy timingPolicy = AngleTimingPolicy::Legacy;

	void setAngle(EngPhase angle);

	bool shouldSchedule(const EnginePhaseInfo& phase) const;
	float getAngleFromNow(const EnginePhaseInfo& phase) const;
};

#define MAX_OUTPUTS_FOR_IGNITION 2

enum class IgnitionOccurrenceState : uint8_t {
	Closed,
	ChargePending,
	Charging
};

class IgnitionEvent {
public:
	uint16_t calculateIgnitionOutputMask() const;

	angle_t calculateSparkAngle() const;

	// Generation wraps only after 2^27 occurrences. Timer callbacks execute under
	// the scheduler lock and cannot remain extracted across even one such wrap.
	// No pointer to a mutable context is ever handed to the executor.
	uint32_t generation = 0;
	uint32_t chargeGeneration = 0;
	uint16_t outputMaskSnapshot = 0;
	uint8_t sparksRemaining = 0;
	IgnitionOccurrenceState state = IgnitionOccurrenceState::Closed;
	float occurrenceDwell = 0;
	uint32_t trailingGeneration = 0;
	float trailingDwell = 0;
	int8_t trailingCylinder = 0;
	int8_t occurrenceCylinder = 0;
	bool trailingPending = false;
	// Distinguishes a trailing HIGH that has not happened yet from one whose LOW is owed.
	bool trailingCharging = false;
	bool trailingEnabled = false;
	uint32_t hardGuardCount = 0;
	uint32_t missedChargeCount = 0;
	uint32_t contentionCount = 0;

	scheduling_s dwellStartTimer;
	AngleBasedEvent sparkEvent;

	scheduling_s trailingSparkCharge;
	scheduling_s trailingSparkFire;

	// Track whether coil charge was intentionally skipped (spark limiter)
	bool wasSparkLimited = false;

	floatms_t sparkDwell = 0;

	// this timer allows us to measure actual dwell time
	Timer actualDwellTimer;

	float dwellAngle = 0;

	/**
	 * [0, cylindersCount)
	 */
	int cylinderIndex = 0;
	int8_t cylinderNumber = 0;
	char* name = nullptr;

	ignition_mode_e m_ignitionMode = IM_INDIVIDUAL_COILS;
};

class IgnitionEventList {
public:
	/**
	 * ignition events, per cylinder
	 */
	IgnitionEvent elements[MAX_CYLINDER_COUNT];
	bool isReady = false;
};

IgnitionEventList* getIgnitionEvents();
