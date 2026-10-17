/*
 * @file spark_logic.h
 *
 * @date Sep 15, 2016
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

void onTriggerEventSparkLogic(const EnginePhaseInfo& phase);
int getNumberOfSparks(ignition_mode_e mode);
percent_t getCoilDutyCycle(float rpm);
void initializeIgnitionActions();

// Passed by value through the one-word timer action. Mutable scheduling fields
// live in IgnitionEvent; an extracted old callback cannot acquire a new occurrence.
union IgnitionContext {
	constexpr IgnitionContext()
		: _pad(nullptr) {
		eventIndex = 0xF;
	}
	struct {
		uint32_t eventIndex : 4;
		uint32_t generation : 27;
		uint32_t isOverdwellProtect : 1;
	};
	void* _pad;
	uint16_t outputsMask() const;
	uint32_t owner() const {
		return ((generation << 4) | eventIndex) + 1;
	}
};

static_assert(sizeof(IgnitionContext) <= sizeof(void*));

void turnSparkPinHigh(IgnitionContext ctx);
void fireSparkAndPrepareNextSchedule(IgnitionContext ctx);

// Cancel main charges that have not reached HIGH yet. This is a main-loop
// lifecycle operation; already charging outputs retain their scheduled LOW.
void cancelPendingIgnition();
void stopIgnition();

void revisePendingIgnition(const EnginePhaseInfo& phase);
