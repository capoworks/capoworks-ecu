/**
 * @file	scheduler.h
 *
 * @date October 1, 2020
 */
#include "pch.h"

#include "scheduler.h"

void action_s::execute() {
	efiAssertVoid(ObdCode::CUSTOM_ERR_ASSERT, m_callback != NULL, "callback==null1");
	m_callback(m_param);
}

schfunc_t action_s::getCallback() const {
	return m_callback;
}

void* action_s::getArgument() const {
	return m_param;
}

bool isScheduleBatchValid(const ScheduledAction* events, size_t count, efitick_t now) {
	if (!events || count == 0 || count > MaxScheduleBatchSize) {
		return false;
	}

	const uint64_t maximumForward = static_cast<uint64_t>(US2NT(MaximumScheduleDelayUs).count());
	for (size_t i = 0; i < count; i++) {
		if (!events[i].action || (i > 0 && events[i].time < events[i - 1].time)) {
			return false;
		}

		if (events[i].time >= now) {
			// Unsigned distance is exact across the full signed timestamp range.
			uint64_t forward =
					static_cast<uint64_t>(events[i].time.count) - static_cast<uint64_t>(now.count);
			if (forward >= maximumForward) {
				return false;
			}
		}
	}

	return true;
}
