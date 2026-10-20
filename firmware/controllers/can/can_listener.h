/**
 * @file	can_listener.h
 *
 * @date March 31, 2021
 * @author Matthew Kennedy, (c) 2021
 */

#pragma once

#include "can.h"

class CanListener {
public:
	CanListener(uint32_t id)
		: m_id(id) {}

	CanListener* processFrame(CanBusIndex busIndex, const CANRxFrame& frame, efitick_t nowNt) {
		if (acceptFrame(busIndex, frame)) {
#ifdef MODULE_EMP_PUMP
			decodeFrame(busIndex, frame, nowNt);
#else
			decodeFrame(frame, nowNt);
#endif
		}

		return m_next;
	}

	uint32_t getId() {
		return m_id;
	}

	void setNext(CanListener* next) {
		m_next = next;
	}

	bool hasNext() const {
		return m_next;
	}

	// Return true if the provided frame should be accepted for processing by the listener.
	// Override if you need more complex logic than comparing to a single ID.
	virtual bool acceptFrame(CanBusIndex /*busIndex*/, const CANRxFrame& frame) const {
		return CAN_ID(frame) == m_id;
	}

protected:
#ifdef MODULE_EMP_PUMP
	// Bus-aware override for listeners whose protocol endpoint can move between
	// CAN controllers. The bridge is compiled only when a bus-aware module needs it.
	virtual void decodeFrame(CanBusIndex, const CANRxFrame& frame, efitick_t nowNt) {
		decodeFrame(frame, nowNt);
	}
#endif
	virtual void decodeFrame(const CANRxFrame& frame, efitick_t nowNt) = 0;

private:
	CanListener* m_next = nullptr;

	const uint32_t m_id;
};
