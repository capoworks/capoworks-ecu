/**
 * @file configuration_write_state.h
 * @author Giacomo di Ciocco, (C) 2026
 */

#pragma once

// Callers serialize state transitions. A failed attempt remains pending, but
// only a new request permits another automatic write.
class ConfigurationWriteState {
public:
	void request() {
		m_pending = true;
		m_requested = true;
	}

	bool pending() const {
		return m_pending;
	}

	bool shouldWrite() const {
		return m_requested && !m_writing && !m_reading;
	}

	bool begin(bool requestedOnly = false) {
		if (m_writing || m_reading || (requestedOnly && !m_requested)) {
			return false;
		}

		m_pending = true;
		m_requested = false;
		m_writing = true;
		return true;
	}

	void complete(bool success) {
		m_writing = false;
		m_pending = !success || m_requested;
	}

	bool beginRead() {
		if (m_writing || m_reading) {
			return false;
		}

		m_reading = true;
		return true;
	}

	void endRead() {
		m_reading = false;
	}

	bool writing() const {
		return m_writing;
	}

	bool reading() const {
		return m_reading;
	}

private:
	bool m_pending = false;
	bool m_requested = false;
	bool m_writing = false;
	bool m_reading = false;
};
