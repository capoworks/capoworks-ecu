/**
 * @file shock_preload.h
 * CAN bridge for the Caponord electronic shock preload controller.
 */

#pragma once

#include "can_listener.h"
#include "engine_module.h"
#include "efi_timer.h"

class ShockPreload final : public EngineModule, public CanListener {
public:
	ShockPreload();
	static constexpr CanBusIndex Bus = CanBusIndex::Bus1;

	void initNoConfiguration() override;
	void pollStatus();

	bool acceptFrame(CanBusIndex busIndex, const CANRxFrame& frame) const override;
	void decodeFrame(const CANRxFrame& frame, efitick_t nowNt) override;

	void handleTsCommand(uint16_t command);

	uint8_t getPosition() const {
		return m_position;
	}
	uint8_t getControllerTarget() const {
		return m_controllerTarget;
	}
	uint8_t getState() const {
		return m_state;
	}
	uint16_t getAlarms() const {
		return m_alarms;
	}
	uint16_t getCurrent() const {
		return m_current;
	}
	uint8_t getSelectedPreset() const {
		return m_selectedPreset;
	}
	bool isAtSavedPreset() const;
	bool isPresetActive(uint8_t preset) const;
	bool isOnline() const;

private:
	static constexpr uint32_t CommandId = 0x720;
	static constexpr uint32_t StatusId = 0x721;
	static constexpr uint32_t AlarmId = 0x722;
	static constexpr uint8_t InvalidPreset = 0xff;

	static constexpr uint8_t CommandSetTarget = 0x01;
	static constexpr uint8_t CommandLoadPreset = 0x02;
	static constexpr uint8_t CommandSavePreset = 0x03;
	static constexpr uint8_t CommandStop = 0x04;
	static constexpr uint8_t CommandClearAlarms = 0x05;
	static constexpr uint8_t CommandMoveRelative = 0x06;
	static constexpr uint8_t CommandRequestStatus = 0x10;
	static constexpr uint8_t CommandSetReference = 0x11;
	static constexpr uint8_t CommandCalibrate = 0x12;

	bool isEnabled() const;
	bool isPositionAtPreset(size_t preset) const;
	void sendSimpleCommand(uint8_t command);
	void sendValueCommand(uint8_t command, uint16_t value);
	void sendRelativeCommand(int16_t value);
	void sendPresetCommand(uint8_t command, uint8_t slot);

	uint8_t m_position = 0;
	uint8_t m_controllerTarget = 0;
	uint8_t m_state = 0;
	uint16_t m_alarms = 0;
	uint16_t m_current = 0;
	uint8_t m_selectedPreset = InvalidPreset;
	uint8_t m_presetValues[5] = {};
	uint8_t m_seenPresets = 0;
	Timer m_pollTimer;
	Timer m_lastRxTimer;
};
