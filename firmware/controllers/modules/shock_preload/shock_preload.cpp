/**
 * @file shock_preload.cpp
 * @author Giacomo di Ciocco, (C) 2026
 */

#include "pch.h"

#include "shock_preload.h"

#include "can_msg_tx.h"

namespace {
// Commands use a fixed-size payload; unused bytes remain zero-initialized.
constexpr uint8_t CommandFrameDlc = 8;
constexpr uint32_t StatusPollIntervalMs = 1000;
constexpr float OnlineTimeoutSeconds = 3.0f;

constexpr uint8_t readU8Percent(uint8_t value) {
	return std::min<uint8_t>(value, 100);
}

uint8_t readU16Percent(const CANRxFrame& frame, size_t offset) {
	uint16_t value = frame.data8[offset] | (static_cast<uint16_t>(frame.data8[offset + 1]) << 8);
	return std::min<uint16_t>(value, 100);
}

uint16_t readU16(const CANRxFrame& frame, size_t offset) {
	return frame.data8[offset] | (static_cast<uint16_t>(frame.data8[offset + 1]) << 8);
}

bool isExtendedFrame(const CANRxFrame& frame) {
#ifdef STM32H7XX
	return frame.common.XTD;
#else
	return frame.IDE;
#endif
}
} // namespace

ShockPreload::ShockPreload()
	: CanListener(StatusId) {}

void ShockPreload::initNoConfiguration() {
#if EFI_CAN_SUPPORT
	registerCanListener(*this);
#endif
}

bool ShockPreload::isEnabled() const {
	return engineConfiguration->enableShockPreload && engineConfiguration->canReadEnabled &&
		   engineConfiguration->canWriteEnabled;
}

bool ShockPreload::acceptFrame(CanBusIndex busIndex, const CANRxFrame& frame) const {
	if (!isEnabled() || busIndex != Bus || isExtendedFrame(frame) || frame.DLC < 8) {
		return false;
	}

	auto id = CAN_ID(frame);
	return id == StatusId || id == AlarmId;
}

void ShockPreload::decodeFrame(const CANRxFrame& frame, efitick_t nowNt) {
	if (CAN_ID(frame) == StatusId) {
		if (frame.data8[0] == 0xF0) {
			// Preset report: preset slot/value, active slot, position, target, alarms.
			const uint8_t presetSlot = frame.data8[1];
			if (presetSlot < efi::size(m_presetValues)) {
				m_presetValues[presetSlot] = readU8Percent(frame.data8[2]);
				m_seenPresets |= 1 << presetSlot;
			}

			m_selectedPreset = frame.data8[3] < efi::size(m_presetValues) ? frame.data8[3] : InvalidPreset;
			m_position = readU8Percent(frame.data8[4]);
			m_controllerTarget = readU8Percent(frame.data8[5]);
			m_alarms = readU16(frame, 6);
		} else {
			m_position = readU16Percent(frame, 0);
			m_controllerTarget = readU16Percent(frame, 2);
			m_selectedPreset = frame.data8[4] < efi::size(m_presetValues) ? frame.data8[4] : InvalidPreset;
			m_state = frame.data8[5];
			m_alarms = readU16(frame, 6);
		}
	} else {
		m_alarms = readU16(frame, 0);
		m_position = readU16Percent(frame, 2);
		m_controllerTarget = readU16Percent(frame, 4);
		m_current = readU16(frame, 6);
	}

	m_lastRxTimer.reset(nowNt);
}

bool ShockPreload::isOnline() const {
	return isEnabled() && !m_lastRxTimer.hasElapsedSec(OnlineTimeoutSeconds);
}

bool ShockPreload::isAtSavedPreset() const {
	if (!isOnline()) {
		return false;
	}

	for (size_t i = 0; i < efi::size(m_presetValues); i++) {
		if (isPositionAtPreset(i)) {
			return true;
		}
	}

	return false;
}

bool ShockPreload::isPresetActive(uint8_t preset) const {
	return isOnline() && m_selectedPreset == preset && isPositionAtPreset(preset);
}

bool ShockPreload::isPositionAtPreset(size_t preset) const {
	if (preset >= efi::size(m_presetValues) || !(m_seenPresets & (1 << preset))) {
		return false;
	}

	constexpr int tolerance = 1;
	return std::abs(static_cast<int>(m_position) - m_presetValues[preset]) <= tolerance;
}

void ShockPreload::pollStatus() {
	if (!isEnabled()) {
		return;
	}

	if (m_pollTimer.hasElapsedMs(StatusPollIntervalMs)) {
		sendSimpleCommand(CommandRequestStatus);
		m_pollTimer.reset();
	}
}

void ShockPreload::sendSimpleCommand(uint8_t command) {
	if (!isEnabled()) {
		return;
	}

#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
	CanTxMessage msg(CommandId, CommandFrameDlc, Bus, false);
	msg[0] = command;
#else
	(void)command;
#endif
}

void ShockPreload::sendValueCommand(uint8_t command, uint16_t value) {
	if (!isEnabled()) {
		return;
	}

#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
	CanTxMessage msg(CommandId, CommandFrameDlc, Bus, false);
	msg[0] = command;
	msg[1] = value & 0xff;
	msg[2] = value >> 8;
#else
	(void)command;
	(void)value;
#endif
}

void ShockPreload::sendRelativeCommand(int16_t value) {
	if (!isEnabled()) {
		return;
	}

#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
	CanTxMessage msg(CommandId, CommandFrameDlc, Bus, false);
	msg[0] = CommandMoveRelative;
	msg[1] = static_cast<uint16_t>(value) & 0xff;
	msg[2] = static_cast<uint16_t>(value) >> 8;
#else
	(void)value;
#endif
}

void ShockPreload::sendPresetCommand(uint8_t command, uint8_t slot) {
	if (!isEnabled() || slot > 4) {
		return;
	}

#if EFI_CAN_SUPPORT || EFI_UNIT_TEST
	CanTxMessage msg(CommandId, CommandFrameDlc, Bus, false);
	msg[0] = command;
	msg[1] = slot;
#else
	(void)command;
	(void)slot;
#endif
}

void ShockPreload::handleTsCommand(uint16_t command) {
	if (!isEnabled()) {
		return;
	}

	switch (command) {
		case SHOCK_PRELOAD_MOVE_DOWN_2:
			sendRelativeCommand(-2);
			break;
		case SHOCK_PRELOAD_MOVE_UP_2:
			sendRelativeCommand(2);
			break;
		case SHOCK_PRELOAD_LOAD_SLOT_0:
		case SHOCK_PRELOAD_LOAD_SLOT_1:
		case SHOCK_PRELOAD_LOAD_SLOT_2:
		case SHOCK_PRELOAD_LOAD_SLOT_3:
		case SHOCK_PRELOAD_LOAD_SLOT_4:
			m_selectedPreset = command - SHOCK_PRELOAD_LOAD_SLOT_0;
			sendPresetCommand(CommandLoadPreset, m_selectedPreset);
			break;
		case SHOCK_PRELOAD_SAVE_SLOT_0:
		case SHOCK_PRELOAD_SAVE_SLOT_1:
		case SHOCK_PRELOAD_SAVE_SLOT_2:
		case SHOCK_PRELOAD_SAVE_SLOT_3:
		case SHOCK_PRELOAD_SAVE_SLOT_4:
			m_selectedPreset = command - SHOCK_PRELOAD_SAVE_SLOT_0;
			m_presetValues[m_selectedPreset] = m_position;
			m_seenPresets |= 1 << m_selectedPreset;
			sendPresetCommand(CommandSavePreset, m_selectedPreset);
			break;
		case SHOCK_PRELOAD_STOP:
			sendSimpleCommand(CommandStop);
			break;
		case SHOCK_PRELOAD_CLEAR_ALARMS:
			sendSimpleCommand(CommandClearAlarms);
			break;
		case SHOCK_PRELOAD_REQUEST_STATUS:
			sendSimpleCommand(CommandRequestStatus);
			break;
		case SHOCK_PRELOAD_SET_REFERENCE_0:
			sendValueCommand(CommandSetReference, 0);
			break;
		case SHOCK_PRELOAD_SET_REFERENCE_50:
			sendValueCommand(CommandSetReference, 50);
			break;
		case SHOCK_PRELOAD_CALIBRATE:
			sendSimpleCommand(CommandCalibrate);
			break;
		case SHOCK_PRELOAD_SET_TARGET:
			sendValueCommand(CommandSetTarget, std::min<uint8_t>(engineConfiguration->shockPreloadCommandTarget, 100));
			break;
		default:
			break;
	}
}
