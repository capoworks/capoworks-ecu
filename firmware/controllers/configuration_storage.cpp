/**
 * @file configuration_storage.cpp
 * Capoworks adaptations and modifications by Giacomo di Ciocco, (C) 2026.
 */

#include "pch.h"

#if EFI_INTERNAL_FLASH || EFI_UNIT_TEST

#include "configuration_storage.h"
#include "crc_accelerator.h"

static ConfigurationFlashState validateHeader(const persistent_config_header_s& header) {
	if (header.version == UINT32_MAX && header.size == UINT32_MAX && header.layoutIdLow == UINT32_MAX &&
		header.layoutIdHigh == UINT32_MAX) {
		return ConfigurationFlashState::BlankChip;
	}
	if (header.version != FLASH_DATA_VERSION || header.size != sizeof(persistent_config_container_s) ||
		getPersistentConfigLayoutId(header) != CONFIG_LAYOUT_ID) {
		return ConfigurationFlashState::IncompatibleVersion;
	}
	return ConfigurationFlashState::Ok;
}

static ConfigurationFlashState readCopy(flashaddr_t address, persistent_config_container_s& destination) {
	if (!address) {
		return ConfigurationFlashState::BlankChip;
	}

	persistent_config_header_s header;
	if (intFlashRead(address, reinterpret_cast<char*>(&header), sizeof(header)) != FLASH_RETURN_SUCCESS) {
		return ConfigurationFlashState::ReadFailed;
	}
	auto headerState = validateHeader(header);
	if (headerState != ConfigurationFlashState::Ok) {
		return headerState;
	}

	if (intFlashRead(
				address + offsetof(persistent_config_container_s, persistentConfiguration),
				reinterpret_cast<char*>(&destination.persistentConfiguration),
				sizeof(destination.persistentConfiguration)) != FLASH_RETURN_SUCCESS) {
		return ConfigurationFlashState::ReadFailed;
	}

	uint32_t storedCrc;
	if (intFlashRead(
				address + offsetof(persistent_config_container_s, value),
				reinterpret_cast<char*>(&storedCrc),
				sizeof(storedCrc)) != FLASH_RETURN_SUCCESS) {
		return ConfigurationFlashState::ReadFailed;
	}

	uint32_t calculatedCrc =
			singleCrc(&destination.persistentConfiguration, sizeof(destination.persistentConfiguration));
	if (storedCrc != calculatedCrc) {
		return storedCrc == UINT32_MAX ? ConfigurationFlashState::BlankChip : ConfigurationFlashState::CrcFailed;
	}

	destination.header = header;
	destination.value = storedCrc;
	return ConfigurationFlashState::Ok;
}

ConfigurationFlashState
readConfigurationCopies(flashaddr_t first, flashaddr_t second, persistent_config_container_s& destination) {
	auto state = readCopy(first, destination);
	return state == ConfigurationFlashState::Ok || !second ? state : readCopy(second, destination);
}

// Check a stored copy without allocating another full configuration in RAM.
// Every read goes through the driver so M7 cache invalidation also applies.
static ConfigurationFlashState checkCopy(flashaddr_t address) {
	persistent_config_header_s header;

	if (!address || intFlashRead(address, reinterpret_cast<char*>(&header), sizeof(header)) != FLASH_RETURN_SUCCESS) {
		return ConfigurationFlashState::ReadFailed;
	}
	auto headerState = validateHeader(header);
	if (headerState != ConfigurationFlashState::Ok) {
		return headerState;
	}

	Crc crc(sizeof(persistent_config_s));
	alignas(4) char buffer[128];
	for (size_t offset = 0; offset < sizeof(persistent_config_s); offset += sizeof(buffer)) {
		size_t count = std::min(sizeof(buffer), sizeof(persistent_config_s) - offset);
		if (intFlashRead(
					address + offsetof(persistent_config_container_s, persistentConfiguration) + offset,
					buffer,
					count) != FLASH_RETURN_SUCCESS) {
			return ConfigurationFlashState::ReadFailed;
		}
		crc.addData(buffer, count);
	}

	uint32_t storedCrc;
	if (intFlashRead(
				address + offsetof(persistent_config_container_s, value),
				reinterpret_cast<char*>(&storedCrc),
				sizeof(storedCrc)) != FLASH_RETURN_SUCCESS) {
		return ConfigurationFlashState::ReadFailed;
	}
	if (storedCrc != crc.getCrc()) {
		return storedCrc == UINT32_MAX ? ConfigurationFlashState::BlankChip : ConfigurationFlashState::CrcFailed;
	}
	return ConfigurationFlashState::Ok;
}

static bool invalidLayout(flashaddr_t first, flashaddr_t second, size_t size) {
	if (!first || !size) {
		return true;
	}
	if (!second) {
		return false;
	}
	if (first > UINTPTR_MAX - (size - 1) || second > UINTPTR_MAX - (size - 1)) {
		return true;
	}

	return intFlashSectorAt(first) <= intFlashSectorAt(second + size - 1) &&
		   intFlashSectorAt(second) <= intFlashSectorAt(first + size - 1);
}

ConfigurationWriteResult
writeConfigurationCopies(flashaddr_t first, flashaddr_t second, const persistent_config_container_s& data) {
	if (invalidLayout(first, second, sizeof(data))) {
		return {ConfigurationWritePhase::Layout, first, FLASH_RETURN_NO_PERMISSION, 0};
	}

	// If only the primary is valid, repair the backup before touching it. If the
	// backup is valid (or neither is), start with the primary as on boot.
	auto backup = second ? checkCopy(second) : ConfigurationFlashState::BlankChip;
	auto primary = checkCopy(first);
	if (backup != ConfigurationFlashState::Ok) {
		if (primary == ConfigurationFlashState::Ok) {
			// Repair and verify the invalid backup before replacing the sole valid primary.
			std::swap(first, second);
		} else if (primary == ConfigurationFlashState::ReadFailed || backup == ConfigurationFlashState::ReadFailed) {
			// An unreadable copy might be the only good one. Do not erase it
			// unless another copy has actually been validated.
			return {ConfigurationWritePhase::Verify, first, FLASH_RETURN_BAD_FLASH, 0};
		}
	}

	ConfigurationWriteResult result{ConfigurationWritePhase::Complete, 0, FLASH_RETURN_SUCCESS, 0};
	for (auto address : {first, second}) {
		if (!address) {
			continue;
		}
		auto error = intFlashErase(address, sizeof(data));
		if (error == FLASH_RETURN_SUCCESS && !intFlashIsErased(address, sizeof(data))) {
			error = FLASH_RETURN_BAD_FLASH;
		}
		if (error != FLASH_RETURN_SUCCESS) {
			return {ConfigurationWritePhase::Erase, address, error, result.verifiedCopies};
		}
		error = intFlashWrite(address, reinterpret_cast<const char*>(&data), sizeof(data));
		if (error != FLASH_RETURN_SUCCESS) {
			return {ConfigurationWritePhase::Program, address, error, result.verifiedCopies};
		}
		// A byte comparison alone is insufficient when tuning can change the
		// source during a write. The stored image must also have a valid CRC.
		if (!intFlashCompare(address, reinterpret_cast<const char*>(&data), sizeof(data)) ||
			checkCopy(address) != ConfigurationFlashState::Ok) {
			return {ConfigurationWritePhase::Verify, address, FLASH_RETURN_BAD_FLASH, result.verifiedCopies};
		}
		result.verifiedCopies++;
	}
	return result;
}

#endif
