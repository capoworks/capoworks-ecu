#pragma once

#include "configuration_write_state.h"
#include "flash_int.h"
#include "persistent_configuration.h"

enum class ConfigurationFlashState {
	Ok,
	CrcFailed,
	IncompatibleVersion,
	BlankChip,
	ReadFailed,
};

enum class ConfigurationWritePhase {
	Complete,
	Layout,
	Erase,
	Program,
	Verify,
};

struct ConfigurationWriteResult {
	ConfigurationWritePhase phase;
	flashaddr_t address;
	int error;
	// Copies whose erase, program, byte comparison, and header/CRC checks all passed.
	unsigned verifiedCopies;

	bool success() const {
		return phase == ConfigurationWritePhase::Complete;
	}
};

ConfigurationFlashState
readConfigurationCopies(flashaddr_t first, flashaddr_t second, persistent_config_container_s& destination);

ConfigurationWriteResult
writeConfigurationCopies(flashaddr_t first, flashaddr_t second, const persistent_config_container_s& data);
