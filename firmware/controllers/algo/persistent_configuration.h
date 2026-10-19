/*
 * @file persistent_configuration.h
 *
 * @date Feb 27, 2020
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config_layout_id.h"
#include "engine_configuration_generated_structures.h"

typedef struct {
	uint32_t version;
	uint32_t size;
	// Stored as low/high words so the on-flash ABI retains 4-byte alignment.
	uint32_t layoutIdLow;
	uint32_t layoutIdHigh;
} persistent_config_header_s;

typedef struct {
	persistent_config_header_s header;
	persistent_config_s persistentConfiguration;
	uint32_t value;
} persistent_config_container_s;

static_assert(sizeof(persistent_config_header_s) == 16, "Persistent configuration header must be 16 bytes");
static_assert(offsetof(persistent_config_header_s, version) == 0, "Unexpected persistent header version offset");
static_assert(offsetof(persistent_config_header_s, size) == 4, "Unexpected persistent header size offset");
static_assert(offsetof(persistent_config_header_s, layoutIdLow) == 8, "Unexpected persistent header layout offset");
static_assert(offsetof(persistent_config_header_s, layoutIdHigh) == 12, "Unexpected persistent header layout offset");
static_assert(
		offsetof(persistent_config_container_s, persistentConfiguration) == sizeof(persistent_config_header_s),
		"Persistent payload must immediately follow its header");
static_assert(
		offsetof(persistent_config_container_s, value) ==
				sizeof(persistent_config_header_s) + sizeof(persistent_config_s),
		"Persistent CRC must immediately follow its payload");
static_assert(
		sizeof(persistent_config_container_s) == offsetof(persistent_config_container_s, value) + 4,
		"Unexpected persistent container size");
static_assert(alignof(persistent_config_container_s) == 4, "Persistent configuration must remain 4-byte aligned");

static inline uint64_t getPersistentConfigLayoutId(const persistent_config_header_s& header) {
	return (static_cast<uint64_t>(header.layoutIdHigh) << 32) | header.layoutIdLow;
}

static inline void initializePersistentConfigHeader(persistent_config_container_s& data) {
	data.header.version = static_cast<uint32_t>(FLASH_DATA_VERSION);
	data.header.size = static_cast<uint32_t>(sizeof(persistent_config_container_s));
	data.header.layoutIdLow = static_cast<uint32_t>(CONFIG_LAYOUT_ID);
	data.header.layoutIdHigh = static_cast<uint32_t>(CONFIG_LAYOUT_ID >> 32);
}
