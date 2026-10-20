#pragma once

/*
 * Build-time engine preset feature switches.
 *
 * A build that includes engine_preset_profile.mk defines
 * FOME_ENGINE_PRESET_PROFILE_CONFIGURED and compiles the generated switches.
 * Keeping a permissive fallback allows source-only consumers and legacy build
 * entry points to retain the historical all-presets behavior.
 */
#ifdef FOME_ENGINE_PRESET_PROFILE_CONFIGURED
#include "../../generated/engine_preset_profile_generated.h"
#else
#define FOME_ENGINE_PRESETS_EXPANDED 0
#define FOME_ENGINE_PRESET_DEFAULT_ENGINE_TYPE_ENABLED 1
#define FOME_ENGINE_PRESET_ENABLED(symbol) 1
#endif
