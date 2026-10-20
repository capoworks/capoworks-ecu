# Build-time engine preset selection.
#
# This file is included by module_profile.mk after the VE dimensions have been
# resolved. It has no default Make goal.

ifeq ($(strip $(MODULE_ROOT)),)
  $(error MODULE_ROOT must be available before including engine_preset_profile.mk)
endif

ENGINE_PRESET_PROFILE_ROOT := $(abspath $(MODULE_ROOT))
ENGINE_PRESET_PROFILE_TOOL := $(ENGINE_PRESET_PROFILE_ROOT)/engine_preset_profile.sh
ENGINE_PRESET_PROFILE_DISPATCH_SOURCE := $(abspath $(PROJECT_DIR)/controllers/algo/engine_configuration.cpp)
ENGINE_PRESET_PROFILE_ENUM_SOURCE := $(abspath $(PROJECT_DIR)/controllers/algo/engine_types.h)
ENGINE_PRESET_PROFILE_GENERATED_DIR := $(if $(strip $(MODULE_PROFILE_FILE)),$(dir $(MODULE_PROFILE_FILE)),$(abspath $(PROJECT_DIR)/generated)/)
ENGINE_PRESET_PROFILE_HEADER := $(ENGINE_PRESET_PROFILE_GENERATED_DIR)engine_preset_profile_generated.h
ENGINE_PRESET_PROFILE_METADATA := $(ENGINE_PRESET_PROFILE_GENERATED_DIR)engine_preset_profile_generated.mk
ENGINE_PRESET_PROFILE_TS_DEFINITIONS := $(ENGINE_PRESET_PROFILE_GENERATED_DIR)engine_preset_profile_ts_definitions.txt
ENGINE_PRESET_PROFILE_SELECTION_FILE := $(ENGINE_PRESET_PROFILE_GENERATED_DIR)engine_preset_selection.txt
ENGINE_PRESET_PROFILE_DEFAULT_FILE := $(ENGINE_PRESET_PROFILE_GENERATED_DIR)engine_preset_default.txt

# $(file ...) does not create parents. Prepare the generated directory before
# any profile input is written, without defining a Make rule or default goal.
$(shell mkdir -p '$(ENGINE_PRESET_PROFILE_GENERATED_DIR)')

ifeq ($(strip $(FOME_VE_RPM_COUNT_RESOLVED)),)
  $(error FOME_VE_RPM_COUNT_RESOLVED must be resolved before including engine_preset_profile.mk)
endif
ifeq ($(strip $(FOME_AIRMASS_BLEND_SIZE_RESOLVED)),)
  $(error FOME_AIRMASS_BLEND_SIZE_RESOLVED must be resolved before including engine_preset_profile.mk)
endif

ENGINE_PRESET_PROFILE_DEFAULT_FLAGS := $(sort $(filter -DDEFAULT_ENGINE_TYPE=%,$(DDEFS)))
ifneq ($(words $(ENGINE_PRESET_PROFILE_DEFAULT_FLAGS)),0)
ifneq ($(words $(ENGINE_PRESET_PROFILE_DEFAULT_FLAGS)),1)
  $(error Expected exactly one DEFAULT_ENGINE_TYPE definition, got '$(ENGINE_PRESET_PROFILE_DEFAULT_FLAGS)')
endif
endif
ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE := $(patsubst -DDEFAULT_ENGINE_TYPE=%,%,$(ENGINE_PRESET_PROFILE_DEFAULT_FLAGS))
ifeq ($(strip $(ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE)),)
  ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE := MINIMAL_PINS
endif

$(file >$(ENGINE_PRESET_PROFILE_SELECTION_FILE),$(FOME_ENGINE_PRESETS))
$(file >$(ENGINE_PRESET_PROFILE_DEFAULT_FILE),$(ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE))

ENGINE_PRESET_PROFILE_RENDER_RESULT := $(shell '$(ENGINE_PRESET_PROFILE_TOOL)' render \
	--header '$(ENGINE_PRESET_PROFILE_HEADER)' \
	--metadata '$(ENGINE_PRESET_PROFILE_METADATA)' \
	--ts-definitions '$(ENGINE_PRESET_PROFILE_TS_DEFINITIONS)' \
	--dispatch-source '$(ENGINE_PRESET_PROFILE_DISPATCH_SOURCE)' \
	--enum-source '$(ENGINE_PRESET_PROFILE_ENUM_SOURCE)' \
	--selection-file '$(ENGINE_PRESET_PROFILE_SELECTION_FILE)' \
	--selection-origin '$(origin FOME_ENGINE_PRESETS)' \
	--default-file '$(ENGINE_PRESET_PROFILE_DEFAULT_FILE)' \
	--ve-rpm '$(FOME_VE_RPM_COUNT_RESOLVED)' \
	--blend-size '$(FOME_AIRMASS_BLEND_SIZE_RESOLVED)' 2>&1)

ifneq ($(firstword $(ENGINE_PRESET_PROFILE_RENDER_RESULT)),success)
  $(error $(ENGINE_PRESET_PROFILE_RENDER_RESULT))
endif

-include $(ENGINE_PRESET_PROFILE_METADATA)

ifndef MODULE_ENGINE_PRESET_PROFILE_MODE
  $(error Engine preset profile metadata was not generated: $(ENGINE_PRESET_PROFILE_METADATA))
endif

MODULE_ENGINE_PRESET_PROFILE_HEADER := $(ENGINE_PRESET_PROFILE_HEADER)
MODULE_ENGINE_PRESET_PROFILE_METADATA := $(ENGINE_PRESET_PROFILE_METADATA)
MODULE_ENGINE_PRESET_PROFILE_TS_DEFINITIONS := $(ENGINE_PRESET_PROFILE_TS_DEFINITIONS)

DDEFS += -DFOME_ENGINE_PRESET_PROFILE_CONFIGURED=1
