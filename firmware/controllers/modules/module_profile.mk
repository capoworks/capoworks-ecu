# Shared resolved module profile. This file intentionally defines no build rules:
# it is included while Make is still discovering its default goal.
ifndef FOME_MODULE_PROFILE_INCLUDED
FOME_MODULE_PROFILE_INCLUDED := 1

FOME_MODULE_CATALOG := fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump shock_preload cylinder_fuel_trim sd_card
MODULE_ROOT := $(abspath $(PROJECT_DIR)/controllers/modules)
MODULE_PROFILE_TOOL := $(MODULE_ROOT)/module_profile.sh
MODULE_PROFILE_FILE := $(abspath $(PROJECT_DIR)/generated/module_profile.txt)

# Boards own their module policy. Keep the declarations canonical and available
# to host-only queries as well as normal firmware and bootloader parsing.
FOME_MODULES_BOARD_REQUIRED := $(sort $(strip $(FOME_MODULES_BOARD_REQUIRED)))
FOME_MODULES_BOARD_UNSUPPORTED := $(sort $(strip $(FOME_MODULES_BOARD_UNSUPPORTED)))
FOME_MODULES_BOARD_REQUIRED_UNKNOWN := $(filter-out $(FOME_MODULE_CATALOG),$(FOME_MODULES_BOARD_REQUIRED))
ifneq ($(strip $(FOME_MODULES_BOARD_REQUIRED_UNKNOWN)),)
  $(error Board $(notdir $(BOARD_DIR)) requires unknown FOME_MODULES module(s): $(FOME_MODULES_BOARD_REQUIRED_UNKNOWN). Expected modules from: $(FOME_MODULE_CATALOG))
endif
FOME_MODULES_BOARD_REQUIRED_UNSUPPORTED := $(filter $(FOME_MODULES_BOARD_REQUIRED),$(FOME_MODULES_BOARD_UNSUPPORTED))
ifneq ($(strip $(FOME_MODULES_BOARD_REQUIRED_UNSUPPORTED)),)
  $(error Board $(notdir $(BOARD_DIR)) declares FOME_MODULES module(s) as both required and unsupported: $(FOME_MODULES_BOARD_REQUIRED_UNSUPPORTED))
endif

# These modules are available for explicit selection but are not part of the
# default profile on any board.
FOME_MODULES_DEFAULT := $(filter-out hella_opst emp_pump shock_preload,$(FOME_MODULE_CATALOG))

# An omitted selection uses the default profile, minus any modules the board
# declares unsupported. An explicitly empty selection remains empty.
ifeq ($(origin FOME_MODULES),undefined)
  FOME_MODULES := $(filter-out $(FOME_MODULES_BOARD_UNSUPPORTED),$(FOME_MODULES_DEFAULT))
endif
FOME_MODULES_UNKNOWN := $(filter-out $(FOME_MODULE_CATALOG),$(FOME_MODULES))
ifneq ($(strip $(FOME_MODULES_UNKNOWN)),)
  $(error Unknown FOME_MODULES module(s): $(FOME_MODULES_UNKNOWN). Expected modules from: $(FOME_MODULE_CATALOG))
endif

# Resolve in catalog order and remove duplicates regardless of selection order.
FOME_MODULES_RESOLVED := $(filter $(FOME_MODULES),$(FOME_MODULE_CATALOG))

FOME_MODULES_BOARD_UNSUPPORTED_SELECTED := $(filter $(FOME_MODULES_RESOLVED),$(FOME_MODULES_BOARD_UNSUPPORTED))
ifneq ($(strip $(FOME_MODULES_BOARD_UNSUPPORTED_SELECTED)),)
  $(error Board $(notdir $(BOARD_DIR)) does not support FOME_MODULES module(s): $(FOME_MODULES_BOARD_UNSUPPORTED_SELECTED). Remove them from FOME_MODULES or select a supported board)
endif

FOME_MODULES_BOARD_REQUIRED_MISSING := $(filter-out $(FOME_MODULES_RESOLVED),$(FOME_MODULES_BOARD_REQUIRED))
ifneq ($(strip $(FOME_MODULES_BOARD_REQUIRED_MISSING)),)
  $(error Board $(notdir $(BOARD_DIR)) requires FOME_MODULES module(s): $(FOME_MODULES_BOARD_REQUIRED_MISSING). Add them to FOME_MODULES)
endif

ifneq ($(strip $(NO_MAP_AVERAGING)),)
  FOME_MODULES_RESOLVED := $(filter-out map_averaging,$(FOME_MODULES_RESOLVED))
endif

# Fuel table dimensions are build-time profile values. FUEL_RPM_COUNT remains
# the shared 16-column axis used by lambda, MAP estimate, and injection phase;
# the dedicated VE dimensions are rendered into the generated configuration.
FOME_VE_RPM_COUNT ?= 16
ifneq ($(words $(strip $(FOME_VE_RPM_COUNT))),1)
  $(error FOME_VE_RPM_COUNT must be 16 or 32, got '$(FOME_VE_RPM_COUNT)')
endif
ifneq ($(strip $(filter-out 16 32,$(strip $(FOME_VE_RPM_COUNT)))),)
  $(error FOME_VE_RPM_COUNT must be 16 or 32, got '$(FOME_VE_RPM_COUNT)')
endif
FOME_VE_RPM_COUNT_RESOLVED := $(strip $(FOME_VE_RPM_COUNT))

FOME_AIRMASS_BLEND_SIZE ?= 8
ifneq ($(words $(strip $(FOME_AIRMASS_BLEND_SIZE))),1)
  $(error FOME_AIRMASS_BLEND_SIZE must be 8 or 16, got '$(FOME_AIRMASS_BLEND_SIZE)')
endif
ifneq ($(strip $(filter-out 8 16,$(strip $(FOME_AIRMASS_BLEND_SIZE)))),)
  $(error FOME_AIRMASS_BLEND_SIZE must be 8 or 16, got '$(FOME_AIRMASS_BLEND_SIZE)')
endif
FOME_AIRMASS_BLEND_SIZE_RESOLVED := $(strip $(FOME_AIRMASS_BLEND_SIZE))

# Cylinder fuel trim owns a fixed-capacity subset of MAX_CYLINDER_COUNT. The
# public build parameter is meaningful only while its module is selected.
ifneq ($(filter cylinder_fuel_trim,$(FOME_MODULES_RESOLVED)),)
  FOME_CYLINDER_TRIM_COUNT ?= 6
  ifneq ($(words $(strip $(FOME_CYLINDER_TRIM_COUNT))),1)
    $(error FOME_CYLINDER_TRIM_COUNT must be an integer from 1 to 12 when cylinder_fuel_trim is selected, got '$(FOME_CYLINDER_TRIM_COUNT)')
  endif
  ifneq ($(strip $(filter-out 1 2 3 4 5 6 7 8 9 10 11 12,$(strip $(FOME_CYLINDER_TRIM_COUNT)))),)
    $(error FOME_CYLINDER_TRIM_COUNT must be an integer from 1 to 12 when cylinder_fuel_trim is selected, got '$(FOME_CYLINDER_TRIM_COUNT)')
  endif
  FOME_CYLINDER_TRIM_COUNT_RESOLVED := $(strip $(FOME_CYLINDER_TRIM_COUNT))
else
  # Keep the generated constant stable and harmless when the optional feature
  # is disabled, regardless of any stale or irrelevant command-line value.
  FOME_CYLINDER_TRIM_COUNT_RESOLVED := 0
endif

# EMP derives cooling demand from both fan-controller states.
ifneq ($(filter emp_pump,$(FOME_MODULES_RESOLVED)),)
  ifeq ($(filter fan,$(FOME_MODULES_RESOLVED)),)
    $(error FOME_MODULES emp_pump requires fan)
  endif
endif

# Engine presets depend on the validated table dimensions. This profile layer
# consumes those fields and restricts presets that are incompatible with them.
include $(PROJECT_DIR)/controllers/modules/engine_preset_profile.mk

MODULE_NAMES := $(strip $(FOME_MODULES_RESOLVED))

# Build-time exclusions are supported only for features with audited consumers.
# MAP averaging remains available through its existing NO_MAP_AVERAGING switch.
MODULE_RESOLVED_DIRS := $(foreach module,$(MODULE_NAMES),$(MODULE_ROOT)/$(module))
MODULE_RECIPE_FILES := $(foreach module,$(MODULE_NAMES),$(MODULE_ROOT)/$(module)/$(module).mk)

# Conventional module-owned fragments do not require a second registry.
MODULE_CONFIG_FRAGMENTS := $(foreach dir,$(MODULE_RESOLVED_DIRS),$(wildcard $(dir)/config.txt))
MODULE_PERSISTENT_CONFIG_FRAGMENTS := $(foreach dir,$(MODULE_RESOLVED_DIRS),$(wildcard $(dir)/persistent_config.txt))
MODULE_OUTPUT_FRAGMENTS := $(foreach dir,$(MODULE_RESOLVED_DIRS),$(wildcard $(dir)/output_channels.txt))
MODULE_INI_FRAGMENTS := $(foreach dir,$(MODULE_RESOLVED_DIRS),$(wildcard $(dir)/tunerstudio.ini))
MODULE_FRAGMENTS := $(MODULE_CONFIG_FRAGMENTS) $(MODULE_PERSISTENT_CONFIG_FRAGMENTS) $(MODULE_OUTPUT_FRAGMENTS) $(MODULE_INI_FRAGMENTS)

# Profiles must run sequentially in the same checkout because generated outputs are shared.
MODULE_PROFILE_BOARD = $(abspath $(BOARD_DIR))
MODULE_PROFILE_FRAGMENT_ARGS = \
	$(foreach fragment,$(MODULE_CONFIG_FRAGMENTS),config '$(fragment)') \
	$(foreach fragment,$(MODULE_PERSISTENT_CONFIG_FRAGMENTS),persistent '$(fragment)') \
	$(foreach fragment,$(MODULE_OUTPUT_FRAGMENTS),output '$(fragment)') \
	$(foreach fragment,$(MODULE_INI_FRAGMENTS),ini '$(fragment)')
MODULE_PROFILE_WRITE_CMD = '$(MODULE_PROFILE_TOOL)' write '$(MODULE_PROFILE_FILE)' '$(MODULE_PROFILE_BOARD)' '$(PROJECT_CPU)' '$(MODULE_NAMES)' --ve-rpm-count '$(FOME_VE_RPM_COUNT_RESOLVED)' --airmass-blend-size '$(FOME_AIRMASS_BLEND_SIZE_RESOLVED)' --cylinder-fuel-trim-count '$(FOME_CYLINDER_TRIM_COUNT_RESOLVED)' --engine-preset-mode '$(MODULE_ENGINE_PRESET_PROFILE_MODE)' --engine-presets '$(MODULE_ENGINE_PRESET_PROFILE_SELECTED)' --engine-preset-default '$(MODULE_ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE)' $(MODULE_PROFILE_FRAGMENT_ARGS)

# Cylinder fuel trims need one UI row per configured cylinder. Render the small
# module-owned template after the validated profile has been written, then compose
# it through the existing TunerStudio fragment path.
MODULE_CYLINDER_TRIM_TEMPLATE := $(MODULE_ROOT)/cylinder_fuel_trim/tunerstudio.template.ini
MODULE_CYLINDER_TRIM_RENDERER := $(MODULE_ROOT)/cylinder_fuel_trim/render_tunerstudio.sh
MODULE_CYLINDER_TRIM_INI := $(dir $(MODULE_PROFILE_FILE))cylinder_fuel_trim.ini
MODULE_PROFILE_OUTPUTS := $(MODULE_CYLINDER_TRIM_INI)
MODULE_PROFILE_OUTPUT_DEPS := $(MODULE_CYLINDER_TRIM_TEMPLATE) $(MODULE_CYLINDER_TRIM_RENDERER)
MODULE_PROFILE_OUTPUT_CMD = '$(MODULE_CYLINDER_TRIM_RENDERER)' "$$('$(MODULE_PROFILE_TOOL)' generation-count '$(MODULE_PROFILE_FILE)')" '$(MODULE_CYLINDER_TRIM_TEMPLATE)' '$(MODULE_CYLINDER_TRIM_INI)'
MODULE_PROFILE_REFRESH_CMD = $(MODULE_PROFILE_WRITE_CMD) && $(MODULE_PROFILE_OUTPUT_CMD)
ifneq ($(filter cylinder_fuel_trim,$(FOME_MODULES_RESOLVED)),)
  MODULE_INI_FRAGMENTS += $(MODULE_CYLINDER_TRIM_INI)
  MODULE_FRAGMENTS += $(MODULE_CYLINDER_TRIM_INI)
endif

endif # FOME_MODULE_PROFILE_INCLUDED
