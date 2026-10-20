# Shared resolved module profile. This file intentionally defines no build rules:
# it is included while Make is still discovering its default goal.

FOME_MODULE_CATALOG := fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump shock_preload cylinder_fuel_trim
MODULE_ROOT := $(abspath $(PROJECT_DIR)/controllers/modules)
MODULE_PROFILE_TOOL := $(MODULE_ROOT)/module_profile.sh

# These modules are available for explicit selection but are not part of the
# default profile on any board.
FOME_MODULES_DEFAULT := $(filter-out hella_opst emp_pump shock_preload cylinder_fuel_trim,$(FOME_MODULE_CATALOG))

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

ifneq ($(strip $(NO_MAP_AVERAGING)),)
  FOME_MODULES_RESOLVED := $(filter-out map_averaging,$(FOME_MODULES_RESOLVED))
endif

# Cylinder fuel trim owns a fixed-capacity subset of MAX_CYLINDER_COUNT. The
# public build parameter is meaningful only while its module is selected.
ifneq ($(filter cylinder_fuel_trim,$(FOME_MODULES_RESOLVED)),)
  ifeq ($(origin FOME_CYLINDER_TRIM_COUNT),undefined)
    $(error FOME_CYLINDER_TRIM_COUNT is required when FOME_MODULES includes cylinder_fuel_trim)
  endif
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
MODULE_PROFILE_FILE := $(abspath $(PROJECT_DIR)/generated/module_profile.txt)
MODULE_PROFILE_BOARD = $(abspath $(BOARD_DIR))
MODULE_PROFILE_FRAGMENT_ARGS = \
	$(foreach fragment,$(MODULE_CONFIG_FRAGMENTS),config '$(fragment)') \
	$(foreach fragment,$(MODULE_PERSISTENT_CONFIG_FRAGMENTS),persistent '$(fragment)') \
	$(foreach fragment,$(MODULE_OUTPUT_FRAGMENTS),output '$(fragment)') \
	$(foreach fragment,$(MODULE_INI_FRAGMENTS),ini '$(fragment)')
MODULE_PROFILE_WRITE_CMD = '$(MODULE_PROFILE_TOOL)' write '$(MODULE_PROFILE_FILE)' '$(MODULE_PROFILE_BOARD)' '$(PROJECT_CPU)' '$(MODULE_NAMES)' --cylinder-fuel-trim-count '$(FOME_CYLINDER_TRIM_COUNT_RESOLVED)' $(MODULE_PROFILE_FRAGMENT_ARGS)

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
