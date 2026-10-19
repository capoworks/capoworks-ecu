# Shared resolved module profile. This file intentionally defines no build rules:
# it is included while Make is still discovering its default goal.

FOME_MODULE_CATALOG := fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump

# The default selection includes available modules, with named opt-ins disabled.
# An explicit FOME_MODULES selection can enable any available module.
# An explicitly empty selection remains empty.
FOME_MODULES_DEFAULT := $(filter-out hella_opst emp_pump,$(FOME_MODULE_CATALOG))
FOME_MODULES ?= $(FOME_MODULES_DEFAULT)
FOME_MODULES_UNKNOWN := $(filter-out $(FOME_MODULE_CATALOG),$(FOME_MODULES))

ifneq ($(strip $(FOME_MODULES_UNKNOWN)),)
  $(error Unknown FOME_MODULES module(s): $(FOME_MODULES_UNKNOWN). Expected modules from: $(FOME_MODULE_CATALOG))
endif

# Resolve in catalog order and remove duplicates regardless of selection order.
FOME_MODULES_RESOLVED := $(filter $(FOME_MODULES),$(FOME_MODULE_CATALOG))

ifneq ($(strip $(NO_MAP_AVERAGING)),)
  FOME_MODULES_RESOLVED := $(filter-out map_averaging,$(FOME_MODULES_RESOLVED))
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
MODULE_ROOT := $(abspath $(PROJECT_DIR)/controllers/modules)
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
MODULE_PROFILE_TOOL := $(MODULE_ROOT)/module_profile.sh
MODULE_PROFILE_FRAGMENT_ARGS = \
	$(foreach fragment,$(MODULE_CONFIG_FRAGMENTS),config '$(fragment)') \
	$(foreach fragment,$(MODULE_PERSISTENT_CONFIG_FRAGMENTS),persistent '$(fragment)') \
	$(foreach fragment,$(MODULE_OUTPUT_FRAGMENTS),output '$(fragment)') \
	$(foreach fragment,$(MODULE_INI_FRAGMENTS),ini '$(fragment)')
MODULE_PROFILE_WRITE_CMD = '$(MODULE_PROFILE_TOOL)' write '$(MODULE_PROFILE_FILE)' '$(MODULE_PROFILE_BOARD)' '$(PROJECT_CPU)' '$(MODULE_NAMES)' $(MODULE_PROFILE_FRAGMENT_ARGS)
