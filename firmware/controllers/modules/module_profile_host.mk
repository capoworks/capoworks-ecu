# Host-only entry point for standalone generators. This must stay independent of
# the firmware Makefile so profile generation does not require ChibiOS or a compiler.
PROJECT_DIR ?= .
BOARDS_DIR ?= $(PROJECT_DIR)/config/boards
BOARD_DIR ?= $(BOARDS_DIR)/f407-discovery

ifeq ($(PROJECT_CPU),)
  PROJECT_CPU = ARCH_STM32F4
endif

# Board files contribute host-visible selection switches such as NO_MAP_AVERAGING.
include $(BOARD_DIR)/board.mk
include $(PROJECT_DIR)/controllers/modules/module_profile.mk

.PHONY: module-profile MODULE_PROFILE_FORCE
module-profile: $(MODULE_PROFILE_FILE) $(MODULE_PROFILE_OUTPUTS)

$(MODULE_PROFILE_FILE): MODULE_PROFILE_FORCE
	@$(MODULE_PROFILE_REFRESH_CMD)

ifneq ($(strip $(MODULE_PROFILE_OUTPUTS)),)
$(MODULE_PROFILE_OUTPUTS): $(MODULE_PROFILE_FILE) $(MODULE_PROFILE_OUTPUT_DEPS)
	@test -f '$@' || $(MODULE_PROFILE_OUTPUT_CMD)
endif

MODULE_PROFILE_FORCE:
