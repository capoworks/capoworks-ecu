MODULES_INC += $(PROJECT_DIR)/controllers/modules/shock_preload
MODULES_CPPSRC += $(PROJECT_DIR)/controllers/modules/shock_preload/shock_preload.cpp
MODULES_INCLUDE += \#include "shock_preload.h"\n
MODULES_LIST += ShockPreload,

DDEFS += -DMODULE_SHOCK_PRELOAD
