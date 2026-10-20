MODULES_INC += $(PROJECT_DIR)/controllers/modules/emp_pump
MODULES_CPPSRC += $(PROJECT_DIR)/controllers/modules/emp_pump/emp_pump.cpp
MODULES_INCLUDE += \#include "emp_pump.h"\n
MODULES_LIST += EmpPump,

DDEFS += -DMODULE_EMP_PUMP
