MODULES_INC += $(PROJECT_DIR)/controllers/modules/cylinder_fuel_trim
MODULES_CPPSRC += $(PROJECT_DIR)/controllers/modules/cylinder_fuel_trim/cylinder_fuel_trim.cpp

# Cylinder fuel trim is a compile-time functional module with no runtime instance.
DDEFS += -DMODULE_CYLINDER_FUEL_TRIM
