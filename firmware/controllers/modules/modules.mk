include $(PROJECT_DIR)/controllers/modules/module_profile.mk

# Module recipes only add C++ sources, include paths, macros and generated-module tuples.
# The profile above remains the only catalog/selection source used by generation.
include $(MODULE_RECIPE_FILES)
