# Code generation runs in two stages with independent stamp files:
#   1. live data   (gen_live_documentation.sh -> LiveDataProcessor)
#   2. main config (gen_config_board.sh        -> ConfigDefinition), which consumes
#      generated/total_live_data_generated.h and the output_channels/data_logs ini
#      produced by stage 1, hence config depends on the live-data stamp.
# Splitting the stamps keeps the dependency graph acyclic and lets each stage rerun
# independently when only its own inputs change.
LIVE_DATA_STAMP := $(BUILDDIR)/generated_live_data.stamp
CONFIG_STAMP := $(BUILDDIR)/generated_config.stamp

# Generated file lists - populated by the Java tools into these .mk fragments.
LIVE_DATA_GENERATED_FILES :=
CONFIG_GENERATED_FILES :=
-include $(PROJECT_DIR)/.dep/generated_live_data_outputs.mk
-include $(PROJECT_DIR)/.dep/generated_config_outputs.mk
GENERATED_FILES := $(LIVE_DATA_GENERATED_FILES) $(CONFIG_GENERATED_FILES)

JAVA_TOOLS = $(BUILDDIR)/java_tools.cookie
JAVA_TOOLS_SOURCE_DIR := $(abspath $(PROJECT_DIR)/../java_tools)
JAVA_TOOL_SOURCES := $(shell find '$(JAVA_TOOLS_SOURCE_DIR)' -type d \( -name build -o -name bin -o -name .gradle -o -name generated \) -prune -o -type f \( -name '*.java' -o -name '*.g4' -o -name 'build.gradle' -o -name 'settings.gradle' -o -name 'dependencies.gradle' -o -name 'gradle.properties' -o -name 'build_tools.sh' -o -name 'gradlew' -o -name 'gradle-wrapper.properties' \) -print 2>/dev/null)

$(JAVA_TOOLS) : $(JAVA_TOOL_SOURCES)
	cd $(PROJECT_DIR)/../java_tools && ./build_tools.sh
	echo "done" > $@

# Refresh the source-tree-global profile on every generation-capable Make invocation.
# The helper replaces it only when its contents change, so identical board/selection
# builds do not invalidate generated outputs or objects. The recipe must be attached to
# the profile itself: an empty-recipe target behind a phony prerequisite is not restatted
# after its prerequisite runs, which delays A->B invalidation until the next Make run.
.PHONY: module-profile MODULE_PROFILE_FORCE
module-profile: $(MODULE_PROFILE_FILE) $(MODULE_PROFILE_OUTPUTS)

$(MODULE_PROFILE_FILE): MODULE_PROFILE_FORCE
	@$(MODULE_PROFILE_REFRESH_CMD)

ifneq ($(strip $(MODULE_PROFILE_OUTPUTS)),)
$(MODULE_PROFILE_OUTPUTS): $(MODULE_PROFILE_FILE) $(MODULE_PROFILE_OUTPUT_DEPS)
	@test -f '$@' || $(MODULE_PROFILE_OUTPUT_CMD)
endif

MODULE_PROFILE_FORCE:

# Stage 1: live data. Extra prerequisites (LiveData.yaml, module *.txt) come from
# .dep/fome_live_data.d, which is auto-included by the build rules.
$(LIVE_DATA_STAMP) : $(JAVA_TOOLS) $(MODULE_PROFILE_FILE) $(MODULE_FRAGMENTS) $(PROJECT_DIR)/integration/LiveData.yaml
	@echo Generating live data...
	cd $(PROJECT_DIR) && ./gen_live_documentation.sh "$(LIVE_DATA_STAMP)" "$(MODULE_PROFILE_FILE)"
	@touch $@

# Stage 2: persistent config + TunerStudio ini. Depends on the live-data stamp because it
# consumes stage 1 outputs. Extra prerequisites come from .dep/fome_generated.d.
$(CONFIG_STAMP) : $(JAVA_TOOLS) $(MODULE_PROFILE_FILE) $(MODULE_FRAGMENTS) $(LIVE_DATA_STAMP) $(PROJECT_DIR)/integration/fome_config.txt $(PROJECT_DIR)/hw_layer/mass_storage/create_ini_image_compressed.sh
	@echo Generating config files...
	cd $(PROJECT_DIR) && ./gen_config_board.sh "$(realpath $(BOARD_DIR))" "$(SHORT_BOARD_NAME)" "$(CONFIG_STAMP)" "$(MODULE_PROFILE_FILE)"
	@touch $@

# Associate each generated file with the stamp recipe that produces it ("multiple outputs
# from one recipe" idiom). These rules carry no recipe, so they never bump a file's mtime -
# a generated file only looks newer to its consumers when its content actually changed.
$(LIVE_DATA_GENERATED_FILES) : $(LIVE_DATA_STAMP)
$(CONFIG_GENERATED_FILES) : $(CONFIG_STAMP)

# Order-only dependency: generation must complete before the first compile, but a stamp
# being newer must NOT force every object to rebuild. The compiler-emitted .dep/*.o.d files
# carry the real generated-header dependencies, so after the first build only the translation
# units whose generated inputs actually changed get recompiled.
$(OBJS) : $(MODULE_PROFILE_FILE)
$(PCHOBJ) : $(MODULE_PROFILE_FILE)
$(OBJS) : | $(CONFIG_STAMP)
$(PCHOBJ) : | $(CONFIG_STAMP)

CLEAN_GENERATED_HOOK:
	rm -f $(GENERATED_DIR)/*
	rm -f $(LIVE_DATA_STAMP) $(CONFIG_STAMP) $(MODULE_PROFILE_FILE)
