#!/bin/bash

# file gen_config_board.sh
#        for example ./gen_config_board.sh hellen/hellen128 hellen128 rusefi_hellen128mercedes.ini
#                 or ./gen_config_board.sh hellen/hellen128 hellen128
# which is short for ./gen_config_board.sh hellen/hellen128 hellen128 rusefi_hellen128.ini

set -euo pipefail

echo "This script reads fome_config.txt and produces firmware persistent configuration headers"
echo "the storage section of rusefiXXX.ini is updated as well"

if [ -z "$1" ]; then
	echo "Board name parameter expected"
	exit 1
fi

BOARD_DIR=$1
SHORT_BOARDNAME=$2
STAMP_FILE=${3:-}
PROFILE_FILE=${4:-}
INI="fome_${SHORT_BOARDNAME}.ini"
PROFILE_READER=controllers/modules/module_profile.sh
PROFILE_MAKE=controllers/modules/module_profile_host.mk
LIVE_DATA_OUTPUTS=(
	generated/live_data_ids.h
	generated/live_data_fragments.h
	generated/total_live_data_generated.h
	generated/log_fields_generated.h
	generated/output_lookup_generated.cpp
	generated/output_channels.ini
	generated/data_logs.ini
)

echo "BOARD_DIR=${BOARD_DIR} SHORT_BOARDNAME=${SHORT_BOARDNAME}"

PREPEND_FILE=${BOARD_DIR}/prepend.txt

if [ -z "$PROFILE_FILE" ]; then
	PROFILE_FILE=generated/module_profile.txt
	PROFILE_SNAPSHOT=$(mktemp)
	trap 'rm -f "$PROFILE_SNAPSHOT"' EXIT
	if [ -f "$PROFILE_FILE" ]; then
		cp "$PROFILE_FILE" "$PROFILE_SNAPSHOT"
	fi

	make --no-print-directory -f "$PROFILE_MAKE" module-profile \
		PROJECT_DIR="$PWD" \
		BOARD_DIR="$BOARD_DIR"

	LIVE_DATA_REFRESH=0
	if ! cmp -s "$PROFILE_SNAPSHOT" "$PROFILE_FILE"; then
		LIVE_DATA_REFRESH=1
	fi
	for output in "${LIVE_DATA_OUTPUTS[@]}"; do
		if [ ! -f "$output" ]; then
			LIVE_DATA_REFRESH=1
		fi
	done

	if (( LIVE_DATA_REFRESH )); then
		./gen_live_documentation.sh "" "$PROFILE_FILE"
	fi

	rm -f "$PROFILE_SNAPSHOT"
	trap - EXIT
fi

"$PROFILE_READER" check "$PROFILE_FILE"

CONFIG_FRAGMENTS=()
while IFS= read -r fragment; do
	CONFIG_FRAGMENTS+=("$fragment")
done < <("$PROFILE_READER" read "$PROFILE_FILE" config)

PERSISTENT_CONFIG_FRAGMENTS=()
while IFS= read -r fragment; do
	PERSISTENT_CONFIG_FRAGMENTS+=("$fragment")
done < <("$PROFILE_READER" read "$PROFILE_FILE" persistent)

TS_FRAGMENTS=()
while IFS= read -r fragment; do
	TS_FRAGMENTS+=("$fragment")
done < <("$PROFILE_READER" read "$PROFILE_FILE" ini)

FRAGMENT_ARGS=()
for fragment in ${CONFIG_FRAGMENTS[@]+"${CONFIG_FRAGMENTS[@]}"}; do
	FRAGMENT_ARGS+=(-configFragment "$fragment")
done
for fragment in ${PERSISTENT_CONFIG_FRAGMENTS[@]+"${PERSISTENT_CONFIG_FRAGMENTS[@]}"}; do
	FRAGMENT_ARGS+=(-persistentConfigFragment "$fragment")
done
for fragment in ${TS_FRAGMENTS[@]+"${TS_FRAGMENTS[@]}"}; do
	FRAGMENT_ARGS+=(-tsFragment "$fragment")
done

STAMP_ARGS=()
if [ -n "$STAMP_FILE" ]; then
	STAMP_ARGS=(-stampFile "$STAMP_FILE")
fi

# Allow the next command to fail, the board may not have a BOARD_SPECIFIC_URL
set +e
BOARD_SPECIFIC_URL=$(cat $PREPEND_FILE | grep MAIN_HELP_URL | cut -d " " -f 3 | sed -e 's/^"//' -e 's/"$//')
set -euo pipefail

echo "BOARD_SPECIFIC_URL=[$BOARD_SPECIFIC_URL] for [$SHORT_BOARDNAME] from [$BOARD_DIR]"
if [ "" = "$BOARD_SPECIFIC_URL" ]; then
  BOARD_SPECIFIC_URL=https://wiki.fome.tech/
fi
echo "BOARD_SPECIFIC_URL=[$BOARD_SPECIFIC_URL]"

# work in progress: migrating to fome_${BUNDLE_NAME}.txt
java \
	-DSystemOut.name=logs/gen_config_${SHORT_BOARDNAME} \
	-jar ../java_tools/ConfigDefinition.jar \
	${FRAGMENT_ARGS[@]+"${FRAGMENT_ARGS[@]}"} \
	-readfile OUTPUTS_SECTION_FROM_FILE generated/output_channels.ini \
	-readfile DATALOG_SECTION_FROM_FILE generated/data_logs.ini \
	-triggerInputFolder ../unit_tests \
	-field_lookup_file generated/value_lookup_generated.cpp generated/value_lookup_generated.md \
	-java_destination ../java_console/generated/src/main/java/com/rusefi/config/generated/Fields.java \
	-initialize_to_zero false \
	-branch "$(git branch --show-current)" \
	-ts_template ./tunerstudio/tunerstudio.template.ini \
	-ts_output_name ./tunerstudio/generated/${INI} \
	-board ${BOARD_DIR} \
	-boardName ${SHORT_BOARDNAME} \
	-prepend generated/total_live_data_generated.h \
	-prepend integration/fome_config_shared.txt \
	-prepend ${BOARD_DIR}/prepend.txt \
	-definition integration/fome_config.txt \
	-enumInputFile controllers/algo/engine_types.h \
	-enumInputFile controllers/algo/rusefi_enums.h \
	-enumInputFile controllers/algo/rusefi_hw_enums.h \
	-c_defines        generated/rusefi_generated.h \
	-c_destination    generated/engine_configuration_generated_structures.h \
	-makefileDep      .dep/fome_generated.d \
	${STAMP_ARGS[@]+"${STAMP_ARGS[@]}"}


[ $? -eq 0 ] || { echo "ERROR generating TunerStudio config for ${BOARD_DIR}"; exit 1; }

hw_layer/mass_storage/create_ini_image.sh            ./tunerstudio/generated/${INI} ./generated/ramdisk_image.h             128 ${SHORT_BOARDNAME} ${BOARD_SPECIFIC_URL}
hw_layer/mass_storage/create_ini_image_compressed.sh ./tunerstudio/generated/${INI} ./generated/ramdisk_image_compressed.h 1088 ${SHORT_BOARDNAME} ${BOARD_SPECIFIC_URL}

echo "Happy ${SHORT_BOARDNAME}!"
exit 0
