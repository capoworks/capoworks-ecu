#!/bin/bash

set -euo pipefail

rm -f gen_live_documentation.log

STAMP_FILE=${1:-}
PROFILE_FILE=${2:-}
PROFILE_READER=controllers/modules/module_profile.sh
PROFILE_MAKE=controllers/modules/module_profile_host.mk

if [ -z "$PROFILE_FILE" ]; then
	PROFILE_FILE=generated/module_profile.txt
	make --no-print-directory -f "$PROFILE_MAKE" module-profile \
		PROJECT_DIR="$PWD" \
		BOARD_DIR=config/boards/f407-discovery
fi

"$PROFILE_READER" check "$PROFILE_FILE"

SELECTED_MODULES=$("$PROFILE_READER" read "$PROFILE_FILE" modules)

OUTPUT_FRAGMENTS=()
while IFS= read -r fragment; do
	OUTPUT_FRAGMENTS+=("$fragment")
done < <("$PROFILE_READER" read "$PROFILE_FILE" output)

FRAGMENT_ARGS=()
for fragment in "${OUTPUT_FRAGMENTS[@]}"; do
	FRAGMENT_ARGS+=(-outputFragment "$fragment")
done

STAMP_ARGS=()
if [ -n "$STAMP_FILE" ]; then
	STAMP_ARGS=(-stampFile "$STAMP_FILE")
fi

java -DSystemOut.name=logs/gen_live_documentation \
 -cp ../java_tools/ConfigDefinition.jar \
 com.rusefi.ldmp.LiveDataProcessor integration/LiveData.yaml \
 -makefileDep .dep/fome_live_data.d \
 -modules "$SELECTED_MODULES" \
 "${FRAGMENT_ARGS[@]}" \
 "${STAMP_ARGS[@]}"

java -DSystemOut.name=logs/gen_java_enum -cp ../java_tools/enum2string.jar com.rusefi.ToJavaEnum -enumInputFile generated/live_data_ids.h -outputPath ../java_console/io/src/main/java/com/rusefi/enums

# live_data_ids.h is generated above, so order of operations is important here!
./gen_enum_to_string.sh
