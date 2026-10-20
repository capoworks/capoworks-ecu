#!/bin/bash

set -euo pipefail

die() {
	printf 'ERROR: %s\n' "$*" >&2
	exit 1
}

[[ $# -eq 3 ]] || die "usage: $0 <count 0..12> <template> <output>"

count=$1
template=$2
output=$3

case "$count" in
	0|1|2|3|4|5|6|7|8|9|10|11|12)
		;;
	*)
		die "cylinder fuel trim render count must be an integer from 0 to 12, got '$count'"
		;;
esac

[[ -f "$template" ]] || die "cylinder fuel trim TunerStudio template not found: $template"
mkdir -p "$(dirname "$output")"
tmp="${output}.tmp.$$"
trap 'rm -f "$tmp"' EXIT

emit_tables() {
	local cylinder
	for ((cylinder = 1; cylinder <= count; cylinder++)); do
		cat <<EOF
	table = cylinderFuelTrimTbl${cylinder}, cylinderFuelTrimMap${cylinder}, "Cylinder fuel trim ${cylinder}", 1
		xyLabels = "RPM", {bitStringValue(loadAxisLabels, cylinderFuelTrimLoadSource${cylinder} == 0 ? modelDefaultLoadAxis : cylinderFuelTrimLoadSource${cylinder})}
		xBins = cylinderFuelTrimRpmBins, RPMValue
		yBins = cylinderFuelTrimLoadBins, cylinderFuelTrimLoad${cylinder}
		zBins = cylinderFuelTrims${cylinder}_table
		upDownLabel = "(RICHER)", "(LEANER)"
EOF
	done
}

emit_gauges() {
	local cylinder
	for ((cylinder = 1; cylinder <= count; cylinder++)); do
		printf '\tcylinderFuelTrimGauge%d = cylinderFuelTrim%d, "Cylinder fuel trim %d", "%%", -25, 25, 0, -25, 25, 25, 2, 2\n' \
			"$cylinder" "$cylinder" "$cylinder"
	done
}

emit_menu() {
	local cylinder
	for ((cylinder = 1; cylinder <= count; cylinder++)); do
		cat <<EOF
			groupChildMenu = cylinderFuelTrimTbl${cylinder}, "Cylinder fuel trim ${cylinder}", 0, { isInjectionEnabled && cylindersCount >= ${cylinder} }
EOF
	done
}

emit_sources() {
	local cylinder
	for ((cylinder = 1; cylinder <= count; cylinder++)); do
		cat <<EOF
		field = "Cylinder ${cylinder}", cylinderFuelTrimLoadSource${cylinder}, { cylindersCount >= ${cylinder} }
EOF
	done
}

if [[ "$count" == 0 ]]; then
	: > "$tmp"
else
	while IFS= read -r line || [[ -n "$line" ]]; do
		case "$line" in
			"; @CYLINDER_FUEL_TRIM_TABLES@")
				emit_tables
				;;
			"; @CYLINDER_FUEL_TRIM_GAUGES@")
				emit_gauges
				;;
			"; @CYLINDER_FUEL_TRIM_MENU@")
				emit_menu
				;;
			"; @CYLINDER_FUEL_TRIM_SOURCES@")
				emit_sources
				;;
			*)
				printf '%s\n' "$line"
				;;
		esac
	done < "$template" > "$tmp"
fi

chmod 0644 "$tmp"
if [[ -f "$output" ]] && cmp -s "$tmp" "$output"; then
	rm -f "$tmp"
else
	mv -f "$tmp" "$output"
fi
trap - EXIT
