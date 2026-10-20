#!/bin/bash

set -euo pipefail

die() {
	printf 'ERROR: %s\n' "$*" >&2
	exit 1
}

extract_dispatch_symbols() {
	local source=$1

	awk '
		/switch \(engineType\)/ {
			in_switch = 1
		}
		in_switch {
			line = $0
			while (match(line, /engine_type_e::[A-Za-z0-9_]+/)) {
				symbol = substr(line, RSTART + 15, RLENGTH - 15)
				if (!seen[symbol]++) {
					symbols[++count] = symbol
				}
				line = substr(line, RSTART + RLENGTH)
			}
			if ($0 ~ /default:/) {
				exit
			}
		}
		END {
			for (i = 1; i <= count; i++) {
				print symbols[i]
			}
		}
	' "$source"
}

extract_enum_symbols() {
	local source=$1

	awk '
		/enum class engine_type_e/ {
			in_enum = 1
			next
		}
		in_enum && /^};/ {
			exit
		}
		in_enum {
			line = $0
			sub(/\/\/.*/, "", line)
			sub(/\/\*.*/, "", line)
			if (match(line, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*=/)) {
				symbol = substr(line, RSTART, RLENGTH)
				sub(/[[:space:]]*=.*/, "", symbol)
				if (!seen[symbol]++) {
					print symbol
				}
			}
		}
	' "$source"
}

contains_word() {
	local needle=$1
	shift
	local word

	for word in "$@"; do
		if [[ "$word" == "$needle" ]]; then
			return 0
		fi
	done
	return 1
}

write_if_changed() {
	local destination=$1
	local content=$2
	local temporary

	temporary="${destination}.tmp.$$"
	mkdir -p "$(dirname "$destination")"
	printf '%s\n' "$content" > "$temporary"
	chmod 0644 "$temporary"
	if [[ -f "$destination" ]] && cmp -s "$temporary" "$destination"; then
		rm -f "$temporary"
	else
		mv -f "$temporary" "$destination"
	fi
}

render() {
	local header=
	local metadata=
	local ts_definitions=
	local dispatch_source=
	local enum_source=
	local selection_file=
	local default_file=
	local selection_origin=
	local ve_rpm=
	local blend_size=
	local symbol
	local token
	local mode
	local expanded
	local default_symbol
	local default_enabled
	local -a supported=()
	local -a enum_symbols=()
	local -a requested=()
	local -a selected=()
	local -a unknown=()
	local -a unsupported=()
	local -a incompatible=()
	local selected_text=
	local header_text
	local metadata_text
	local ts_text

	while (( $# > 0 )); do
		case "$1" in
			--header)
				(( $# >= 2 )) || die "missing value for --header"
				header=$2
				shift 2
				;;
			--metadata)
				(( $# >= 2 )) || die "missing value for --metadata"
				metadata=$2
				shift 2
				;;
			--ts-definitions)
				(( $# >= 2 )) || die "missing value for --ts-definitions"
				ts_definitions=$2
				shift 2
				;;
			--dispatch-source)
				(( $# >= 2 )) || die "missing value for --dispatch-source"
				dispatch_source=$2
				shift 2
				;;
			--enum-source)
				(( $# >= 2 )) || die "missing value for --enum-source"
				enum_source=$2
				shift 2
				;;
			--selection-file)
				(( $# >= 2 )) || die "missing value for --selection-file"
				selection_file=$2
				shift 2
				;;
			--selection-origin)
				(( $# >= 2 )) || die "missing value for --selection-origin"
				selection_origin=$2
				shift 2
				;;
			--default-file)
				(( $# >= 2 )) || die "missing value for --default-file"
				default_file=$2
				shift 2
				;;
			--ve-rpm)
				(( $# >= 2 )) || die "missing value for --ve-rpm"
				ve_rpm=$2
				shift 2
				;;
			--blend-size)
				(( $# >= 2 )) || die "missing value for --blend-size"
				blend_size=$2
				shift 2
				;;
			*)
				die "unknown engine preset renderer argument: $1"
				;;
		esac
	done

	[[ -n "$header" ]] || die "engine preset header path is empty"
	[[ -n "$metadata" ]] || die "engine preset metadata path is empty"
	[[ -n "$ts_definitions" ]] || die "engine preset TunerStudio definitions path is empty"
	[[ -f "$dispatch_source" ]] || die "engine preset dispatch source not found: $dispatch_source"
	[[ -f "$enum_source" ]] || die "engine type enum source not found: $enum_source"
	[[ -f "$selection_file" ]] || die "engine preset selection file not found: $selection_file"
	[[ -f "$default_file" ]] || die "engine preset default file not found: $default_file"

	case "$ve_rpm" in
		16|32)
			;;
		*)
			die "FOME_VE_RPM_COUNT_RESOLVED must be 16 or 32, got '$ve_rpm'"
			;;
	esac
	case "$blend_size" in
		8|16)
			;;
		*)
			die "FOME_AIRMASS_BLEND_SIZE_RESOLVED must be 8 or 16, got '$blend_size'"
			;;
	esac

	if [[ "$ve_rpm" == 16 && "$blend_size" == 8 ]]; then
		expanded=0
	else
		expanded=1
	fi

	mapfile -t supported < <(extract_dispatch_symbols "$dispatch_source")
	(( ${#supported[@]} > 0 )) || die "could not derive engine preset dispatch symbols from $dispatch_source"
	mapfile -t enum_symbols < <(extract_enum_symbols "$enum_source")
	(( ${#enum_symbols[@]} > 0 )) || die "could not derive engine_type_e symbols from $enum_source"

	# The catalog contains every switch case, including cases behind HW_* guards.
	# Make-time board rejection is not attempted here because this renderer does
	# not receive the board preprocessor defines. Runtime availability uses the
	# same guarded dispatch and rejects a missing board case before mutation.
	for symbol in "${supported[@]}"; do
		contains_word "$symbol" "${enum_symbols[@]}" ||
			die "dispatch symbol '$symbol' is not present in engine_type_e"
	done

	selection=$(cat "$selection_file")
	default_symbol=$(tr -d '[:space:]' < "$default_file")
	[[ -n "$default_symbol" ]] || default_symbol=MINIMAL_PINS
	[[ "$default_symbol" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] ||
		die "DEFAULT_ENGINE_TYPE must be an engine_type_e symbol, got '$default_symbol'"

	read -ra requested <<< "$selection"
	if [[ "$selection_origin" == undefined ]]; then
		if (( expanded )); then
			mode=default_none
		else
			mode=default_all
		fi
	else
		(( ${#requested[@]} > 0 )) ||
			die "FOME_ENGINE_PRESETS is explicitly empty; use 'none', 'all', or a whitespace-separated symbol list"

		if contains_word all "${requested[@]}"; then
			(( ${#requested[@]} == 1 )) ||
				die "FOME_ENGINE_PRESETS cannot mix 'all' with other values"
			mode=explicit_all
		elif contains_word none "${requested[@]}"; then
			(( ${#requested[@]} == 1 )) ||
				die "FOME_ENGINE_PRESETS cannot mix 'none' with other values"
			mode=explicit_none
		else
			mode=explicit
		fi
	fi

	if [[ "$mode" == default_all || "$mode" == explicit_all ]]; then
		selected=("${supported[@]}")
	elif [[ "$mode" == default_none || "$mode" == explicit_none ]]; then
		selected=(MINIMAL_PINS)
	else
		for token in "${requested[@]}"; do
			if contains_word "$token" "${supported[@]}"; then
				:
			elif contains_word "$token" "${enum_symbols[@]}"; then
				unsupported+=("$token")
			else
				unknown+=("$token")
			fi
		done
		(( ${#unknown[@]} == 0 )) ||
			die "unknown FOME_ENGINE_PRESETS value(s): ${unknown[*]}"
		(( ${#unsupported[@]} == 0 )) ||
			die "unsupported engine preset(s) with no engine_configuration.cpp dispatch: ${unsupported[*]}"

		for symbol in "${supported[@]}"; do
			if [[ "$symbol" == MINIMAL_PINS ]] || contains_word "$symbol" "${requested[@]}"; then
				selected+=("$symbol")
			fi
		done
	fi

	if (( expanded )); then
		for symbol in "${selected[@]}"; do
			if [[ "$symbol" != MINIMAL_PINS ]]; then
				incompatible+=("$symbol")
			fi
		done
		(( ${#incompatible[@]} == 0 )) ||
			die "expanded FOME_VE_RPM_COUNT_RESOLVED=$ve_rpm / FOME_AIRMASS_BLEND_SIZE_RESOLVED=$blend_size builds cannot include literal engine presets: ${incompatible[*]}"
	fi

	if [[ "$default_symbol" == MINIMAL_PINS ]] ||
		contains_word "$default_symbol" "${selected[@]}"; then
		default_enabled=1
	else
		default_enabled=0
	fi

	selected_text="${selected[*]-}"

	header_text='/* Generated by engine_preset_profile.sh. Do not edit. */
#pragma once

#define FOME_ENGINE_PRESET_PROFILE_CONFIGURED 1
#define FOME_ENGINE_PRESET_PROFILE_CAT_I(a, b) a##b
#define FOME_ENGINE_PRESET_PROFILE_CAT(a, b) FOME_ENGINE_PRESET_PROFILE_CAT_I(a, b)
#define FOME_ENGINE_PRESET_ENABLED(symbol) FOME_ENGINE_PRESET_PROFILE_CAT(FOME_ENGINE_PRESET_ENABLED_, symbol)'
	header_text+=$(printf '\n#define FOME_ENGINE_PRESETS_EXPANDED %s' "$expanded")
	header_text+=$(printf '\n#define FOME_ENGINE_PRESET_COUNT %s' "${#selected[@]}")
	header_text+=$(printf '\n#define FOME_ENGINE_PRESET_DEFAULT_ENGINE_TYPE_ENABLED %s' "$default_enabled")
	header_text+=$(printf '\n#define FOME_ENGINE_PRESET_DEFAULT_ENGINE_TYPE_SYMBOL %s\n' "$default_symbol")
	for symbol in "${supported[@]}"; do
		if contains_word "$symbol" "${selected[@]}"; then
			header_text+=$(printf '\n#define FOME_ENGINE_PRESET_ENABLED_%s 1' "$symbol")
		else
			header_text+=$(printf '\n#define FOME_ENGINE_PRESET_ENABLED_%s 0' "$symbol")
		fi
	done

	metadata_text='# Generated by engine_preset_profile.sh. Do not edit.'
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_MODE := %s' "$mode")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_SELECTED := %s' "$selected_text")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_EXPANDED := %s' "$expanded")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE := %s' "$default_symbol")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_DEFAULT_ENGINE_TYPE_ENABLED := %s' "$default_enabled")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_VE_RPM_COUNT := %s' "$ve_rpm")
	metadata_text+=$(printf '\nMODULE_ENGINE_PRESET_PROFILE_AIRMASS_BLEND_SIZE := %s' "$blend_size")

	ts_text='! Generated by engine_preset_profile.sh. Do not edit.'
	for symbol in "${supported[@]}"; do
		if contains_word "$symbol" "${selected[@]}"; then
			ts_text+=$(printf '\n#define show_engine_preset_%s true' "$symbol")
		else
			ts_text+=$(printf '\n#define show_engine_preset_%s false' "$symbol")
		fi
	done

	write_if_changed "$header" "$header_text"
	write_if_changed "$metadata" "$metadata_text"
	write_if_changed "$ts_definitions" "$ts_text"
	printf 'success\n'
}

mode=${1:-}
case "$mode" in
	render)
		shift
		render "$@"
		;;
	*)
		die "usage: $0 render --header <path> --metadata <path> --ts-definitions <path> --dispatch-source <path> --enum-source <path> --selection-file <path> --selection-origin <origin> --default-file <path> --ve-rpm <16|32> --blend-size <8|16>"
		;;
esac
