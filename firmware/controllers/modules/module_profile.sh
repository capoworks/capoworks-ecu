#!/bin/bash

set -euo pipefail

die() {
	printf 'ERROR: %s\n' "$*" >&2
	exit 1
}

check_profile() {
	local profile=$1
	local line field value
	local have_modules=0
	local have_board=0
	local have_cpu=0

	[[ -f "$profile" ]] || die "module profile not found: $profile; run 'make module-profile' from a build directory"

	while IFS= read -r line || [[ -n "$line" ]]; do
		[[ "$line" == *" "* ]] || die "malformed module profile line in $profile: $line"
		field=${line%% *}
		value=${line#* }
		case "$field" in
			modules)
				have_modules=1
				;;
			board)
				[[ -n "$value" ]] || die "empty board in module profile: $profile"
				have_board=1
				;;
			cpu)
				[[ -n "$value" ]] || die "empty CPU in module profile: $profile"
				have_cpu=1
				;;
			config|output|ini)
				[[ -f "$value" ]] || die "module fragment not found: $value (from $profile)"
				;;
			*)
				die "unknown field '$field' in module profile: $profile"
				;;
		esac
	done < "$profile"

	(( have_modules && have_board && have_cpu )) || die "incomplete module profile: $profile"
}

write_profile() {
	[[ $# -ge 4 ]] || die "usage: $0 write <profile> <board> <cpu> <modules> [config|output|ini <path>]..."

	local profile=$1
	local board=$2
	local cpu=$3
	local modules=$4
	local kind path tmp
	shift 4

	[[ -n "$profile" ]] || die "module profile path is empty"
	[[ -n "$board" ]] || die "module profile board is empty"
	[[ -n "$cpu" ]] || die "module profile CPU is empty"

	mkdir -p "$(dirname "$profile")"
	tmp="${profile}.tmp.$$"
	trap 'rm -f "$tmp"' EXIT

	{
		printf 'modules %s\n' "$modules"
		printf 'board %s\n' "$board"
		printf 'cpu %s\n' "$cpu"

		while (( $# > 0 )); do
			(( $# >= 2 )) || die "missing path for module fragment '$1'"
			kind=$1
			path=$2
			shift 2
			case "$kind" in
				config|output|ini)
					[[ -f "$path" ]] || die "module fragment not found: $path"
					printf '%s %s\n' "$kind" "$path"
					;;
				*)
					die "unknown module fragment kind: $kind"
					;;
			esac
		done
	} > "$tmp"

	chmod 0644 "$tmp"
	if [[ -f "$profile" ]] && cmp -s "$tmp" "$profile"; then
		rm -f "$tmp"
	else
		mv -f "$tmp" "$profile"
	fi
	trap - EXIT
}

read_profile() {
	[[ $# -eq 2 ]] || die "usage: $0 read <profile> <modules|board|cpu|config|output|ini>"

	local profile=$1
	local wanted=$2
	local line field value

	check_profile "$profile"
	while IFS= read -r line || [[ -n "$line" ]]; do
		field=${line%% *}
		value=${line#* }
		if [[ "$field" == "$wanted" ]]; then
			printf '%s\n' "$value"
		fi
	done < "$profile"
}

mode=${1:-}
case "$mode" in
	write)
		shift
		write_profile "$@"
		;;
	check)
		[[ $# -eq 2 ]] || die "usage: $0 check <profile>"
		check_profile "$2"
		;;
	read)
		shift
		read_profile "$@"
		;;
	*)
		die "usage: $0 <write|check|read> ..."
		;;
esac
