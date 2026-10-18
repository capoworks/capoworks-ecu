#!/usr/bin/env bash

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
readonly FW_DIR="${REPO_DIR}/firmware"
readonly CORE8_DIR="${FW_DIR}/config/boards/core8"
readonly IMAGE="${FW_DIR}/deliver/fome.bin"
readonly ELF="${FW_DIR}/build/fome.elf"
readonly INI="${FW_DIR}/tunerstudio/generated/fome_core8.ini"
readonly EXPECTED_DEVICE_ID="0x0413"
# Keep this directory aligned with firmware/use_arm_gcc.mk.
readonly ARM_TOOLCHAIN_BIN="${FW_DIR}/ext/build-tools/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin"
readonly BINUTILS_ALIAS_DIR="${REPO_DIR}/build/core8-binutils"

die() {
	printf 'ERROR: %s\n' "$*" >&2
	exit 1
}

setup_bundled_binutils() {
	local tool tool_path

	for tool in objcopy objdump; do
		tool_path="${ARM_TOOLCHAIN_BIN}/arm-none-eabi-${tool}"
		[[ -x "${tool_path}" ]] || die "Bundled ARM tool not found: ${tool_path}"
	done

	mkdir -p "${BINUTILS_ALIAS_DIR}"
	for tool in objcopy objdump; do
		tool_path="${ARM_TOOLCHAIN_BIN}/arm-none-eabi-${tool}"
		ln -sf "${tool_path}" "${BINUTILS_ALIAS_DIR}/${tool}"
		[[ -x "${BINUTILS_ALIAS_DIR}/${tool}" ]] || \
			die "Binutils alias was not created: ${BINUTILS_ALIAS_DIR}/${tool}"
	done

	PATH="${BINUTILS_ALIAS_DIR}:${PATH}"
	export PATH
}

find_programmer() {
	local candidate

	if [[ -n "${STM32_PROGRAMMER_CLI:-}" ]]; then
		candidate="${STM32_PROGRAMMER_CLI}"
		if [[ -f "${candidate}" ]]; then
			printf '%s\n' "${candidate}"
			return 0
		fi
		if candidate="$(command -v -- "${candidate}" 2>/dev/null)"; then
			printf '%s\n' "${candidate}"
			return 0
		fi
		die "STM32_PROGRAMMER_CLI does not name an executable: ${STM32_PROGRAMMER_CLI}"
	fi

	for candidate in STM32_Programmer_CLI STM32_Programmer_CLI.exe; do
		if candidate="$(command -v -- "${candidate}" 2>/dev/null)"; then
			printf '%s\n' "${candidate}"
			return 0
		fi
	done

	for candidate in \
		"${REPO_DIR}/misc/install/STM32_Programmer_CLI/bin/STM32_Programmer_CLI" \
		"${REPO_DIR}/misc/install/STM32_Programmer_CLI/bin/STM32_Programmer_CLI.exe"; do
		if [[ -f "${candidate}" ]]; then
			printf '%s\n' "${candidate}"
			return 0
		fi
	done

	die "STM32_Programmer_CLI was not found. Add it to PATH, install it under misc/install/STM32_Programmer_CLI, or set STM32_PROGRAMMER_CLI."
}

build_core8() {
	printf '\n== Building FOME Core8 ==\n'

	# Board builds share outputs. Clear firmware and bootloader products so
	# objects, PCH, and deliverables from another target cannot be reused.
	make -C "${FW_DIR}" clean \
		PROJECT_BOARD=core8 \
		PROJECT_CPU=ARCH_STM32F4 \
		BOARD_DIR="${CORE8_DIR}"
	make -C "${FW_DIR}/bootloader" clean \
		PROJECT_BOARD=core8 \
		PROJECT_CPU=ARCH_STM32F4 \
		BOARD_DIR="${CORE8_DIR}"

	# A board-specific clean does not remove shared PCH files used by GCC.
	if [[ -d "${FW_DIR}/pch/pch.h.gch" ]]; then
		find "${FW_DIR}/pch/pch.h.gch" -maxdepth 1 -type f -print -delete
	fi

	setup_bundled_binutils
	(
		cd "${CORE8_DIR}"
		bash compile_core8.sh
	)

	[[ -s "${IMAGE}" ]] || die "Image was not generated: ${IMAGE}"
	[[ -s "${ELF}" ]] || die "ELF was not generated: ${ELF}"
	[[ -s "${INI}" ]] || die "INI was not generated: ${INI}"

	printf '\nBuild complete.\n'
	printf 'Firmware: %s\n' "${IMAGE}"
	printf 'SHA-256: '
	sha256sum "${IMAGE}" | awk '{print $1}'
	verify_artifact_signatures
}

ini_signature() {
	awk -F '"' '/^[[:space:]]*signature[[:space:]]*=/{print $2; exit}' "${INI}"
}

firmware_signature() {
	strings "${ELF}" | awk '/^rusEFI \(FOME\) /{signature=$0} END{print signature}'
}

image_signature() {
	strings -a "${IMAGE}" | awk '/^rusEFI \(FOME\) /{signature=$0} END{print signature}'
}

verify_artifact_signatures() {
	local ini_sig firmware_sig image_sig
	ini_sig="$(ini_signature)"
	firmware_sig="$(firmware_signature)"
	image_sig="$(image_signature)"

	[[ -n "${ini_sig}" ]] || die "Signature not found in ${INI}."
	[[ -n "${firmware_sig}" ]] || die "Signature not found in ${ELF}."
	[[ -n "${image_sig}" ]] || die "Signature not found in ${IMAGE}."
	[[ "${firmware_sig}" == *".core8."* ]] || \
		die "Generated firmware is not Core8: ${firmware_sig}"
	[[ "${ini_sig}" == "${firmware_sig}" ]] || \
		die "Firmware and INI signatures do not match: '${firmware_sig}' != '${ini_sig}'."
	[[ "${image_sig}" == "${firmware_sig}" ]] || \
		die "Image and firmware signatures do not match: '${image_sig}' != '${firmware_sig}'."
	[[ "${image_sig}" == "${ini_sig}" ]] || \
		die "Image and INI signatures do not match: '${image_sig}' != '${ini_sig}'."

	printf 'Signature: %s\n' "${firmware_sig}"
}

flash_core8() {
	[[ -s "${IMAGE}" ]] || die "${IMAGE} does not exist. Run the build task first."
	[[ -s "${ELF}" ]] || die "${ELF} does not exist. Run the build task first."
	[[ -s "${INI}" ]] || die "${INI} does not exist. Run the build task first."
	verify_artifact_signatures

	local programmer
	programmer="$(find_programmer)"

	printf '\n== Detecting DFU device ==\n'
	local devices
	devices="$("${programmer}" -l usb)"
	printf '%s\n' "${devices}"

	local count
	count="$(awk '/Device Index/{count++} END{print count+0}' <<<"${devices}")"
	[[ "${count}" -eq 1 ]] || die "Expected exactly one DFU device, found: ${count}."

	local port serial device_id
	port="$(awk -F: '/Device Index/{gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${devices}")"
	serial="$(awk -F: '/Serial number/{gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${devices}")"
	device_id="$(awk -F: '/Device ID/{gsub(/[[:space:]]/, "", $2); print tolower($2); exit}' <<<"${devices}")"

	[[ -n "${port}" ]] || die "DFU port was not detected."
	[[ -n "${serial}" ]] || die "DFU serial number was not detected."
	[[ "${device_id}" == "${EXPECTED_DEVICE_ID}" ]] || \
		die "Device ID ${device_id:-unknown}; Core8 requires ${EXPECTED_DEVICE_ID}."

	local image_arg="${IMAGE}"
	if [[ "${programmer}" == *.exe ]]; then
		command -v wslpath >/dev/null 2>&1 || die "wslpath is required to invoke a Windows programmer from WSL."
		image_arg="$(wslpath -w "${IMAGE}")"
	fi

	printf '\n== Writing and verifying Core8 firmware ==\n'
	printf 'ECU:      %s (%s, %s)\n' "${serial}" "${port}" "${device_id}"
	printf 'Image:    %s\n' "${IMAGE}"

	"${programmer}" -q -c "port=${port}" "sn=${serial}" \
		-w "${image_arg}" 0x08000000 -v

	printf '\n== Starting firmware ==\n'
	"${programmer}" -q -c "port=${port}" "sn=${serial}" -g 0x08000000

	local remaining
	remaining="$("${programmer}" -l usb)"
	count="$(awk '/Device Index/{count++} END{print count+0}' <<<"${remaining}")"
	[[ "${count}" -eq 0 ]] || die "The ECU is still in DFU mode."

	printf '\nFirmware verified and started; the ECU left DFU mode.\n'
}

show_ini() {
	[[ -s "${INI}" ]] || die "${INI} does not exist. Run the build task first."
	[[ -s "${ELF}" ]] || die "${ELF} does not exist. Run the build task first."
	printf 'TunerStudio INI:\n%s\n' "${INI}"
	if command -v wslpath >/dev/null 2>&1; then
		wslpath -w "${INI}"
	fi
	verify_artifact_signatures
}

if [[ "$#" -gt 1 ]]; then
	die "Usage: $0 [build|flash|ini]"
fi

case "${1:-build}" in
	build)
		build_core8
		;;
	flash)
		flash_core8
		;;
	ini)
		show_ini
		;;
	*)
		die "Usage: $0 [build|flash|ini]"
		;;
esac
