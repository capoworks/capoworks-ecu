#!/usr/bin/env bash
# Run the same checks locally and on GitHub Actions.
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
target=${1:-all}
jobs=${JOBS:-12}

usage() {
    echo "Usage: $0 [all|unit-tests|simulator|core8]"
    echo "JOBS controls host build parallelism (default: 12)."
    echo "Logs and reports: build/capoworks-checks/<target>/"
}

case "$target" in
    -h|--help) usage; exit 0 ;;
    all|unit-tests|simulator|core8) ;;
    *) usage >&2; exit 2 ;;
esac
if [[ $# -gt 1 || ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
    usage >&2
    exit 2
fi

if [[ "$target" == all ]]; then
    # Builds share generated headers and PCH files: run sequentially.
    for check in unit-tests simulator core8; do
        bash "$repo_root/misc/ci/capoworks-checks.sh" "$check"
    done
    exit 0
fi

output_dir="$repo_root/build/capoworks-checks/$target"
mkdir -p "$output_dir"
# A failed rerun must not leave successful reports from an earlier run.
rm -f "$output_dir"/*.xml "$output_dir/status.txt"

run_check() (
    set -euo pipefail
    cd "$repo_root"
    git rev-parse HEAD
    # Each build has its own clean target, but all of them search this PCH folder.
    # Do not run this harness alongside another build in the same worktree.
    rm -f firmware/pch/pch.h.gch/*

    case "$target" in
        unit-tests)
            cd unit_tests
            make clean
            export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
            make -j"$jobs" all SANITIZE=yes
            # The second build resolves generated dependency ordering.
            make -j"$jobs" all SANITIZE=yes
            ./build/fome_test --gtest_output="xml:$output_dir/tests.xml"
            test -s "$output_dir/tests.xml"
            python3 - "$output_dir/tests.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
failures = int(root.attrib["failures"])
errors = int(root.attrib["errors"])
if failures or errors:
    raise SystemExit(
        f"GoogleTest report has {failures} failures and {errors} errors"
    )
PY
            ;;
        simulator)
            cd simulator
            make clean
            make -j"$jobs"
            timeout 60s ./build/fome_simulator 10
            ;;
        core8)
            # The checksum script calls unprefixed objcopy/objdump. Ubuntu's host
            # binutils do not read ARM ELFs; use the firmware's bundled tools.
            # Keep this directory aligned with firmware/use_arm_gcc.mk.
            toolchain_dir="$repo_root/firmware/ext/build-tools/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi/bin"
            binutils_dir="$repo_root/build/capoworks-tools"
            mkdir -p "$binutils_dir"
            for tool in objcopy objdump; do
                test -x "$toolchain_dir/arm-none-eabi-$tool"
                ln -sf "$toolchain_dir/arm-none-eabi-$tool" "$binutils_dir/$tool"
            done
            export PATH="$binutils_dir:$PATH"
            board_dir="$repo_root/firmware/config/boards/core8"
            make -C firmware clean PROJECT_BOARD=core8 PROJECT_CPU=ARCH_STM32F4 BOARD_DIR="$board_dir"
            make -C firmware/bootloader clean PROJECT_BOARD=core8 PROJECT_CPU=ARCH_STM32F4 BOARD_DIR="$board_dir"
            cd "$board_dir"
            bash compile_core8.sh
            cd "$repo_root/firmware"
            test -s build/fome.list
            bash check_illegal_conversion.sh
            for artifact in fome.bin fome_update.srec fome_bl.srec; do
                test -s "deliver/$artifact"
            done
            test -s tunerstudio/generated/fome_core8.ini
            ;;
    esac
)

# Keep errexit active inside run_check and preserve failure through tee.
set +e
run_check 2>&1 | tee "$output_dir/check.log"
status=$?
set -e
echo "$status" > "$output_dir/status.txt"
exit "$status"
