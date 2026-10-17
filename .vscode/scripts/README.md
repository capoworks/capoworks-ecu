# Core8 developer helper

Run `bash .vscode/scripts/core8_dfu.sh` with no arguments, or use the default
`Core8: Build firmware` VS Code task, to build Core8 only. The build clears
firmware and bootloader outputs, removes shared firmware PCH files under
`firmware/pch/pch.h.gch`, and replaces artifacts in `firmware/deliver/`.
It creates local `objcopy` and `objdump` aliases under the ignored
`build/core8-binutils/` directory and prepends them to `PATH`, so the checksum
script uses the bundled ARM binutils selected to match
`firmware/use_arm_gcc.mk` instead of unprefixed host tools.

Flashing is never implicit. Use the non-default `Core8: Flash firmware (USB DFU)`
task or the explicit `flash` argument after building. The helper requires
exactly one DFU device, a detected serial number and the Core8 device ID, and
requires matching ELF, INI and embedded `fome.bin` signatures. That combined
image is written at `0x08000000` and boots from the same address. Signature
checking completes before any programmer call. Do not run two board builds
concurrently because they share generated and build outputs.

The helper uses `STM32_PROGRAMMER_CLI` when set, then searches `PATH`, then
the repository copy under `misc/install/STM32_Programmer_CLI`. A system
STM32CubeProgrammer installation can instead be made available through `PATH`;
for vendor installs, put its `bin` directory on `PATH`.
