# Cylinder model fuel trims

Per-cylinder fuel trims now have two independent 16x16 families:

- Speed-density trims use RPM and effective MAP (kPa).
- Alpha-N trims use RPM and TPS (%).

Each family has its own shared axes and one table per configured cylinder. In
blended Speed Density + Alpha-N mode, both active families contribute according
to their captured air-mass weights. A Speed-Density-only or Alpha-N-only mode
uses only its own family. MAF, Lua and mock air-mass modes do not use these
trims.

## Core8 memory tradeoff

Core8 keeps the expanded cylinder-model calibration, a 5 KiB (5120-byte) main
working stack, and full SD card/FATFS and USB mass-storage (MSD) support. To fit
that configuration in RAM, its Lua heap is 26,000 bytes instead of the
30,000-byte F407 default. Other boards keep their existing Lua heap size.

The factor scales only modeled fuel. Coolant, intake-air, barometric,
after-start and Lua multiplier corrections can therefore move the trimmed
modeled fuel. Acceleration enrichment, the Lua fuel offset, injector dead time
and fixed cranking fuel remain outside the factor. DFCO and disabled injection
still produce zero fuel.

## Migrating old trims

The old 4x4 fuel-trim tables are not copied automatically. Record or screenshot
the old per-cylinder values before upgrading, then recreate them in the
appropriate family table at the desired RPM/load points. For a blended tune,
enter the intended Speed-Density and Alpha-N behavior separately.

## Blended-mode authority

The trim family is selected from the effective authority after branch fallback,
not from the requested blend alone. If the blend authority table, its load
source, or its breakpoints are changed, recheck and retune the MAP authority
breakpoints so the intended branch is active at idle, cruise and transitional
loads. Then verify the resulting per-cylinder trims under steady-state load.
