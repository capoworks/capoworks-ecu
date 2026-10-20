# Build-time module profiles

Firmware modules are selected at compile time. `firmware/controllers/modules/module_profile.mk` is the single canonical catalog and profile resolver; there is no runtime module registry, tune setting, or new configuration option for choosing modules.

## Catalog and selection

The canonical catalog is:

```text
fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump shock_preload cylinder_fuel_trim
```

Set `FOME_MODULES` to a whitespace-separated list of these exact, lowercase tokens. It replaces the default profile rather than adding to it. Unknown tokens fail during Make configuration, and selections are deduplicated and resolved in catalog order. An explicitly empty value remains empty. The existing `NO_MAP_AVERAGING` switch still removes `map_averaging`.

An omitted value selects:

```text
fan fuel_pump gear_detector tachometer trip_odometer map_averaging boost engine_sniffer lua cylinder_fuel_trim
```

On `hellen128`, `lua` is removed from that default because this board does not support Lua. Non-`hellen128` boards retain `lua` in their default profile. `cylinder_fuel_trim` is included with a default capacity of 6 on every board. `hella_opst`, `emp_pump`, and `shock_preload` are omitted from the default profile on every board, but remain available for explicit selection. The catalog above lists every token recognized by the resolver; it is not the default profile.

Keep `fan fuel_pump gear_detector tachometer trip_odometer map_averaging` in every supported profile. Build availability and default inclusion are:

| Feature | Token | Included by default |
|---|---|---|
| Hella OPS-T | `hella_opst` | No |
| Boost | `boost` | Yes |
| Engine Sniffer | `engine_sniffer` | Yes |
| Lua | `lua` | Yes on non-`hellen128` boards |
| EMP coolant pump | `emp_pump` (requires `fan`) | No |
| Shock preload | `shock_preload` (CAN2, requires CAN read and write) | No |
| Cylinder fuel trim | `cylinder_fuel_trim` | Yes, with capacity 6 |

An explicitly selected profile that includes `cylinder_fuel_trim` uses a capacity of 6 unless overridden:

```bash
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging cylinder_fuel_trim" \
bash compile_core8.sh
```

`FOME_CYLINDER_TRIM_COUNT` is optional when the module is selected and defaults to 6. An explicit command-line or environment value must be an integer from 1 through 12; values such as 0, 13, or an empty value fail during Make configuration. The count sets the number of cylinder-trim tables. Whenever injection is enabled, it must cover the engine's configured cylinder count: an 8- or 12-cylinder engine must explicitly set `FOME_CYLINDER_TRIM_COUNT` to at least its cylinder count, or omit `cylinder_fuel_trim` from `FOME_MODULES`. The default count of 6 is only the cylinder-trim table capacity; it does not reduce or otherwise override the global `MAX_CYLINDER_COUNT` of 12. Because `FOME_MODULES` replaces the default profile, omitting `cylinder_fuel_trim` disables the module, records a count of zero even if a stale `FOME_CYLINDER_TRIM_COUNT` value is present, and generates no cylinder-trim storage, output channels, or TunerStudio UI.

Native unit tests default `FOME_CYLINDER_TRIM_COUNT` to 12 for fixtures covering engines up to 12 cylinders; explicitly testing 6 cylinders should use focused tests. This test-only default does not change the firmware default of 6.

Other legacy modules have not been newly audited for removal. Explicitly selecting `lua` on `hellen128` is an error rather than a silent omission. `emp_pump` reads both fan-controller states for ram-air relief, so selecting it without `fan` is rejected during Make configuration.

To select the full 13-token catalog on Core8, which is not `hellen128`, spell out the complete profile. This example also overrides the cylinder-trim capacity to 12:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump shock_preload cylinder_fuel_trim" \
FOME_CYLINDER_TRIM_COUNT=12 \
bash compile_core8.sh
```

A base-only Core8 profile is:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging" bash compile_core8.sh
```

## Fuel table dimensions

Two optional build-profile values control fuel calibration table sizes:

| Option | Values | Default | Effect |
|---|---|---|---|
| `FOME_VE_RPM_COUNT` | `16`, `32` | `16` | RPM points in both the Speed Density VE table and the Alpha-N table |
| `FOME_AIRMASS_BLEND_SIZE` | `8`, `16` | `8` | Both axes of the SD + Alpha-N authority table |

The VE tables retain 16 load points. Neither option changes load-table quantization or table value packing. `FOME_VE_RPM_COUNT` uses a dedicated VE RPM dimension. The shared `FUEL_RPM_COUNT` remains 16, so lambda, MAP-estimate, and injection-phase tables do not grow with the VE table.

The resolver validates both values during Make configuration. Omitted values use the defaults, while explicitly empty or unsupported values fail configuration:

```bash
FOME_VE_RPM_COUNT=32 FOME_AIRMASS_BLEND_SIZE=16 \
  FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging" \
  bash compile_core8.sh
```

The resolved values are recorded as `ve_rpm_count` and `airmass_blend_size` in `firmware/generated/module_profile.txt`. `module_profile.sh` renders them into `module_profile_defines.h`; both generators prepend that file, and the resulting `firmware/generated/rusefi_generated.h` is authoritative. These dimensions are not compiler command-line definitions.

The 16-point defaults preserve the existing VE and blend axes. A 32-point VE axis explicitly spans 650 through 7000 RPM monotonically for both VE maps, rather than inheriting the low-RPM bunching of the generic 32-point RPM-bin helper. A 16-point blend uses monotonic load defaults ending at 100% TPS and RPM defaults ending at 18000 RPM.

## Engine presets

`FOME_ENGINE_PRESETS` selects which engine-specific preset dispatch code is available in a firmware build. It accepts `all`, `none`, or a whitespace-separated list of exact `engine_type_e` symbol names without the `engine_type_e::` prefix, for example:

```bash
cd firmware/config/boards/proteus
FOME_ENGINE_PRESETS="PROTEUS_BMW_M73" FOME_CYLINDER_TRIM_COUNT=12 bash compile_proteus_f7.sh
```

An explicitly empty value, an unknown symbol, or mixing `all` or `none` with other tokens fails during Make configuration.

With the default 16 RPM-point VE and 8-point blend dimensions, omitting `FOME_ENGINE_PRESETS` keeps the legacy behavior and includes all presets. Omitting it in any expanded-dimension build includes no engine-specific presets. `none` is valid in every build. An explicit symbol list, including `all`, is rejected when either table dimension is expanded; use `none` for those builds.

The canonical `firmware/generated/module_profile.txt` records the resolved `engine_preset_mode`, `engine_presets` list, and `engine_preset_default`. The board generator consumes the matching generated per-preset visibility definitions, so changing the selection regenerates the configuration, embedded INI, and objects without requiring a clean build. Profiles created before these fields existed remain readable with the legacy defaults above.

Selection is a build-time filter only; there is no runtime preset registry. It does not remove entries from the persisted `engineType` enum, and enumerator numeric IDs and labels remain unchanged. Changing only `FOME_ENGINE_PRESETS` therefore does not change the saved-tune layout. Preset action buttons still require the existing board and board-family visibility conditions in addition to the per-preset condition, so a selected preset may remain inapplicable to the current board. The reset-to-default command remains available independently of preset selection.

On `hellen128`, for example, explicitly adding `lua` to a profile fails during Make configuration with:

```text
Board hellen128 does not support FOME_MODULES module(s): lua. Remove them from FOME_MODULES or select a supported board
```

`compile_core8.sh` builds and packages the firmware; it does not flash it. Flashing is always a separate, explicit action.

Build one profile at a time in a checkout because generated outputs and module headers are shared. When first crossing the source change that moved `value_lookup.cpp` and `value_lookup.h` from `controllers/lua` to `controllers/core`, clean both targets once, sequentially, from the repository root:

```bash
make -C firmware clean PROJECT_BOARD=core8 PROJECT_CPU=ARCH_STM32F4 BOARD_DIR="$PWD/firmware/config/boards/core8"
make -C firmware/bootloader clean PROJECT_BOARD=core8 PROJECT_CPU=ARCH_STM32F4 BOARD_DIR="$PWD/firmware/config/boards/core8"
```

The explicit arguments prevent bare `make clean` from defaulting to another board. After that initial cleanup, switching `FOME_MODULES` profiles without cleaning is supported. Do not run cleans or builds concurrently.

## Module-owned inputs

Each module may provide four conventional fragments, without another registry:

| File | Purpose |
|---|---|
| `config.txt` | Fields inserted at `! MODULE_CONFIG` into the active `engine_configuration_s`. |
| `persistent_config.txt` | Calibration fields inserted at `! MODULE_PERSISTENT_CONFIG` into root `persistent_config_s`, avoiding another active configuration copy. |
| `output_channels.txt` | Live output channels inserted at `! MODULE_OUTPUTS`. |
| `tunerstudio.ini` | TunerStudio sections inserted into named contexts. |

TunerStudio fragments use section names such as `[Menu.Advanced]`; the generator places each section body at the matching `; MODULE_INI Menu.Advanced` marker in the template. A module's `.mk` owns its source files, include paths, defines, and generated module tuples. Shared code adds ownership guards only where an external call would otherwise reference module-owned code or types; broad per-function or hot-path guards are not a module pattern.

The EMP pump owns its CAN protocol, mailbox and queue state machine, configuration block and tables, output channels, TunerStudio contexts, defaults, and unit test. Shared code retains only guarded integration seams: `setDefaultEmpPump()` at base-default setup, EMP CAN polling on the selected bus's existing TX thread, status output-channel assignments, and TunerStudio service-command dispatch. The CAN listener bus-aware decode bridge is compiled only with `emp_pump`; without it, listeners retain the original single decode dispatch. Runtime `enableEmpPump` remains false by default.

The Shock preload module owns its CAN2 protocol, runtime state, configuration, output channels, TunerStudio contexts, and unit test. Shared code retains only guarded integration seams for status polling on the existing CAN TX thread, telemetry assignments, and TunerStudio service-command dispatch. It uses standard IDs `0x720`-`0x722` on CAN2 and must be explicitly selected at build time; it is disabled at runtime by zero-initialized `enableShockPreload`. CAN read and write must also be enabled.

Cylinder fuel trim owns one persistent 16x16 factor table per configured cylinder, two shared axis arrays, per-cylinder load-source selectors, output channels, and a materialized TunerStudio fragment. The module does not create separate Speed Density and Alpha-N tables: each cylinder selects one of all six load coordinates (`Model default`, measured MAP, TPS, accelerator pedal, cylinder filling, or effective MAP), and the table remains a single factor table. Because every cylinder shares the same load-bin and RPM-bin arrays, a profile whose cylinders use sources with different units must use common breakpoints appropriate for the selected mix; changing a source does not retune those breakpoints.
The shared load axis preserves fractional TPS breakpoints at 0.1 resolution without additional storage.
Cylinder fuel trim does not persist a factor cache. Published telemetry is quantized to 0.01 percent trim and 0.1 load for binary transport, while the captured load and factor used by control retain full precision.

`integration/LiveData.yaml` owns live-data usage declarations and marks module-specific usages with `module:`. YAML order fixes `LDS_<usage>` IDs, so an unselected module keeps its declaration and IDs while its fragment and packed output usage disappear. Do not reorder or duplicate usages just to change a profile.

## Resource budgets

These are compile-time defaults and placements, not calculated free-RAM margins. Board-specific `LUA_USER_HEAP` overrides can change the heap figures.

- The default Lua heap is 20,000 bytes on F407, 60,000 bytes on non-Ethernet F429, and 25,000 bytes on Ethernet F429. The F7 and H7 defaults remain 200,000 and 100,000 bytes respectively. The F407 reduction from 30,000 to 20,000 bytes lowers script capacity. A 32-bit simulator check with a 572-byte script reported 14,085 bytes used across startup, execute, and reload; that is not a hardware or real-workload guarantee.
- On all F4 boards, CAN and USB background objects use ordinary SRAM, and the Lua CAN filter table remains one static 48-entry array there (768 bytes at 16 bytes per entry). The main loop remains in CCM on F407. Non-Ethernet F429 builds move it to ordinary SRAM; Ethernet F429 builds retain CCM. The main-loop stack remains 5,120 bytes in every case.
- Compressed embedded INI mass storage uses one shared 16 KiB DEFLATE dictionary. A static assertion enforces that the firmware dictionary is at least as large as the generated image's declared window.
- On STM32F7, every persistent configuration copy must fit the minimum runtime flash slot of 32 KiB. Dual-bank 2 MB places its copies 32 KiB apart (`0x08100000` and `0x08108000`); a larger record overlaps the second copy's sector and `writeConfigurationCopies()` rejects the layout. A static assertion in `persistent_configuration.h` enforces this limit only for F7 builds. The measured Proteus F7 profile with the expanded 32-point VE, 16-point blend, and default six cylinder trims is 31,332 bytes (31,312-byte payload plus 16-byte header and 4-byte CRC), leaving 1,436 bytes. In the measured Proteus F7 profile, raising cylinder-trim capacity from 6 to 12 adds 1,540 bytes: the 32,872-byte record exceeds the slot by 104 bytes and is rejected during compilation. Exact sizes remain board- and profile-dependent. This check adds no RAM cost; F4 and H7 layouts are unchanged.
- F7 functions use 4-byte alignment and retain the default `-O2`; timing for these changes has not been benchmarked.

## Generated configuration

The selected profile drives both live-data generation and `gen_config_board.sh`. Host-only standalone generators refresh the profile through `module_profile_host.mk`, which reuses the canonical resolver without requiring a compiler. When parent Make supplies an explicit profile, the generators validate and reuse it.

Generated headers, configuration structures, and `firmware/tunerstudio/generated/*.ini` files are build products and are not committed. Treat a generated INI and its firmware as one matching pair.

Cylinder fuel trim renders its exact 1-to-N TunerStudio rows into `firmware/generated/cylinder_fuel_trim.ini`. This is a small fixed renderer for the module's single repeated row pattern, not a general template language. The profile is refreshed before generation, so changing the count, fuel table dimensions, or blend size updates the generated headers and configuration stamp on the first Make invocation without requiring a clean build.

The flash header identifies a snapshot by version, container size, and a static configuration-layout ID. A snapshot with an incompatible layout is rejected even when its byte size is unchanged, and defaults are restored. There is no binary migration or retention of values from inactive modules; use the matching generated INI to export and manually re-import values after an intentional layout change.

Enabling, disabling, or changing the cylinder-trim count changes the persistent layout. No tune migration is provided for an incompatible cylinder-trim layout; the firmware starts from its defined defaults and the matching generated INI must be used to transfer values manually.

## Retained shared boundaries

- Overboost fuel cut and generic DC/ETB wastegate control remain shared; both work without the Boost module.
- Generic oil pressure and temperature paths remain shared; Hella is an optional provider.
- The binary tooth logger and shared RPM threshold remain available without Engine Sniffer, which adds only the optional event recorder.
- CAN `scriptTable1` handling and the VVT dimensions remain shared contracts.
- Generic CLI `get` lookup remains available. The Lua-only ETB timer/getter/setter path compiles out with existing `EFI_LUA`, while ETB itself stays enabled; small neutral generated Lua adjustment state remains in Boost, idle, ETB, and ignition so hot paths do not need sprawling guards.

Electronic throttle control and traction control are outside this modularization scope.
