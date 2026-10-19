# Build-time module profiles

Firmware modules are selected at compile time. `firmware/controllers/modules/module_profile.mk` is the single canonical catalog and profile resolver; there is no runtime module registry, tune setting, or new configuration option for choosing modules.

## Catalog and selection

The canonical catalog is:

```text
fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua emp_pump shock_preload
```

Set `FOME_MODULES` to a whitespace-separated list of these exact, lowercase tokens. It replaces the default profile rather than adding to it. Unknown tokens fail during Make configuration, and selections are deduplicated and resolved in catalog order. An explicitly empty value remains empty. The existing `NO_MAP_AVERAGING` switch still removes `map_averaging`.

An omitted value selects:

```text
fan fuel_pump gear_detector tachometer trip_odometer map_averaging boost engine_sniffer lua
```

On `hellen128`, `lua` is removed from that default because this board does not support Lua. Non-`hellen128` boards retain `lua` in their default profile. The three modules `hella_opst`, `emp_pump`, and `shock_preload` are omitted from the default profile on every board, but remain available for explicit selection. The catalog above lists every token recognized by the resolver; it is not the default profile.

Keep `fan fuel_pump gear_detector tachometer trip_odometer map_averaging` in every supported profile. Build availability and default inclusion are:

| Feature | Token | Included by default |
|---|---|---|
| Hella OPS-T | `hella_opst` | No |
| Boost | `boost` | Yes |
| Engine Sniffer | `engine_sniffer` | Yes |
| Lua | `lua` | Yes on non-`hellen128` boards |
| EMP coolant pump | `emp_pump` (requires `fan`) | No |
| Shock preload | `shock_preload` (CAN2, requires CAN read and write) | No |

Other legacy modules have not been newly audited for removal. Explicitly selecting `lua` on `hellen128` is an error rather than a silent omission. `emp_pump` reads both fan-controller states for ram-air relief, so selecting it without `fan` is rejected during Make configuration.

To select all currently default-omitted modules on Core8, which is not `hellen128`, spell out the complete profile:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer emp_pump shock_preload" bash compile_core8.sh
```

Adding `lua` to that list selects the full catalog on Core8. A base-only Core8 profile is:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging" bash compile_core8.sh
```

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

`integration/LiveData.yaml` owns live-data usage declarations and marks module-specific usages with `module:`. YAML order fixes `LDS_<usage>` IDs, so an unselected module keeps its declaration and IDs while its fragment and packed output usage disappear. Do not reorder or duplicate usages just to change a profile.

## Resource budgets

These are compile-time defaults and placements, not calculated free-RAM margins. Board-specific `LUA_USER_HEAP` overrides can change the heap figures.

- The default Lua heap is 20,000 bytes on F407, 60,000 bytes on non-Ethernet F429, and 25,000 bytes on Ethernet F429. The F7 and H7 defaults remain 200,000 and 100,000 bytes respectively. The F407 reduction from 30,000 to 20,000 bytes lowers script capacity. A 32-bit simulator check with a 572-byte script reported 14,085 bytes used across startup, execute, and reload; that is not a hardware or real-workload guarantee.
- On all F4 boards, CAN and USB background objects use ordinary SRAM, and the Lua CAN filter table remains one static 48-entry array there (768 bytes at 16 bytes per entry). The main loop remains in CCM on F407. Non-Ethernet F429 builds move it to ordinary SRAM; Ethernet F429 builds retain CCM. The main-loop stack remains 5,120 bytes in every case.
- Compressed embedded INI mass storage uses one shared 16 KiB DEFLATE dictionary. A static assertion enforces that the firmware dictionary is at least as large as the generated image's declared window.
- F7 functions use 4-byte alignment and retain the default `-O2`; timing for these changes has not been benchmarked.

## Generated configuration

The selected profile drives both live-data generation and `gen_config_board.sh`. Host-only standalone generators refresh the profile through `module_profile_host.mk`, which reuses the canonical resolver without requiring a compiler. When parent Make supplies an explicit profile, the generators validate and reuse it.

Generated headers, configuration structures, and `firmware/tunerstudio/generated/*.ini` files are build products and are not committed. Treat a generated INI and its firmware as one matching pair.

The flash header identifies a snapshot by version, container size, and a static configuration-layout ID. A snapshot with an incompatible layout is rejected even when its byte size is unchanged, and defaults are restored. There is no binary migration or retention of values from inactive modules; use the matching generated INI to export and manually re-import values after an intentional layout change.

## Retained shared boundaries

- Overboost fuel cut and generic DC/ETB wastegate control remain shared; both work without the Boost module.
- Generic oil pressure and temperature paths remain shared; Hella is an optional provider.
- The binary tooth logger and shared RPM threshold remain available without Engine Sniffer, which adds only the optional event recorder.
- CAN `scriptTable1` handling and the VVT dimensions remain shared contracts.
- Generic CLI `get` lookup remains available. The Lua-only ETB timer/getter/setter path compiles out with existing `EFI_LUA`, while ETB itself stays enabled; small neutral generated Lua adjustment state remains in Boost, idle, ETB, and ignition so hot paths do not need sprawling guards.

Electronic throttle control and traction control are outside this modularization scope.
