# Build-time module profiles

Firmware modules are selected at compile time. `firmware/controllers/modules/module_profile.mk` is the single canonical catalog and profile resolver; there is no runtime module registry, tune setting, or new configuration option for choosing modules.

## Catalog and selection

The canonical catalog is:

```text
fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer lua
```

Set `FOME_MODULES` to a whitespace-separated list of these exact, lowercase tokens. An omitted value defaults to the full catalog. Unknown tokens fail during Make configuration, and selections are deduplicated and resolved in catalog order. An explicitly empty value remains empty. The existing `NO_MAP_AVERAGING` switch still removes `map_averaging`.

Keep `fan fuel_pump gear_detector tachometer trip_odometer map_averaging` in every supported profile. The audited optional exclusions are:

| Feature | Token |
|---|---|
| Hella OPS-T | `hella_opst` |
| Boost | `boost` |
| Engine Sniffer | `engine_sniffer` |
| Lua | `lua` |

Other legacy modules have not been newly audited for removal. A supported Core8 build excluding only Lua (the base modules plus Hella OPS-T, Boost, and Engine Sniffer) is:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging hella_opst boost engine_sniffer" bash compile_core8.sh
```

A base-only Core8 build excluding all four optional modules is:

```bash
cd firmware/config/boards/core8
FOME_MODULES="fan fuel_pump gear_detector tachometer trip_odometer map_averaging" bash compile_core8.sh
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

`integration/LiveData.yaml` owns live-data usage declarations and marks module-specific usages with `module:`. YAML order fixes `LDS_<usage>` IDs, so an unselected module keeps its declaration and IDs while its fragment and packed output usage disappear. Do not reorder or duplicate usages just to change a profile.

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
