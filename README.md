<div align="center">

<img src="https://avatars.githubusercontent.com/u/325562820?v=4" alt="Capoworks logo" width="160">

![Last Commit](https://img.shields.io/github/last-commit/capoworks/capoworks-ecu?style=flat)
[![Capoworks Checks](https://img.shields.io/github/actions/workflow/status/capoworks/capoworks-ecu/capoworks-checks.yaml?branch=master&label=Capoworks%20Checks)](https://github.com/capoworks/capoworks-ecu/actions/workflows/capoworks-checks.yaml?query=branch%3Amaster)

</div>

# Capoworks ECU

Capoworks ECU is an independent open-source engine-control firmware project focused on new features, experimental control strategies, and application-specific development.

The firmware provides a platform for testing alternative fuel models, ignition scheduling, signal processing, and hardware integration. Experimental features are documented with their operating requirements, limitations, and validation status so developers and tuners can evaluate them deliberately.

**[On-Demand Firmware Builder](https://capoworks.undo.it/firmware) · [Wiki and Documentation](https://capoworks.github.io/capoworks-ecu-docs/) · [Recent changes](https://capoworks.github.io/capoworks-ecu-docs/recent-changes/) · [Source](https://github.com/capoworks/capoworks-ecu) · [Issues](https://github.com/capoworks/capoworks-ecu/issues) · [Actions](https://github.com/capoworks/capoworks-ecu/actions)**

## On-Demand Firmware Builder

Use the public [on-demand firmware builder](https://capoworks.undo.it/firmware) to create a firmware image for a supported board. Sign in to your account, select the board, and choose any opt-in modules and table dimensions before starting the build. When the build completes, download the matching firmware image and generated TunerStudio INI. Each user can run up to six builds per day. See the [on-demand firmware build documentation](https://capoworks.github.io/capoworks-ecu-docs/Software/On-Demand-Firmware-Build/) for details.

## Firmware Highlights

- **Blended fuel strategy:** independently calibrated Speed Density and Alpha-N air-mass models. At 0% authority the result uses Speed Density; at 100% authority it uses Alpha-N. Intermediate authority blends the calculated air masses.
- Separate **16×16 per-cylinder trim maps** for Speed Density and Alpha-N.
- Independent load-coordinate selection for control functions and protections.
- Opt-in rolling idle-RPM feedback for idle-air and idle ignition correction.
- Two ignition strategy selectors default to **Legacy**; **Cycle Profile** and **Time Budget** are opt-in alternatives.

Cycle Profile and Time Budget depend on trigger geometry, cam synchronization, and ignition configuration. Check the wiki prerequisites before enabling either strategy.

## Documentation

- [Blended fuel algorithm](https://capoworks.github.io/capoworks-ecu-docs/Fuel/Fuel-Algorithms/Blended/)
- [Air-mass models and migration](https://capoworks.github.io/capoworks-ecu-docs/Fuel/Fuel-Algorithms/Airmass-Models-and-Migration/)
- [VE Analyze](https://capoworks.github.io/capoworks-ecu-docs/Fuel/Fuel-Settings/VE-Analyze/)
- [Core8 build tools](https://capoworks.github.io/capoworks-ecu-docs/Software/Core8-Build-Tools/)

VE Analyze can calibrate a Speed Density or Alpha-N model in a Blended configuration only when authority is uniformly 0% or 100% throughout the qualified session.

## Compatibility

**Current tune format: 20032.** Binary tunes from older formats are reset rather than automatically migrated. Create a tune with the matching INI, then manually transfer the relevant values, maps, and axes according to the [migration guide](https://capoworks.github.io/capoworks-ecu-docs/Fuel/Fuel-Algorithms/Airmass-Models-and-Migration/). Back up configurations before upgrading.

## Repository Layout

- [firmware/](firmware/) — engine-control firmware and board configurations
- [java_console/](java_console/) — ECU tuning and debugging console
- [simulator/](simulator/) — firmware simulation on Windows or Linux
- [unit_tests/](unit_tests/) — firmware unit tests
- [java_tools/](java_tools/) — development utilities

## Development

Build instructions and development guidance are in [firmware/DEVELOPER.md](firmware/DEVELOPER.md).

## License

The project license and terms are in [LICENSE](LICENSE). Firmware and bundled third-party components retain their respective copyright notices, license terms, and third-party licence notices; review them before redistribution.

## Credits

Author of Capoworks-specific contributions: **Giacomo di Ciocco**.

Capoworks derives from [FOME-Tech/fome-fw](https://github.com/FOME-Tech/fome-fw), which derives from [rusEFI/rusefi](https://github.com/rusefi/rusefi). We thank the authors, contributors, and communities of both projects for the engineering, knowledge, and tools that made this work possible.

Capoworks is independent and is not officially endorsed or supported by FOME or rusEFI. Documentation credits and source acknowledgements are maintained at [Credits and Sources](https://capoworks.github.io/capoworks-ecu-docs/credits-and-sources/).
