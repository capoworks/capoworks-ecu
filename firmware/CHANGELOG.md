<!---

How to use this file:
- When making a change to the firmware that users should know about, add it to the
	appropriate section (breaking/added/fixed) in the next release.
- If it's something interesting or breaking, make a post about it on the forum! We want users
	to find out about changes before they have to get help because their car won't start.

Release template (copy/paste this for new release):

## Month 202x Release - "Release Name"
or
## Unreleased

### Breaking Changes
 - 

### Added
 - Added a dedicated 16x16 Alpha-N cylinder-filling map with independent TPS/RPM axes and configurable temperature, MAP-estimate, BARO and Idle VE policies; because this changes the calibration layout, older binary tunes reset and Alpha-N cells/axes must be transferred manually.
 -

### Fixed
 - 

### Removed
 - 

 -->

# Changelog

## Unreleased

### Breaking Changes
 - Advance the flash data version to `20032` for the current tune layout, which adds a dedicated airmass table, independent consumer selectors, the SD + Alpha-N authority schema, and experimental ignition settings. Older binary tunes reset; transfer values manually using the firmware's matching generated TunerStudio INI.
 - Tune images now carry a static configuration-layout identity for module-aware snapshots. Binary tunes with an incompatible layout are intentionally rejected and defaults are restored; export and re-import values with matching definitions because binary migration is not provided.
 - Fuel table resolution is now fixed by the firmware build. Changing `FOME_VE_RPM_COUNT` or `FOME_AIRMASS_BLEND_SIZE` creates an incompatible configuration layout, so binary tunes from a differently sized build are rejected and reset; use the matching generated TunerStudio INI to transfer calibration values manually.
 - Removed the explicit threshold for the closed loop fuel idle region. Instead, the idle region is used whenever the idle controller decides the engine is idling, as this better matches behaviors like idle-up when cold, return to idle, etc.
 - MAF trim is split out as its own table, rather than sharing the VE table. Most setups should have this table set to all 100, but if yours isn't, just copy values over from the VE table.

### Added
 - Add build-profile options for `FOME_VE_RPM_COUNT` (`16` or `32`) and `FOME_AIRMASS_BLEND_SIZE` (`8` or `16`). The defaults remain the legacy 16 RPM-point VE tables and 8x8 SD + Alpha-N authority table; expanded resolutions are intended for builds that can accommodate their larger configuration layout.
 - Add an optional Caponord electronic shock preload controller on CAN2, including preset, position, target, alarm and current telemetry, command service controls, and a default-off `enableShockPreload` runtime setting. The build module is opt-in.
 - Add an optional EMP coolant pump module with its Caponord J1939 state machine, configuration and fallback tables, telemetry, service controls, after-run power hold, and CAN TX-thread queue. The build module is opt-in and remains disabled at runtime by `enableEmpPump`; selecting it requires the fan module.
 - Add an optional Engine Sniffer event recorder while preserving the shared RPM guard for the binary tooth logger and verbose trigger logging.
 - Lua core support is now a board-aware build module, included by default on every board except `hellen128`, with its VM, tune fields, gauges, bench outputs, and digital inputs omitted when the module is not selected.
 - Add an optional, default-on cylinder fuel trim build module with one 16x16 factor table per configured cylinder, all six selectable load coordinates, shared load/RPM axes, and materialized TunerStudio tables, load-source dialog, gauges, and output channels. Its default capacity is 6 and can be overridden from 1 to 12; explicitly replacing the module profile without `cylinder_fuel_trim` disables it. Enabling it or changing its count changes the persistent layout; no binary tune migration is provided for an incompatible layout.
 - Add an SD + Alpha-N airmass blend authority table with selectable TPS or measured-MAP load axes.
 - Add opt-in full-engine-cycle RPM feedback for closed-loop idle air and ignition timing, with instantaneous fallback until a complete cycle is valid and a `rollingCycleRpm` diagnostic channel. Instantaneous feedback remains the default.
 - Add experimental opt-in ignition cycle-profile conversion and time-budget dwell planning on supported rising-only missing-tooth crank wheels with full cam sync. Time-budget planning requires individual coils, admits each target for one TDC occurrence, and retargets only within causal timing and physical dwell windows. Legacy timing remains the default and fallback; RPM and injection policies are unchanged.
 - Independent full-precision load coordinates for injection phase, ignition trims, staged injection, STFT, lambda protection, knock, trailing spark, IAT correction and HPFP, with per-function selector UI and diagnostic cursors.
 - Add optional support for the Hella OPS+T (6PR 010 378-207) combined digital oil pressure and temperature sensor.
 - Add optional support for boost control at build time, including its configuration, tables, telemetry, Lua hooks, and TunerStudio controls. Overboost fuel cut and the generic DC/ETB wastegate path remain available independently.
 - SD card logs are now organized into `YYYY/MM/DD` folders when the real-time clock is set. If the clock is not set, logs are still written to the root of the card with a sequential name.
 - Add an opt-in "Service / Workshop" flood-clear mode that keeps fuel cut through the cranking-to-running transition until the accelerator is released. Roadside remains the default and restores fuel at running RPM; cranking priming now follows driver throttle intent and its flood-clear help reflects the 90% threshold.
 - Scheduler batches can atomically reserve paired output transitions, preventing partially scheduled pulses when the event queue is full or a request is invalid.
 - Fahrenheit temperature support: pick "Fahrenheit" under Settings > Temperature Units in TunerStudio and all temperature gauges, datalogs, sensor adjustments, and thermistor calibration points display in °F. The stored tune is unchanged (always Celsius internally), so switching units never resets your configuration and works on every supported board.
 - Add mode for "true" wasted spark on odd fire engines (Viper V10) where companion cylinders are not exactly 360 degrees apart. Requires cam sync.
 - New `CPU usage` output channel showing approximate firmware CPU load
 - SD card logs now contain every output channel (full parity with the TunerStudio log), rather than only a curated subset - previously-missing data like idle, throttle, wideband and wall-fuel state is now logged
 - IMU data (lateral/longitudinal acceleration and yaw rate) from the BMW E90 MK60e1/MK60e5 DSC/ABS module
 - New VVT mode "Honda K24Z Exhaust" for the three-tooth unevenly spaced exhaust cam wheel used on the K24Z, which differs from the 4+1 wheel of earlier K series engines. This pattern is used by K series engines that use a 60-2 crank pattern.
 - CAN wideband controllers now report "time since last CAN frame received" and a TunerStudio alive indicator for all 4 wideband channels (previously only 2 had live data at all), so a dead/disconnected controller can be spotted even when its lambda reading is invalid
 - New firing order 1-6-2-5-3-4 (Maserati V6) #789
 - Brake pedal switch state is now decoded from CAN when CAN VSS is set to BMW E8x/E9x MK60e5, so no physical brake switch input is required on those cars
 - Brake pedal switch input can now be inverted, for vehicles where the brake switch pulls the input low when the pedal is pressed
 - New CAN VSS type "AUMOVIO MK 100 UHP" for the Continental/AUMOVIO MK 100 UHP ABS module, decoding vehicle speed, all four wheel speeds, brake pedal state, and IMU data (lateral/longitudinal/vertical acceleration and yaw rate)
 - New VVT mode "Honda J 6-2" for the Honda J-series V6 intake cam, which has six evenly spaced tooth slots with two of them missing, giving one distinct wide gap per cam revolution.
 - Cooling fans can be turned off above a set vehicle speed ("Disable above speed") #827
 - Flex fuel ethanol content is now correct immediately at startup, instead of ramping up from 0% over the first second while the sensor's filter settles. The last valid reading is stored in backup RAM, and used to prime the filter at startup as well as any time the sensor is failed - the fuel in the tank can't change while the ECU isn't watching. If no value was stored and the sensor is dead, the fallback is configurable: "Failed flex sensor ethanol content", defaulting to 50%.
 - Added gauges for EGT channels 3-8 in TunerStudio
 - ECUs with WiFi (Atlas) can download logs over WiFi connection in addition to USB #842
 - ECUs with WiFi (Atlas) can download the TunerStudio ini matching the installed firmware over WiFi, from the same page as the logs. The page is available even with no SD card inserted.
 - USB mass storage can now be enabled independently of SD-card file logging, allowing INI-only MSD builds without FatFs.

### Tooling
 - Add deterministic ELF resource observations, bounded calibration plans, and a manual resource-matrix workflow. Normal firmware CI now gates packaging on the current build's resource report and publishes measurement indices separately from firmware artifacts. See [build resource evidence and calibration](../docs/build-resources.md).

### Changed
 - Build-time engine preset selection through `FOME_ENGINE_PRESETS` is mutually exclusive with expanded fuel table dimensions: presets are available at the default 16 RPM-point / 8-point blend resolution and are not applied to expanded builds.
 - Ethernet TCP buffers are now sized for the standard 1500-byte network MTU (1460-byte MSS) instead of growing with the TunerStudio block size. The effective packet MSS was already capped by the netif MTU, so this aligns lwIP's configured pool size with the packets it actually sends; no hardware testing has been performed for this change.
 - On STM32F4 boards, move background CAN, USB console and USB mass-storage objects from CCM to ordinary SRAM. On non-Ethernet STM32F429 builds, also move the main loop to ordinary SRAM; Ethernet-console builds retain the main-loop CCM placement. The common non-Ethernet F429 Lua heap is reduced from 65,000 to 60,000 bytes, while the 25,000-byte Ethernet Lua heap is unchanged. The F407 Lua heap is now 20,000 bytes, down from 30,000, reducing available Lua script capacity. Runtime behavior at these smaller heaps and timing with the new memory placement have not yet been validated on hardware.
 - The embedded compressed INI drive now uses a 16 KiB DEFLATE dictionary, reducing firmware RAM use by 16 KiB.
 - STM32F7 function alignment now uses 4-byte word alignment instead of 16-byte function alignment to reduce inter-function flash padding; timing has not been benchmarked.
 - Firmware CI now runs on master pushes and cancels superseded runs, while hardware CI is manual-dispatch only because it flashes connected ECUs.
 - Module profiles now omit Hella OPS-T, EMP pump, and shock preload by default on every board, and omit Lua by default only on `hellen128`. Select modules explicitly with `FOME_MODULES`; selecting Lua on `hellen128` is an error, while non-`hellen128` boards retain Lua by default.
 - Cylinder count is now derived automatically from the firing order instead of being a separate setting, so the two can no longer disagree.
 - Instant RPM is now used automatically on triggers with 24 or more teeth per engine cycle (a 12 tooth crank wheel or better), instead of only when "Always use instant RPM" was enabled. RPM, and everything derived from it, now responds within a fraction of an engine cycle instead of once per cycle. The setting remains, and now forces instant RPM on triggers with fewer teeth than that.

### Fixed
 - VE Analyze now applies strategy-specific crank, transient-fuel, cut-recovery, launch/anti-lag and lambda filters to Speed Density, Alpha-N and MAF maps, and publishes cranking and cut-recovery telemetry without moving existing output channels.
 - Invalidate superseded planner firing callbacks independently of occurrence ownership. Pending charge estimates, active pulse minimum dwell, physical hard deadlines, multispark timing and trailing-charge lifecycle retain their existing guards.
 - Electronic throttle and DC wastegate outputs stop immediately when disabled in configuration or when reinitialization fails.
 - Blocking calibration burns inhibit configured electronic-throttle and DC wastegate outputs, then resume only after fresh position feedback and a new valid control cycle.
 - Repeated software PWM stop and restart cycles no longer accumulate callback nesting until the nesting-limit error is hit.
 - Configuration flash operations are now serialized, so concurrent saves and reads cannot overwrite each other, and a failed write is not retried continuously without a new request.
 - Calibration burns now verify erase, programmed bytes, header, and CRC before replacing the remaining valid copy; a failed write stops immediately and reports how many copies were verified.
 - Bound every continuous ignition output pulse with an independent 1.5x nominal dwell guard. Physical ownership and pulse age survive callback promotion and repeated HIGH, competing occurrences cannot take over an energized coil, and the guard still forces LOW if the ignition occurrence was invalidated. Invalid dwell cannot energize a main, trailing or raw ignition output.
 - Preserve a pending ignition discharge before reusing the same event after rapid synchronization loss/restart or trigger configuration changes. Skip the overlapping charge so it cannot lose its overdwell protection.
 - Close ignition occurrences atomically and reject obsolete ordinary, underdwell, fire and charge callbacks. Snapshot each occurrence's outputs and dwell, give multispark pulses distinct generations, and recheck ignition authorization before charging or starting another pulse.
 - Use explicit trigger queue membership and tail pointers for constant-time insertion and unqueued-event cancellation
 - Prevent stale trigger-scheduled events and overdwell callbacks from interfering with newer ignition cycles after stop/restart or trigger reconfiguration; keep HPFP shutdown timers from restarting an old scheduling chain
 - The small embedded INI drive uses 512-byte clusters so the combined Capoworks definition and documentation fit without enlarging the image.
 - Board-fixed settings are now read-only in TunerStudio and protected during tune import, avoiding configuration CRC errors from editing board-enforced values.
 - Changing or clearing the MAP 2, MAF, MAF 2 or fuel level sensor input no longer leaves the sensor reading its old pin (and the pin claimed) until the ECU is rebooted. MAF and fuel level input changes now take effect immediately, like other analog sensors
 - STM32F7 dual-bank ECUs no longer stall (potentially stopping the engine) when burning configuration with the engine running - configuration is now committed to flash when the engine is stopped #776
 - Failed starts that never report positive RPM now complete the stopped transition after trigger timeout and clear stale engine scheduling.
 - SD binary logs encode the full 32-bit MLG data offset, including telemetry headers larger than 64 KiB.
 - SD card log field names now include their category prefix (e.g. `Boost: Target` instead of just `Target`), matching the names shown in TunerStudio
 - Injector and ignition circuit fault codes now name the correct cylinder on boards with smart driver chips. Cylinder 1 previously reported P0202/P0352 instead of P0201/P0351, cylinders 10-12 reported nonsense codes, and cylinder 12 reported no code at all
 - The "wideband controller firmware too old" fault now reports its own code (P2902) instead of P2133, which is also used for "accelerator pedal secondary too high" - the two faults could not be told apart
 - Sensor, trigger, cam, knock and injector/ignition circuit fault codes now require the fault to persist for about a second before they light the check engine light, so a single bad reading no longer latches a code #780
 - Fix fuel level input
 - Improve STM32H7/Atlas SD card reliability
 - General SD card logging performance and reliabilty improvements
 - Fix conflict between aux temp 2 and oil temperature sensor configuration
 - DC wastegate control now disables the motor bridge when position feedback or the requested output is invalid, instead of trying to drive the motor with an invalid value.
 - MAP cylinder balancing no longer corrupts the MAP reading above 255 kPa. Engines running more than ~22 psi of boost could see reported MAP jump anywhere between 60 and 440 kPa while actual manifold pressure was steady, throwing fuel and ignition off badly at high load.
 - Fix updating wideband O2 sensor modules with older firmware
 - DFCO "After cut timing ramp-in time" is now actually respected, rather than ignored and using 0.5s. #819
 - Fix an issue where STM32H7-based ECUs can freeze while applying an engine preset #834
 - Improve WiFi reconnection reliability after your laptop goes to sleep/drops off the network while connected #713
 - Fix vehicle speed indication while using a BMW E90/Mk60e5 ABS controller as the speed source
 - Core8 v2.3/v2.4 critical error LED now uses PG11 instead of the low-side output9 PD14
 - Configured MC33816 SPI selection is now honored instead of being overwritten during initialization

## May 2026 Release

### Breaking Changes
 - Upgrade console to use Java 21. This may require an update of your Java runtime.

### Added
 - Dead knock sensor detection #578
 - Allow sending CAN broadcast on both busses #712
 - PWM fan control with configurable duty cycle #233
 - AC state available as a GPPWM axis (e.g. for AC-dependent fan control) #233
 - Lua `getChannel` function for safely reading ECU values #721
 - Improved Atlas board support, including Polygonus engine presets #700
 - Clearer ignition/injection pin naming in TunerStudio #596
 - 1996-2006 Dodge Viper V10 crank trigger pattern

### Fixed
 - Prevent false ETB jam detection when the throttle is not being commanded (for example with "Disable ETB if engine is stopped" enabled), which could latch the throttle off permanently
 - TPS/PPS raw voltage now readable even when calibration is invalid, allowing users to calibrate sensors #710
 - Fix OpenBLT updating 1MB STM32F7-based ECUs
 - Improve RPM rate of change signal to idle control so the D-term is actually usable
 - Fix automatic knock frequency calculation when cylinder bore is not configured
 - Correct ADC conversion precision #707
 - Delay sensor error checking briefly after boot to avoid false errors
 - Fix boost control blend table axis names in TunerStudio
 - Default initialize output channels to avoid invalid values at startup
 - Clean up CAN verbose DBC file, removing invalid/orphan entries #677
 - Idle VE table when using an electronic throttle correctly uses accel pedal to match main idle logic

## November 2025 Release

### Breaking Changes
 - Closed loop boost control tables now have their own axes, rather than sharing with open loop. If using closed loop boost control, you will have to reconfigure the X/Y axes on the closed loop table.

### Added
 - Allow fractional tachometer pulse ratio for fine tachometer calibration
 - Add an option to ramp the idle target down as engine speed returns to idle. Makes the running -> idle transition much smoother while in closed loop mode #570
 - Lua functions `storePersistentValue`/`getPersistentValue` to persist values in backup memory between reboots #592
 - Fully disable VVT when the target is 0. This prevents trying to control VVT when near the stop, which can damage the locking pin on some engines.
 - Automatic cylinder balancing of MAP sensor readings for smoother running at high load #610
 - Table-based injector small pulse compensation #609
 - MAF filtering for better transient response
 - Minimum injector pulse width setting
 - Allow selection of DTC severity #625
 - AC pressure switch & startup delay (#623, #660, #661, #662)
 - Y axis override for VVT target
 - Feature to skip initial trigger pulses for noisy triggers #634
 - VVT open loop "hold" table #638
 - Fire DTCs for detectable cam/crank errors
 - Selectable X/Y axis channels for boost control #861
 - Implement a trailing spark table (for rotary and dual-plug engines) #695

### Fixed
 - CAN reception on STM32H7-based ECUs
 - Speed up bootloader launch by 250ms
 - Move ETB sensor redundancy from Experimental/Broken/Parking Lot to sensor dialogs #165
 - K20/K24 intake VVT angle reads correctly
 - Easier to decipher check engine light blink codes #653
 - TunerStudio: Provide sensible EGT sensor "high" alerting threshold defaults
 - Improve knock detection dialog UX
 - Increase memory available to Lua on STM32F4 and STM32F7-based ECUs

### Removed
 - Removed A/C-based alternator open-loop duty adder (#660)

## February 2025 Release

### Breaking Changes
 - Large data logging improvements and changes - in TunerStudio, you may need to re-create your dash layouts from default and then re-add any broken gauges and indicators.

### Added
 - Read (some) OBD2 error codes using a normal scan tool! Currently limited to some common sensor errors. (#529 thank you @dynfer for implementation, @paleppp for testing, and others!)
 - Support Miata NA CAS "CMP" signal as a cam input (if you've installed a 36-1 crank trigger on an NA, for example) #495
 - Support 4 independent wideband O2 channels, and 4 fuel trim banks
 - Flash error codes for some sensors using the check engine light #526
 - Implement knock sensing support for STM32H7 chips
 - Show knock sensing window on FOME console engine sniffer

### Fixed
 - Idle VE table's indicator dot properly tracks when using an idle VE Y-axis override
 - Minor performance improvements to injection/ignition scheduling
 - Per-cylinder knock sensing while in wasted spark mode (fixed by #553)
 - Improved transitions between fuel/spark modes, improves cranking and startup (#551, #553. #554)
 - Disable radiator fans while ignition switch is off

## September 2024 Release

### Added
 - Staged injection: a second set of fuel injectors that inject part of the fuel load when you run out of primary injector
 - VE/Ign/Boost blend tables can select a Y axis override for even more flexibility
 - Added Lua function `getAuxDigital` for parity with rusEFI #351
 - Optional table to change after start idle taper time #369 (thank you, @Krakert!)
 - Increased precision available for Lua tables
 - Add EGT values to CAN broadcast format #398
 - Add options to enable/disable optional CAN frames (cams, EGT so far)
 - Add 1-5-4-8-3-7-2-6 and 1-6-5-10-2-7-3-8-4-9 firing orders
 - Console command `set_sensor_mock` now accepts a sensor by name, instead of index. Example: `set_sensor_mock CLT 85.5`
 - TunerStudio UI improvements (#436, etc)
 - Dropdown selector for popular gearbox ratios (#358, thank you @alrijleh and @nmschulte!)
 - Add two more aux linear sensors #476
 - Support wasted spark on odd cylinder count and odd-fire engines. Improves startup and allows running without a cam sensor!
 - Add an option for the DFCO MAP threshold to use a table dependent upon RPM #485 (thank you @alrijleh!)
 - Option to disable DFCO on gear shift #487
 - Ability to use an 8x8 table for after-start fuel multiplier that depends on CLT and engine run time
 - Start-up tachometer sweep!

### Fixed
 - Improve performance with Lua CAN reception of a high volume of frames
 - Displayed units in TunerStudio change when switching between volume vs. mass injector flow modes #42
 - Make Toyota "3 Tooth Cam" decoder more robust #382
 - Flex sensor-derived fuel temperature indication works properly
 - Fix a scenario where noisy trigger can cause overdwell [rusefi/rusefi#6349](https://github.com/rusefi/rusefi/issues/6349)
 - Fix decel fuel cutoff (DFCO) on engines without a MAP sensor #457
 - Automatic calculation of knock sense frequency based on cylinder bore #379
 - Removed MAP sampling option that can result in inaccurate MAP sensor readings
 - Improve TunerStudio performance on STM32F4-based ECUs #474
 - Inhibit VVT control if the cam position sensor dies
 - Fix "Alpha-N IAT density correction" option

## December 2023 Release

### Breaking Changes
 - Simplified VVT single tooth cam modes. "Single tooth second half" and "single tooth first half" have been replaced by simply "single tooth". This may require re-setting your trigger and VVT timing offsets if you used "single tooth first half" prior to this change. #178

### Added
 - Log per-cylinder true ignition timing (includes trim, knock retard, etc) #76
 - Add mode for CLT/IAT sensors that are installed "high side" instead of typical "low side" #116
 - Automatic supply voltage compensation for VVT solenoids, giving more stable control as battery voltage changes #209
 - Configurable ignition timing limits, adding safety particularly for engines running wasted spark with a large cam #211
 - Improved injection, ignition bench test UI and functionality: More gauges, fuel pump on/off buttons for injector testing, and configurable duration/count #219
 - Preset for Mazdaspeed Miata (NB) IAT sensor
 - Option to compensate Alpha-N fueling with intake air temperature
 - Gasoline-scale AFR gauge: instead of showing true AFR, show "gasoline scale" where 14.7 means stoichiometric
 - Decode fuel temperature from flex fuel sensors #254
 - Allow brief operation over 100% injector duty cycle and add configurable duty cycle limits #215
 - Buttons to bump VVT targets for testing/PID tuning
 - Improved serial port selection for FOME console and firmware update
 - Better default configuration values for some advanced features
 - Support BMW E8x/E9x CAN speed sensor input (works for standalone MK60e5 too)

### Fixed
 - Improved bench test resolution (more usable for testing injectors, dwell, etc)
 - Maximum knock retard table displays correct Y axis values in TunerStudio
 - Make errors about fuel pressure sensors less aggressive #111 #117
 - Always operate in "two wire" mode for batch fuel, fixing batch firing order #23
 - Fix "Toyota 3 Tooth Cam" VVT mode (1JZ, 2JZ, 1G, etc) actually works now #237
 - Fix autotune while TS project is in AFR mode - corrections now made properly for target AFR table values
 - Changing idle stepper settings causes kernel panic #265
 - Data log channel "Fuel: Injection timing SOI" indicates correct values

## May 2023 Release

### Added
 - Detected gear and wastegate position to CAN format
 - Y axis override for Idle VE table. Helps with idle quality on ITB setups. #94
 - Blend tables for boost open and closed loop control #73
 - Log channels to make MegaLogViewer VE Analyze work #82
 - Enable knock sensing on F407 Discovery build, hardcoded to PA3 #105
 - Support gauge and differential fuel pressure sensors (in addition to absolute) #32
 - Snapshot builds have a date code in the folder name #91

### Fixed
 - Sensor checker wasn't checking sensors
 - Removed vestigial warning about unset ignition pins #79
 - Gear detection sometimes didn't work #86
 - Restore built-in INI storage on STM32F7-based ECUs
 - Improve log field names, easier to find the channel you're looking for #98

### For older changelog entries prior to fork from rusEFI
see [CHANGELOG.rusefi.md](CHANGELOG.rusefi.md)
