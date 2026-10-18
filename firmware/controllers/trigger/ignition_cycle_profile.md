# Experimental ignition cycle profile and time-budget planning

TunerStudio exposes two independent settings under Experimental/Broken:

- **Ignition scheduling: Legacy / Cycle profile (experimental)** changes only the final angle-to-time conversion for ignition charge and firing. RPM, injection, global phase interpolation, trailing-angle offsets and dwell-angle planning keep their existing behavior.
- **Dwell planning: Legacy / Time budget (experimental)** plans charge and fire timing from the same bounded causal cycle history. It requires Individual Coils; shared-coil and wasted-spark modes remain on Legacy planning.

Both settings default to Legacy. Time-budget planning additionally requires a supported four-stroke, single-primary, rising-only missing-tooth crank wheel and full cam phase synchronization. Current support includes 36-2 and 60-2 wheels. Unsupported geometry, unknown phase, direct self stimulation, or an unavailable profile falls back to the Legacy conversion.

## Configuration migration

Each new setting is appended at the end of `engine_configuration_s`, after the existing idle rolling-RPM field. The flash data version is `20029`. The loader rejects incompatible versions or stored sizes and restores defaults. Export a tune with the matching firmware and TunerStudio definition and migrate it field by field; do not copy a binary tune from another layout.

## Planned timing behavior

The profile reuses Instant RPM's existing timestamp history and updates it before the current tooth timestamp is overwritten. There is no second timestamp ring. The causal ETA and inverse conversion are limited to 180 degrees and 32 references. Missing history uses the existing RPM conversion.

When armed, a time-budget target is attached to one cylinder TDC occurrence. A newly observed target already behind the bounded horizon is counted as expired and is not wrapped into a later occurrence. A retained candidate can accept a feasible portion of a changed advance or retard command; an unchanged command is not repeatedly rescheduled merely because another tooth arrived.

Before HIGH, the window reserves 0.8 times nominal dwell plus 100 microseconds for planning and timer setup. While charging, a revised LOW must allow physical minimum dwell and finish before the immutable 1.5 times nominal-dwell hard deadline, less the same service reserve. HIGH and LOW callback generations are invalidated before replacement registration. The first HIGH timestamp, output ownership, nominal dwell snapshot and physical deadline do not change when LOW is retargeted.

## Limits

The 100-microsecond reserve is a software planning allowance, not a measured worst-case interrupt or execution bound. The 1.5 times nominal-dwell cutoff is a firmware timer, not independently clocked hardware current protection. Physical TDC, coil current, output propagation delay, cranking behavior and worst-case interrupt latency require hardware validation.

Interpolation assumes roughly constant speed within each historical interval. Tooth logs do not establish physical TDC or instantaneous speed between edges. Low-speed ripple rescaled to high RPM is synthetic stress evidence, and host test duration is not STM32 execution time. The planner is bounded but may perform multiple conversions when several targets share a tooth; keep it experimental pending target builds, replay review and bench validation.
