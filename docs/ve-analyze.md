<!--
@file ve-analyze.md
@author Giacomo di Ciocco, (C) 2026
-->

# VE Analyze qualification diagnostics

Composite SD + Alpha-N VE Analyze remains an opt-in, session-qualified feature. These
channels explain the existing qualification latch; they do not rearm it, relax it,
or change fuel admission.

The TPS sample gate remains independent of the 20 Hz acceleration-enrichment update
and the composite-model qualification latch. The matching TunerStudio INI wires the
sample, rate, and event gates into all three VE Analyze blocks. Fuel and spark totals
continue to be calculated, the existing composite latch policy is unchanged, and
client-side physical alignment is not guaranteed.

`blendedVeAnalyzeCandidateEndpoint` records the endpoint identifiable for the
current candidate: `0` none, `1` Speed Density, or `2` Alpha-N. A uniformly
qualified map supplies the endpoint directly. For a mixed map, an authority
coordinate that resolves exactly to 0 or 100 supplies the candidate branch;
intermediate authority supplies 0. The original `blendedVeAnalyzeEndpoint` is
the session-qualified endpoint and still requires the whole authority map to be
uniform. A nonzero candidate with a zero qualified endpoint therefore means that
a branch was identifiable but the current calculation did not qualify.

`blendedVeAnalyzeFirstReason` is the immutable cause selected when the session
first rejects a result. `blendedVeAnalyzeRejectReasons` is the current bitmask;
after a valid candidate arrives in an already-invalid session it contains
`SessionAlreadyInvalid`. `blendedVeAnalyzeLatchedTime` is the whole number of
seconds since the first cause, saturated at 65535. Normal status export refreshes
this elapsed time even when no new fuel publication succeeds.

Qualification reasons are emitted only when the existing qualification predicate
does not accept the candidate. A fallback that is irrelevant to the endpoint does
not reject it: in particular, an otherwise valid Alpha-N result remains qualified
when only the strict MAP/native-load substitute falls back. This rule does not
weaken the whole-authority-map uniformity check.

Reason bits:

| Value | Reason | Meaning |
| ---: | --- | --- |
| 1 | `NonUniformAuthority` | The authority map is valid but is not uniformly 0 or 100. |
| 2 | `Configuration` | Required model or authority calibration is invalid. |
| 4 | `Sensor` | Required captured sensor evidence is absent or invalid. |
| 8 | `Fallback` | A physical-model, authority, temperature, baro, idle, correction, or MAP fallback participated. |
| 16 | `IdleVe` | Separate idle VE was active, or the model used its idle-VE fallback. |
| 32 | `CalculationResult` | The candidate failed validation without a more specific category. |
| 64 | `StaleCalculation` | A superseded generation calculation was rejected. Stale candidate diagnostics are never published over accepted state. |
| 128 | `CalibrationWrite` | A running-engine composite `LM_SD_ALPHA_N` calibration mutation invalidated the session. |
| 256 | `StrategyChange` | A running-engine strategy mutation invalidated the session. |
| 512 | `SessionAlreadyInvalid` | The current candidate did not own the latch, but the session was already invalid. |
| 1024 | `Stabilization` | Reserved; no stabilization policy is implemented. |

The first-reason priority is strategy change, calibration write, nonuniform
authority, configuration, sensor, idle VE, fallback, calculation result, stale
calculation, then session-already-invalid. In composite `LM_SD_ALPHA_N`, a running
calibration write latches `CalibrationWrite` even when a VE or Alpha-N table cell
changed to the same value, the authority map remains uniformly valid, and the next
valid candidate is otherwise usable. The write does not by itself change accepted
fuel numbers.

`blendedVeAnalyzeCalibrationGeneration`, `blendedVeAnalyzeCalculationEpoch`, and
`blendedVeAnalyzeSessionGeneration` distinguish a calibration write, an
individual calculation, and the lifetime of a VE Analyze session. A session is
cleared by the existing owner Engine reset or synchronization-loss path, and by
existing stopped-engine or strategy-reset transitions. A calibration write keeps
the existing behavior of clearing `veAnalyzeSessionStarted` and invalidating the
session.

`blendedVeAnalyzeDiscardedCount`, `blendedVeAnalyzeLastDiscardedReason`, and
`blendedVeAnalyzeLastDiscardedGeneration` are a separate discarded-calculation
domain. They record composite calculations rejected at the publication commit
when the captured calculation is no longer current. `LastDiscardedReason` is the
`StaleCalculation` bit, the count saturates at 65535, and the generation is the
calibration generation captured by the discarded calculation. This evidence is
updated under the publication critical section. It never latches the VE Analyze
qualification reason, replaces accepted candidate or endpoint evidence, or changes
accepted fuel output. It persists across later valid publications until the owner
Engine reset or synchronization-loss reset.

Qualification describes model idoneity, not lambda validity. A valid lambda
reading does not make a blended authority endpoint suitable for VE analysis.

## Independent TPS sample gate

TPS is read with `Sensor::get(Tps1)` at the ECU fast callback rate. A valid reading is
not clamped to the normal TPS percentile range. A new history sample is accepted only
when the `Tps1` provider reports a timestamp newer than the previous fast observation;
providers without timestamp support fail closed. The sampled value, provider freshness,
and previous-observation marker are read under the ECU critical section so an ADC update
cannot pair an old value with new freshness. History timestamps are approximate fast-loop
observation times rather than exact ADC conversion times. A cached valid value can remain
usable for at most the configured 50 ms sample tolerance. Invalid TPS is never converted
into a valid zero-percent sample. This does not change the separate 20 Hz
acceleration-enrichment sampling path.

The initial firmware policy is experimental and conservative:

| Policy | Default |
| --- | ---: |
| TPS rate threshold | 5 percentage points/s |
| TPS quick-movement threshold | 1 percentage point |
| Causal rate window | 0.2 s |
| TPS or fuel-correction recovery | 1.5 s |
| Fuel/spark-cut or DFCO recovery | 2.0 s |
| Maximum cached TPS age | 0.05 s |

These values are firmware guardrails, not reads of TunerStudio-adjustable numeric
thresholds or the Lambda Delay Table. They have not been physically validated against a
particular engine, exhaust transport delay, wideband response, or measured VE result.

The fast observation is performed after module callbacks so that current
`LimpManager` injection/ignition permission is available. Actual fuel cuts, spark cuts,
and DFCO all keep the gate closed and use the cut recovery hold. Published TPS
acceleration enrichment above 0.01 ms uses the fuel-correction hold, including while it
remains active. Other active launch, anti-lag, cranking, invalid-RPM, invalid-sensor, or
invalid-fuel-calculation conditions keep the gate closed for as long as they persist.

`veAnalyzeTpsRateAbs` is the absolute TPS rate in percentage points per second, and
`veAnalyzeRateValid` reports only whether the TPS timestamp history is complete enough
to calculate it. `veAnalyzeSampleReady` additionally requires recovery holds and every
operating exclusion to clear. `veAnalyzeSampleRejectReasons` combines the sampler and
operating reason bits. Time since movement, time since correction, and remaining
recovery are exported as physical seconds with saturation at 655.35 s.
`veAnalyzeAcceptedObservations` and `veAnalyzeRejectedObservations` classify each ECU
fast observation after applying the sample gate and operating exclusions. They do not
apply composite-model qualification, validate lambda, or represent client/TunerStudio
acceptance of a table cell. Resetting the owner engine or losing trigger synchronization
clears these counters and requires a fresh TPS window plus the reset hold.

The existing gate reason bits are:

| Value | Reason |
| ---: | --- |
| 1 | `InvalidConfiguration` |
| 2 | `InvalidTps` |
| 4 | `StaleTps` |
| 8 | `InsufficientHistory` |
| 16 | `SampleGap` |
| 32 | `DuplicateTimestamp` |
| 64 | `RegressedTimestamp` |
| 128 | `TpsMovementRecovery` |
| 256 | `FuelCorrectionRecovery` |
| 512 | `FuelCutRecovery` |
| 1024 | `CalibrationRecovery` |
| 2048 | `StartupRecovery` |
| 4096 | `ResetRecovery` |

The operating exclusions above those bits are:

| Value | Reason |
| ---: | --- |
| 8192 | `InvalidRpm` |
| 16384 | `Cranking` |
| 32768 | `Launch` |
| 65536 | `Antilag` |
| 131072 | `InvalidSensor` |
| 262144 | `InvalidFuelCalculation` |
| 524288 | `FuelCut` |
| 1048576 | `SparkCut` |
| 2097152 | `Dfco` |

## Fuel event integration

`EngineState` owns the fixed-size fuel-event collector. The same owner reset used
for the TPS sample gate also runs from Engine construction, Engine reset, and
trigger-synchronization loss, so stopped or unsynchronized engines require fresh
fuel-event coverage. Collector observation and snapshots require the caller to
hold the ECU critical section. The trigger path only records one bounded attempt.
Production TunerStudio export uses a compact Engine-owned summary and streams one
bounded cylinder at a time under the existing lock. The full collector snapshot
API is retained for standalone use and tests; production does not take a full
automatic snapshot or allocate heap storage for this export.

For an actual `InjectionEvent` that has passed admission, the owner cylinder's
cycle mass and `CylinderFuelProvenance` are captured as one pair under the
publication critical section. The base pre-AE and applied-AE provenance masses
are each multiplied by the active injection-mode multiplier exactly once.
Wall-film evidence is derived as the wall-adjusted event mass minus the
mode-scaled input event mass, rather than from stale `wallFuel` state.
`wallFuelCorrection` itself is not reused as event evidence.

Every phase-relevant attempt after admission is recorded, including non-finite
or negative pulse validation, a first-stage pulse shorter than 50 us, duration
or delay bounds, timestamp overflow, and scheduler failure. `SchedulerAccepted`
is set to true only from an actual successful `scheduleFuelCallbacks()` call.
It means that the scheduler reserved callbacks; it does not claim that a pin opened or that fuel
was physically delivered. `veAnalyzeFuelEventAttemptedCount` and the per-bank
attempt counters count all observed attempts. `veAnalyzeFuelEventAcceptedCount`
counts only coherent, completely represented attempts with
`SchedulerAccepted=true`. `veAnalyzeFuelEventRejectedCount` counts all observed
attempts with `SchedulerAccepted=false`, including structurally invalid attempts
and failures in early validation. `veAnalyzeFuelEventInvalidCount` counts
structurally invalid or unsupported evidence, independent of scheduling.
Rejection is evaluated before the structural-invalid return, so the Rejected and
Invalid counters can overlap and are not disjoint partitions of attempts.

The event timestamp and age describe the trigger phase that calculated and
attempted the schedule. They are not the scheduled opening time, an observed
pin edge, or a physical-delivery timestamp.

The affected cylinder mask comes from `calculateInjectorOutputMask()`. For
sequential injection this is the owner cylinder; batch adds the firing-order
twin; simultaneous selects all configured cylinders; single-point selects
injector 1. The affected bank mask is the union of `cylinderBankSelect` for every
selected physical cylinder. Event readiness requires fresh evidence for every
configured cylinder and every selected bank, not merely one fresh event or one
quiet bank. The expected, observed, and fresh masks are exported separately.

The following command shapes are represented as fully scheduled evidence:

- Sequential injection with a coherent owner provenance and no split.
- Batch injection with the actual paired output mask.
- Simultaneous injection with the actual all-cylinder output mask.
- Staged injection when stage 2 is either absent or actually scheduled.

The following cases deliberately fail closed with `IntegrationUnsupported`:

- Single-point injection, because one inferred event cannot establish
  per-cylinder provenance.
- Split injection, because the continuation is scheduled later from the first
  pulse callback and the initial call does not represent the complete command.
- A positive stage-2 command whose duration is suppressed at or below 50 us,
  because the first-stage success does not make the omitted stage-2 pulse part
  of the scheduled command.

These cases keep the existing fuel calculation, wall model, stage split, and
scheduler decisions unchanged. They do not create a complete scheduled mass or
accepted count. Unsupported and rejected attempts apply a recovery hold to every
affected bank, so polling or a later quiet cylinder cannot make them eligible.
Missing, invalid, or stale provenance publishes generation zero and also fails
closed without changing the real injection command.

Fuel-event reason bits are:

| Value | Reason |
| ---: | --- |
| 1 | `InvalidConfiguration` |
| 2 | `InvalidOwnerCylinder` |
| 4 | `InvalidCylinderMask` |
| 8 | `InvalidBankMask` |
| 16 | `InvalidGeneration` |
| 32 | `RegressedGeneration` |
| 64 | `DuplicateTimestamp` |
| 128 | `RegressedTimestamp` |
| 256 | `NonFiniteMass` |
| 512 | `NegativeMass` |
| 1024 | `MassIdentityMismatch` |
| 2048 | `StageMassMismatch` |
| 4096 | `InvalidDenominator` |
| 8192 | `ExcessAeRatio` |
| 16384 | `ExcessFilmRatio` |
| 32768 | `ExcessCombinedRatio` |
| 65536 | `NormalRecovery` |
| 131072 | `WallModelClampRecovery` |
| 262144 | `ZeroCommandedRecovery` |
| 524288 | `SchedulingRejectedRecovery` |
| 1048576 | `ResetRecovery` |
| 2097152 | `NoFreshEvent` |
| 4194304 | `StaleEvent` |
| 8388608 | `FutureTimestamp` |
| 16777216 | `IntegrationUnsupported` |

`veAnalyzeFuelEventReady` describes event evidence only. It is separate from
`veAnalyzeSampleReady`, which retains its TPS, history, recovery, and operating-
condition semantics. Both must still be combined with model qualification and
lambda validity outside this diagnostic contract.

All exported fuel masses are explicitly in milligrams. Global last-event,
per-bank latest, and per-cylinder latest evidence include base, applied AE, film,
attempted final, and accepted scheduled command where applicable. A scheduled
mass is zero unless the underlying record is fresh, scheduler-accepted, and not
superseded by a later rejected attempt affecting the same owner or bank.
Per-bank peaks are absolute AE and film percentages of base plus their sum.
Per-cylinder reason and age arrays expose each owner cylinder independently.
The last owner, age, serial, clamp, mask, base, AE, film, and attempted-final
fields describe the latest phase-relevant attempt even when it was rejected.
Only its scheduled mass requires the same attempt to have left a fresh,
accepted coherent event. The serial is split into low and high 32-bit channels.
It is producer-local, assigned under the observation lock, and not reset with
the event collector. The 25 per-reason rejection counters correspond to the
reason table above and saturate at `UINT32_MAX`.

Model-qualification and lambda validity are separate concerns. A ready sample gate does
not qualify a mixed SD + Alpha-N authority map, validate lambda, or guarantee that
TunerStudio will accept a cell. TunerStudio 3.3.01 automatic ECU writes are a user-target
workflow, not a capability already supported or validated by this firmware change for
continuous VE analysis. This change provides no client handshake and does not change the
PR1 latch: in composite `LM_SD_ALPHA_N`, an ordinary running calibration write still
invalidates the session, and the latch remains immutable until an existing
session-boundary transition. The composite latch does not cover standalone modes; see
[Live TunerStudio write target](#live-tunerstudio-write-target) for their limitations.

For composite recovery, correct the reported calibration, authority, fallback, idle-VE,
or sensor cause, then perform the normal engine stop/restart or synchronization loss
that starts a new session. Firmware does not automatically reset client history, force
endpoint `2`, expose a rearm command, rearm on a timer alone, or guarantee physical
alignment of analysis samples with lambda measurement.

## TunerStudio INI integration

This section describes the TunerStudio-side policy added on top of the exported PR2
and PR3 channels. It does not change the ECU configuration layout, generated INI files,
or the composite-model qualification condition on `veAnalyzeMap`.

The `Fuel > VE Analyze settings` dialog contains a TunerStudio PC-only
`Lambda sensor bank` choice: `Lambda 1` (the default) or `Lambda 2`. The selector is a
PC variable with a default in `[ConstantsExtensions]`; it is not an ECU calibration
field. Selecting a bank does not average lambda or EGO correction. Instead, one
`veAnalyzeSelectedLambda` value drives every VE Analyze map and its minimum, maximum,
and invalid-lambda filters, while `veAnalyzeSelectedEgoCorrection` uses the matching
`100 + fuelPidCorrection1/2` bank. WUE Analyze remains fixed to `lambdaValues1` and
its bank-1 `egoCorrectionForVeAnalyze` alias; the selector does not change WUE.

Choose the reference bank before starting a fresh analysis session. Changing the
bank during a running analysis is unsupported because historical corrections from
the old bank can mix with current lambda from the new bank. To change banks, stop
the analysis, start a new session, and verify that the client discarded its previous
history. Merely switching the selector is not known to clear client history or to
guarantee recovery; no such client handshake or UI lock is claimed.

Each of the three VE Analyze model blocks requires `veAnalyzeSampleReady == 1`,
`veAnalyzeRateValid == 1`, and `veAnalyzeFuelEventReady == 1`; a value of `0` in any of
these channels rejects the row. TunerStudio filters reject when their comparison is
true, so the INI implements all three gates as `< 1`. Their filter definitions end in
`userAdjustable=false`, but actual tablet-client behavior for that flag has not been
validated, so this document does not claim that a particular UI prevents changes.
The readiness filters are separate from `activeCondition`, which retains the existing
Speed Density, Alpha-N, MAF, blended-endpoint, Idle VE, and other model qualification.
The legacy `deltaTps` filters are replaced by a user-adjustable absolute TPS-rate filter
at `veAnalyzeTpsRateAbs > 5` percentage points per second.

The ECU event collector has its own fixed conservative policy: AE peaks above `1%`,
film peaks above `2%`, or their combined absolute ratio above `2%` reject the event.
The client filters use the same initial ratios and can be tightened for a particular
installation, but increasing a client threshold cannot relax the ECU gate. Existing
thermal and engine/tune defaults remain unchanged.

The default lambda acceptance window is `0.70` through `1.30`. Existing global CLT,
IAT, and other engine/tune filters are unchanged. Narrower CLT filters around
`75-90 C`, IAT filters around `15-35 C`, and a warmup-correction filter of `4%` are
optional experiments for a particular installation, not new engine or tune defaults.

An unavailable analog lambda sensor is exported as zero. Missing (`0`), nonpositive,
or NaN selected lambda values fail closed through the invalid-lambda filter. The
selected-bank check also fails closed when that bank is not represented in the
configured cylinder-to-bank assignments. This verifies consistency of the TunerStudio
selection and calibration assignments only; it cannot verify that
the physical wideband sensor, exhaust location, injector wiring, or bank plumbing
matches the label. A current installation report describes Lambda 1 as front and
Lambda 2 as rear, but physical cylinder, injector, and bank wiring remains unverified.
The tuning workflow must still confirm those physical details.

The per-bank event ratio channels are percentages of each event's pre-AE BASE fuel.
`veAnalyzeAePercent`, `veAnalyzeWallCorrectionPercent`, and
`veAnalyzeSumAbsAeFilmPercent` each take the maximum across all four banks, rather than
following the selected lambda bank. This prevents a quiet or unselected bank from
hiding a transient elsewhere. Their initial adjustable rejection limits are `1%` for
AE, `2%` for film, and `2%` for the combined ratio. `veAnalyzeWallCorrectionPercent`
remains as the saved-film-filter alias, but its value now comes from
`veAnalyzeFuelBankPeakFilmPercent1` through `...4`; it no longer uses
`wallFuelCorrectionValue`, `running_fuel`, or an injection multiplier.
`veAnalyzeSumAbsAeFilmPercent` consumes the ECU-side sum of the independently absolute
AE and film peaks. It must not apply `abs(AE + film)` in the client.
`veAnalyzeInjectionMultiplier` remains only for saved-definition compatibility and is
unused by these event ratios.

For a common VE reference-bank strategy, hold cylinder trims and thermal/film
calibrations fixed, then examine the other bank's residual separately. Do not average
the banks, automatically apply rear-cylinder trim, or migrate the reference axis.
The existing per-cylinder trim-load coordinate selection must be calibrated
deliberately; the workflow does not silently coerce one source or coordinate to
another.

The `wallFuelCorrection` and `wallFuel` fields are raw C++ `float` values in grams with
unchanged names and layout. Their generated telemetry descriptors scale by `1000` and
declare `mg`, so a raw `0.001 g` value is decoded by the client as `1 mg`.
`wallFuelCorrectionValue` is a separate packed output that already converts grams to
milligrams before transport; its scaling must not be applied a second time.

## PR5: thermal provenance and live-write/session contract

This section adds diagnostic provenance only. It does not add a production thermal
filter, tune change, airflow formula, fuel fallback, or VE Analyze rearm policy.

### Charge-temperature observation domain

The fast path evaluates `updateTChargeK(rpm, tps)` before it captures airmass inputs
and calculates the current fuel candidate. In `AIR_INTERP` and
`AIR_INTERP_TABLE` modes, the charge-temperature coefficient consumes the previous
accepted `sdAirMassInOneCylinder` multiplied by the current cylinder count, together
with the current RPM and engine-cycle duration. The new `airflowEstimate` and
`sdAirMassInOneCylinder` values are written later, only when that new fuel candidate
is accepted. They therefore are not the airflow that fed the coefficient in the same
callback.

The following channels make that ordering observable:

| Channel | Meaning |
| --- | --- |
| `thermalCoefficientAirflowKgPerHour` | The exact local airflow expression passed to the charge-temperature coefficient. |
| `thermalCoefficientSourceGeneration` | The calibration generation attached to the preceding accepted airflow publication. |
| `thermalCoefficientAirflowValid` | Whether a source publication was valid and the coefficient actually consumed a finite airflow input. |
| `thermalCoefficientSourceMode` | The `tChargeMode_e` value used for this evaluation: `0` RPM/TPS, `1` airflow interpolation, `2` airflow table. |
| `thermalAcceptedAirflowGeneration` | Calibration generation stamped with the currently accepted fuel publication. |
| `thermalAcceptedAirflowValid` | Whether that accepted fuel publication exists and is valid. |
| `thermalFuelTemperatureK` | The captured temperature supplied to the accepted standalone fuel candidate, in kelvin. |
| `thermalFuelTemperatureGeneration` | Calibration generation captured with that fuel candidate. |
| `thermalFuelTemperatureValid` | Whether the temperature used by the accepted candidate is available. |

The accepted mass and its generation/validity are committed together under the
existing fuel-publication critical section. Coefficient evaluation captures the prior
mass and source generation under that same lock before applying the unchanged airflow
operation. This prevents an old mass from being paired with a newer generation during
diagnostic export. A stale candidate never replaces either the accepted stamp or the
last exported accepted evidence. A current calculation failure, owner reset, or
synchronization-loss reset clears the accepted stamp and all thermal snapshots because
their source is then unknown or no longer live.

After a calibration write, the previously accepted publication can remain live until
its replacement is accepted. Its older generation makes that distinction explicit; it
is not evidence that the new calibration has already propagated into thermal input.
On the callback after a successful replacement, the coefficient source generation
still identifies the preceding publication while `thermalAcceptedAirflowGeneration`
and `airflowEstimate` advance to the newly accepted publication.

`RPM/TPS` charge-temperature mode does not consume airflow. It exports
`thermalCoefficientAirflowValid == 0`, generation `0`, and mode `0`; a zero airflow
value in this case is not a valid measurement. Mode `1` or `2` with an invalid source
stamp also exports invalid evidence rather than treating the stale scalar as current.

The temperature snapshot is captured with standalone physical-model fuel inputs.
Non-standalone legacy models do not expose an immutable temperature-input snapshot on
this path, so the fuel-temperature provenance remains invalid for them. Existing
`sdTcharge_coff`, `sd_tChargeK`, and `airmassTemperature` channels remain available.
Their values do not establish a common calibration generation when read on their own.

The optional charge-temperature slope, `dTcharge/dt` in C/s, is explicitly
experimental. This change does not implement or filter that slope, and it must not be
used to tune or alter the existing temperature rate limits.

### Live TunerStudio write target

TunerStudio 3.3.01 automatic ECU update is a user-target workflow. It is not a
continuous VE Analyze workflow validated by this firmware. The local TunerStudio JAR
artifact inspected for this contract had SHA-256
`3ebafb867fef3722816e768879501845904f81bc5e13249c77d170e9adf61cf8`.
Identical bytes on any user's tablet are not established, and the proprietary JAR or
bytecode is not copied into this repository.

Static bytecode inspection observed current axes and current lambda alongside
historical VE-table, EGO, and target scalars. It also showed that rows rejected by the
current-packet filters have already been inserted into history. An inactive
`activeCondition` skips the sample and history insertion, but no reliable false-to-true
transition that clears history was observed. Consequently firmware has no reliable
client-history reset handshake, upper delay bound, or post-write watermark to key an
automatic rearm on.

The existing qualification latch is therefore preserved for composite
`LM_SD_ALPHA_N`. For the user-target blended all-authority-100 workflow, the
running-write branch in `AirmassInjectionState::onConfigurationWrite` is gated by
`isComposite(m_mode)`: it records `CalibrationWrite`, clears `veAnalyzeSessionStarted`,
and keeps the session invalid through the existing transitions. The firmware does not
force endpoint `2`, automatically rearm after a write, or rearm on a timer alone. In
that mode, a calibration write while the engine is running, including an ordinary VE
table edit, can latch `CalibrationWrite`. Fueling continues once a valid replacement
candidate is accepted; the latch prevents that candidate from silently qualifying a
new VE Analyze session.

Standalone `LM_ALPHA_N`, `LM_SPEED_DENSITY`, and `LM_REAL_MAF` do not take that latch
branch. Their running `onConfigurationWrite` path resets qualification diagnostics and
generation/session recovery instead. The TPS sample and fuel-event gates provide only
per-sample or per-event evidence; they do not guarantee that the client drops
historical rows recorded under an earlier calibration. Standalone modes are therefore
also unsupported for continuous live-write historical-row correctness, and the
composite latch does not protect them. No firmware-side fallback or recovery policy is
added for those modes, and endpoint forcing and automatic rearm remain unimplemented.

### Recovery procedure

For the composite workflow, do not continue an automatic-write analysis after a live
calibration write and expect the old client history or ECU qualification state to be
reused. The same client-history uncertainty applies to standalone modes, which are not
covered by the composite latch. The following procedure is proposed for validation:

1. Finish or stop VE Analyze, then exit TunerStudio completely. A restart discards
   process-local memory, but persisted or restored client state has not been verified.
2. Stop the engine.
3. Apply the desired tune while the engine is stopped.
4. Reopen TunerStudio and reconnect to the ECU.
5. Start a new ECU session and verify full authority/model qualification.
6. Verify model qualification plus `veAnalyzeSampleReady == 1`,
   `veAnalyzeRateValid == 1`, `veAnalyzeFuelEventReady == 1`, and valid lambda while
   holding steady operating conditions before analyzing samples.

This procedure is proposed for validation and is not evidence that client history was
reset. Returning to automatic ECU writes can latch `CalibrationWrite` again.
Continuous automatic ECU writes are not supported by this workflow.

### Validation protocol

Client-log validation should use a bench or safe stationary setup, synthetic
bounded-lambda-delay cases of `0.2`, `0.5`, and `1.0` seconds, a controlled synthetic
VE write, a short acceleration-enrichment event, a short film/wall-wetting event, and
both lambda banks. Record matching ECU telemetry, especially the accepted and
coefficient generations, temperature provenance, sample-gate fields, calibration
generation, endpoint, and first/current reject reasons. Confirm that pre-write rows do
not qualify the new session and that a completed recovery produces current,
generation-matching data for both banks.

The repository unit tests cover the deterministic ECU-side provenance sequence:
preceding accepted airflow, a subsequent VE-only publication, stale-publication
preservation, failure/reset invalidation, and RPM/TPS air-input invalidity. These are
unit tests only. Real TunerStudio client behavior, physical lambda delay, hardware
timing, and the original October 30 replay were not validated here; the replay package
was unavailable. Static bytecode observations do not prove runtime behavior beyond the
observed code paths, and the public product/definition documentation does not establish
client history-reset semantics:

- <https://www.efianalytics.com/TunerStudio/>
- <https://efianalytics.com/TunerStudio/docs/EFI%20Analytics%20ECU%20Definition%20files.pdf>
