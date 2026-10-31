<!--
@file ve-analyze.md
@author Giacomo di Ciocco, (C) 2026
-->

# VE Analyze qualification diagnostics

Composite SD + Alpha-N VE Analyze remains an opt-in, session-qualified feature. These
channels explain the existing qualification latch; they do not rearm it, relax it,
or change fuel admission.

The firmware also owns a separate 250 Hz TPS sample gate. It is independent of the
legacy 20 Hz acceleration-enrichment update and of the composite-model qualification
latch. The gate is currently exposed as diagnostic evidence only: this change does not
alter TunerStudio filters, fuel scheduling, or the PR1 latch policy.

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
| 128 | `CalibrationWrite` | A running-engine calibration mutation invalidated the session. |
| 256 | `StrategyChange` | A running-engine strategy mutation invalidated the session. |
| 512 | `SessionAlreadyInvalid` | The current candidate did not own the latch, but the session was already invalid. |
| 1024 | `Stabilization` | Reserved; no stabilization policy is implemented. |

The first-reason priority is strategy change, calibration write, nonuniform
authority, configuration, sensor, idle VE, fallback, calculation result, stale
calculation, then session-already-invalid. A running calibration write latches
`CalibrationWrite` even when a VE or Alpha-N table cell changed to the same value,
the authority map remains uniformly valid, and the next valid candidate is
otherwise usable. The write does not by itself change accepted fuel numbers.

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

Model-qualification and lambda validity are separate concerns. A ready sample gate does
not qualify a mixed SD + Alpha-N authority map, validate lambda, or guarantee that
TunerStudio will accept a cell. TunerStudio 3.3.01 automatic ECU writes are a user-target
workflow, not a capability already supported or validated by this firmware change for
continuous VE analysis. This change provides no client handshake and does not change the
PR1 latch: an ordinary running calibration write still invalidates the session, and the
latch remains immutable until an existing session-boundary transition.

For recovery, correct the reported calibration, authority, fallback, idle-VE, or
sensor cause, then perform the normal engine stop/restart or synchronization
loss that starts a new session. Firmware does not automatically reset client history,
force endpoint `2`, expose a rearm command, rearm on a timer alone, or guarantee physical
alignment of analysis samples with lambda measurement.
