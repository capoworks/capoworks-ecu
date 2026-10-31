<!--
@file ve-analyze.md
@author Giacomo di Ciocco, (C) 2026
-->

# VE Analyze qualification diagnostics

Composite SD + Alpha-N VE Analyze remains an opt-in, session-qualified feature. These
channels explain the existing qualification latch; they do not rearm it, relax it,
or change fuel admission.

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
seconds since the first cause, saturated at 65535.

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
`CalibrationWrite` even when the next valid candidate is otherwise usable.

`blendedVeAnalyzeCalibrationGeneration`, `blendedVeAnalyzeCalculationEpoch`, and
`blendedVeAnalyzeSessionGeneration` distinguish a calibration write, an
individual calculation, and the lifetime of a VE Analyze session. A session is
cleared by the existing owner Engine reset or synchronization-loss path, and by
existing stopped-engine or strategy-reset transitions.

Qualification describes model idoneity, not lambda validity. A valid lambda
reading does not make a blended authority endpoint suitable for VE analysis.

For recovery, correct the reported calibration, authority, fallback, idle-VE, or
sensor cause, then perform the normal engine stop/restart or synchronization
loss that starts a new session. Firmware does not automatically reset client
history, force endpoint `2`, or expose a rearm command.
