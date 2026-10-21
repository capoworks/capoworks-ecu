# Firmware resource evidence and calibration

The firmware resource tools are a standard-library Python library and command-line interface. They do not deploy an HTTP service or promise that a build fits every runtime workload. They measure build-time technical fit from the actual ELF, generated module profile, selected toolchain, and resource policy.

## Evidence identity

An exact measurement is reusable only when all of the following identity inputs match:

- The full Git source commit, not a branch name or abbreviated SHA.
- The board id and catalog-provided CPU.
- The exact compiler binary SHA-256 and compiler version, represented by the toolchain identity digest.
- The normalized resource-policy content digest.
- The whole canonical profile: modules, cylinder-trim capacity, VE RPM count, airmass blend size, and effective engine preset selection.

Profile semantics are normalized before hashing. Module lists and preset selections are ordered canonically, and the `default_all` versus `explicit_all` spelling does not by itself create a new profile identity when the effective selection is the same. Policy requirements are sorted before hashing. The measurement context nevertheless remains exact: a changed compiler, policy content, source commit, board CPU, or effective profile requires new evidence.

## Results and capacities

| Status | Meaning |
|---|---|
| `verified` | A complete measurement with the exact requested context satisfies the active policy. |
| `incompatible` | A complete measurement with the exact context proves a resource-policy failure. |
| `requires_build` | No exact complete evidence exists, or only an estimate is available. Missing, stale, malformed, and incomplete evidence is never promoted to `verified`. |

Flash and RAM capacities are derived from ELF linker symbols and LOAD sections, not from a second capacity table. RAM aliases are reported separately and are not summed. Heap bytes are a reserved capacity, not runtime-free memory. Conservative slot policy reserves the F7 32 KiB persistent configuration slot and uses the H7 safe application boundary at `0x08100000`.

The default policy has zero technical-fit margins. A file at `misc/firmware_builder/policies/<board>.json` replaces the default policy for that board and carries its own content hash automatically. Board policies set minimum free bytes and heap capacity; they do not duplicate linker or CPU capacities.

Baseline-plus-single-feature estimates are meaningful only in the same source, board, compiler, policy, dimensions, and preset context. Cylinder trim uses six cylinders in these profiles; twelve-cylinder use is not inferred. Estimates do not account for every LTO, shared INI, alias, or optional-module interaction they have not measured. An estimate that exceeds a policy threshold is a warning and schedules a build; it is never a hard rejection.

## Calibration plan

`calibration.py plan` asks the source catalog for board mappings and optional-module support, then resolves every selected profile through the canonical Make resolver. Unknown boards, Make failures, unexpected modules, and unsupported effective options fail the whole plan instead of silently dropping rows.

The coverage choices are:

| Coverage | Bounded profiles per board |
|---|---|
| `baseline` | Six mandatory modules, 16 VE RPM points, 8-point airmass blend, no trim, default all presets. |
| `singles` | The baseline plus each supported optional module, including a six-cylinder trim profile. Capability-blocked modules are absent. |
| `full` | The baseline and all supported optional modules together. |
| `expanded` | No-preset base profiles for `16/8`, `32/8`, `16/16`, and `32/16`, plus one all-supported `32/16` no-preset trim-6 probe. |

The plan has stable board and profile ordering, URL-safe profile ids, no duplicate profile entries, and no runner-local absolute paths. Its GitHub matrix contract is:

```json
{
  "include": [
    {
      "board_id": "core8",
      "profile_id": "core8-baseline",
      "features": "",
      "ve_rpm_count": 16,
      "airmass_blend_size": 8,
      "cylinder_trim_count": 0,
      "engine_presets": "all"
    }
  ]
}
```

The record values are the effective values returned by canonical resolution, and they are passed back as the build controls.

```bash
python3 misc/firmware_builder/calibration.py plan \
  --root "$PWD" --board all --coverage singles \
  --output "$RUNNER_TEMP/calibration-matrix.json"
```

The manual `firmware-resource-matrix.yaml` workflow runs one plan job, then a maximum of three independent build jobs in parallel. The plan job checks out fixed `master`, resolves every profile, and emits one full source-commit SHA. Each build job checks out that single captured SHA, collects the actual generated profile and `firmware/build/fome.elf`, and uploads only observations, resource indices, and catalog context. It does not publish firmware.

## JSON resource API

`resources.py extract` writes one observation. A complete observation is copied into `resource-index.json.observations`; malformed or incomplete extraction evidence can only appear in `.diagnostics`. The index has stable `schema_version`, `observations`, and `diagnostics` arrays, with deterministic ordering by context, profile, status, and observation id.

`matrix.py` owns index normalization, merging, evaluation, and profile/context identity. It rejects malformed indexes rather than accepting partial evidence. Consumers should persist and merge versioned indices by source commit and board context instead of replacing older evidence wholesale.

Evaluation returns one of the three statuses above. `requires_build` may include an estimate and diagnostics, but callers must not treat an estimated policy overrun as a proven incompatibility.

## Commands

Inspect the source catalog:

```bash
python3 misc/firmware_builder/catalog.py catalog --root "$PWD"
```

Resolve one canonical profile:

```bash
python3 misc/firmware_builder/catalog.py resolve \
  --root "$PWD" --board core8 --features boost,lua \
  --ve-rpm-count 16 --airmass-blend-size 8 \
  --output "$RUNNER_TEMP/resolved-profile.json"
```

Extract the current build with the vendored toolchain:

```bash
python3 misc/firmware_builder/resources.py extract \
  --root "$PWD" --elf firmware/build/fome.elf \
  --profile firmware/generated/module_profile.txt \
  --board core8 --cpu ARCH_STM32F4 \
  --source-commit "$(git -C "$PWD" rev-parse HEAD)" \
  --policy misc/firmware_builder/policies/default.json \
  --output "$RUNNER_TEMP/resources.json"
```

Collect the current build into an artifact directory:

```bash
python3 misc/firmware_builder/calibration.py collect \
  --root "$PWD" --board core8 \
  --output "$RUNNER_TEMP/resource-observation-core8"
```

Collection writes `resources.json`, a normalized `resource-index.json`, and `catalog.json`. It reads the actual generated profile, selects the board policy when present (otherwise `policies/default.json`), uses only tools below the vendored build-tools directory, and exits nonzero when resources are rejected. It does not copy firmware into the artifact.

Merge and evaluate evidence:

```bash
python3 misc/firmware_builder/matrix.py merge \
  --observations observation-a/resource-index.json \
  observation-b/resource-index.json \
  --output resource-index.json

python3 misc/firmware_builder/matrix.py evaluate \
  --root "$PWD" --index resource-index.json \
  --board core8 --features lua \
  --source-commit "$(git -C "$PWD" rev-parse HEAD)" \
  --toolchain-id TOOLCHAIN_ID --policy-id POLICY_ID \
  --output evaluation.json
```

Package accepted custom firmware:

```bash
python3 misc/firmware_builder/pipeline.py package \
  --root "$PWD" \
  --resolved "$RUNNER_TEMP/resolved-profile.json" \
  --resources "$RUNNER_TEMP/resources.json" \
  --request-id request-123 \
  --output "$RUNNER_TEMP/package"
```

## Publication and retention

An accepted custom firmware package contains `resources.json`, `resource-index.json`, `catalog.json`, `resolved-profile.json`, and schema-2 `manifest.json` alongside its firmware artifacts. The manifest records the exact resource context and measurements.

Packaging validates the actual generated profile's board and CPU identity against the resolved request and checks the generated INI layout. When an embedded INI is available for comparison, packaging also compares it with the generated INI.

Normal `build-firmware.yaml` builds and Core8 CI collect resources after the build safety check and before packaging or firmware uploads. A rejected resource gate therefore prevents firmware publication in that job. GitHub Actions artifacts retain evidence for 14 days; they are transfer artifacts, not a database. A backend that consumes them should persist and merge versioned indices by commit and board. When an index is absent or stale for the exact request, the result is `requires_build`.

The preset catalog is global vocabulary. Its presence does not prove that every preset is wired or applicable on every engine or board. Resource results prove build-time technical fit only and are not runtime or hardware guarantees.
