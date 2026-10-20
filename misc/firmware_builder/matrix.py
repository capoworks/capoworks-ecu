#!/usr/bin/env python3
"""Build a resource-evidence index and make conservative firmware preflights.

The index is deliberately an evidence store, not a cache of inferred truth.
Only complete, internally consistent resource reports can become exact
measurements.  Estimates are always reported as ``requires_build`` and never
promoted to ``verified``.

CLI status convention:

* ``merge`` exits 2 for an invalid input or index and leaves the output and
  any existing index untouched.
* ``evaluate`` always emits a schema-1 structured result.  Its exit code is 0
  for ``verified``, ``requires_build``, and ``incompatible`` so callers can
  inspect the result without conflating a valid rejection with a CLI error.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable

try:
    from . import catalog
except ImportError:  # Allow direct execution from this directory.
    import catalog  # type: ignore[no-redef]


SCHEMA_VERSION = 1
OBSERVATION_STATUSES = {"accepted", "rejected"}
RESULT_STATUSES = {"verified", "requires_build", "incompatible"}
BASE_MODULES = tuple(catalog.BASE_MODULES)
SHA_RE = re.compile(r"^[0-9a-fA-F]{64}$")
SOURCE_RE = re.compile(r"^(?:[0-9a-fA-F]{40}|[0-9a-fA-F]{64})$")
REGION_REQUIRED_FIELDS = {
    "origin",
    "limit",
    "capacity_bytes",
    "used_bytes",
    "free_bytes",
    "required_free_bytes",
    "status",
}
EXPECTED_REGION_NAMES = frozenset(
    {"flash", *(f"ram{index}" for index in range(8))}
)
# These codes are emitted by resources.extract after comparing ELF LOAD-section
# ranges with the bootloader and configuration reservations.  They are the
# only violation-only evidence that can establish a flash reservation failure.
FLASH_LOAD_RESERVATION_FAILURE_CODES = frozenset(
    {
        "bootloader_section_overlap",
        "flash_load_in_reserved_range",
        "flash_load_overlaps_config",
        "flash_symbol_free_before_load",
    }
)


class MatrixError(ValueError):
    """A report, request, or index violates the matrix contract."""


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _is_int(value: Any) -> bool:
    return type(value) is int


def _integer(value: Any, label: str, *, minimum: int | None = 0) -> int:
    if not _is_int(value):
        raise MatrixError(f"{label} must be an integer")
    if minimum is not None and value < minimum:
        raise MatrixError(f"{label} must be at least {minimum}")
    return value


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise MatrixError(f"{label} must be a non-empty string")
    if any(character in value for character in ("\x00", "\n", "\r")):
        raise MatrixError(f"{label} must be a single safe line")
    return value


def _sha256(value: Any, label: str, *, optional: bool = False) -> str | None:
    if value is None and optional:
        return None
    if not isinstance(value, str) or not SHA_RE.fullmatch(value):
        raise MatrixError(f"{label} must be a 64-character hexadecimal SHA-256")
    return value.lower()


def _source_commit(value: Any) -> str:
    if not isinstance(value, str) or not SOURCE_RE.fullmatch(value):
        raise MatrixError(
            "source_commit must be a 40- or 64-character hexadecimal SHA"
        )
    return value.lower()


def _string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list):
        raise MatrixError(f"{label} must be a list")
    result = []
    for index, item in enumerate(value):
        result.append(_text(item, f"{label}[{index}]"))
    if len(result) != len(set(result)):
        raise MatrixError(f"{label} must contain unique values")
    return sorted(result)


def normalize_profile(profile: Any) -> dict[str, Any]:
    """Validate and canonicalize a complete resolved module profile."""
    if not isinstance(profile, dict):
        raise MatrixError("profile must be an object")
    required = {
        "modules",
        "ve_rpm_count",
        "airmass_blend_size",
        "cylinder_trim_count",
        "engine_preset_mode",
        "engine_presets",
        "engine_preset_default",
    }
    missing = sorted(required - set(profile))
    if missing:
        raise MatrixError(f"profile is missing fields: {', '.join(missing)}")
    unknown = sorted(set(profile) - required)
    if unknown:
        raise MatrixError(f"profile has unknown fields: {', '.join(unknown)}")

    modules = _string_list(profile["modules"], "profile.modules")
    if not modules:
        raise MatrixError("profile.modules must not be empty")
    trim_count = _integer(
        profile["cylinder_trim_count"], "profile.cylinder_trim_count"
    )
    ve_rpm_count = _integer(profile["ve_rpm_count"], "profile.ve_rpm_count")
    airmass_blend_size = _integer(
        profile["airmass_blend_size"], "profile.airmass_blend_size"
    )
    preset_mode = _text(
        profile["engine_preset_mode"], "profile.engine_preset_mode"
    )
    if preset_mode not in catalog.ENGINE_PRESET_MODES:
        raise MatrixError(f"unknown profile.engine_preset_mode: {preset_mode}")
    presets = _string_list(profile["engine_presets"], "profile.engine_presets")
    if not presets:
        raise MatrixError("profile.engine_presets must not be empty")
    preset_default = _text(
        profile["engine_preset_default"], "profile.engine_preset_default"
    )
    if "cylinder_fuel_trim" in modules:
        if trim_count < 1 or trim_count > 12:
            raise MatrixError(
                "profile.cylinder_trim_count must be 1-12 when "
                "cylinder_fuel_trim is selected"
            )
    elif trim_count != 0:
        raise MatrixError(
            "profile.cylinder_trim_count must be 0 when cylinder_fuel_trim "
            "is not selected"
        )

    # The mode spelling is intentionally not part of the semantic identity.
    # Two resolved profiles select the same presets and default even when one
    # report calls that selection default_all and another explicit_all.
    return {
        "modules": modules,
        "ve_rpm_count": ve_rpm_count,
        "airmass_blend_size": airmass_blend_size,
        "cylinder_trim_count": trim_count,
        "engine_preset_mode": preset_mode,
        "engine_presets": presets,
        "engine_preset_default": preset_default,
    }


def profile_key(profile: Any) -> str:
    """Return the semantic profile identity, excluding preset-mode spelling."""
    normalized = normalize_profile(profile)
    semantic = {
        key: value
        for key, value in normalized.items()
        if key != "engine_preset_mode"
    }
    return hashlib.sha256(_canonical_json(semantic).encode("utf-8")).hexdigest()


def _fixed_profile_key(profile: dict[str, Any]) -> str:
    """Identity shared by a baseline and its single-feature variants."""
    fixed = {
        "ve_rpm_count": profile["ve_rpm_count"],
        "airmass_blend_size": profile["airmass_blend_size"],
        "engine_presets": profile["engine_presets"],
        "engine_preset_default": profile["engine_preset_default"],
    }
    return hashlib.sha256(_canonical_json(fixed).encode("utf-8")).hexdigest()


def context_key(
    board_id: str,
    cpu: str,
    source_commit: str,
    toolchain_id: str,
    policy_id: str,
    profile: Any,
) -> str:
    """Return the full exact-measurement context key."""
    identity = {
        "board_id": _text(board_id, "board_id"),
        "cpu": _text(cpu, "cpu"),
        "source_commit": _source_commit(source_commit),
        "toolchain_id": _text(toolchain_id, "toolchain id"),
        "policy_id": _text(policy_id, "policy id"),
        "profile_key": profile_key(profile),
    }
    return hashlib.sha256(_canonical_json(identity).encode("utf-8")).hexdigest()


def _normalize_toolchain(raw: Any) -> dict[str, str | None]:
    if not isinstance(raw, dict):
        raise MatrixError("toolchain must be an object")
    toolchain_id = _text(raw.get("id"), "toolchain.id")
    version = raw.get("version")
    if version is not None:
        version = _text(version, "toolchain.version")
    sha256 = _sha256(raw.get("sha256"), "toolchain.sha256", optional=True)
    return {"id": toolchain_id, "version": version, "sha256": sha256}


def _policy_content_id(
    required_free_bytes: dict[str, int],
    minimum_heap_capacity_bytes: int,
) -> str:
    identity = {
        "schema_version": SCHEMA_VERSION,
        "required_free_bytes": dict(sorted(required_free_bytes.items())),
        "minimum_heap_capacity_bytes": minimum_heap_capacity_bytes,
    }
    return hashlib.sha256(_canonical_json(identity).encode("utf-8")).hexdigest()


def normalize_policy(raw: Any) -> dict[str, Any]:
    """Validate a schema-1 policy and verify its immutable content id."""
    if not isinstance(raw, dict):
        raise MatrixError("policy must be an object")
    allowed = {
        "id",
        "schema_version",
        "required_free_bytes",
        "minimum_heap_capacity_bytes",
        "error",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise MatrixError(f"policy has unknown fields: {', '.join(unknown)}")
    schema_version = raw.get("schema_version")
    if not _is_int(schema_version) or schema_version != SCHEMA_VERSION:
        raise MatrixError("policy.schema_version must be integer 1")
    policy_id = _sha256(raw.get("id"), "policy.id")
    requirements = raw.get("required_free_bytes", {})
    if not isinstance(requirements, dict):
        raise MatrixError("policy.required_free_bytes must be an object")
    normalized_requirements = {}
    for region, amount in requirements.items():
        region = _text(region, "policy region")
        normalized_requirements[region] = _integer(
            amount, f"policy.required_free_bytes[{region!r}]"
        )
    minimum_heap = _integer(
        raw.get("minimum_heap_capacity_bytes", 0),
        "policy.minimum_heap_capacity_bytes",
    )
    expected_id = _policy_content_id(normalized_requirements, minimum_heap)
    if policy_id != expected_id:
        raise MatrixError(
            "policy.id must equal the SHA-256 content hash of the normalized policy"
        )
    result = {
        "id": policy_id,
        "schema_version": SCHEMA_VERSION,
        "required_free_bytes": dict(sorted(normalized_requirements.items())),
        "minimum_heap_capacity_bytes": minimum_heap,
    }
    if "error" in raw:
        result["error"] = _text(raw["error"], "policy.error")
    return result


def _normalize_violations(raw: Any) -> list[dict[str, str]]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise MatrixError("violations must be a list")
    result = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise MatrixError(f"violations[{index}] must be an object")
        code = _text(item.get("code"), f"violations[{index}].code")
        message = _text(item.get("message"), f"violations[{index}].message")
        result.append({"code": code, "message": message})
    return sorted(result, key=lambda item: (item["code"], item["message"]))


def _normalize_region(name: str, raw: Any) -> tuple[dict[str, Any] | None, bool]:
    if not isinstance(raw, dict):
        raise MatrixError(f"regions[{name!r}] must be an object")
    if not REGION_REQUIRED_FIELDS.issubset(raw):
        return None, False
    origin = _integer(raw["origin"], f"regions[{name!r}].origin")
    limit = _integer(raw["limit"], f"regions[{name!r}].limit")
    capacity = _integer(
        raw["capacity_bytes"], f"regions[{name!r}].capacity_bytes"
    )
    used = _integer(raw["used_bytes"], f"regions[{name!r}].used_bytes")
    free = _integer(
        raw["free_bytes"], f"regions[{name!r}].free_bytes", minimum=None
    )
    required = _integer(
        raw["required_free_bytes"], f"regions[{name!r}].required_free_bytes"
    )
    status = _text(raw["status"], f"regions[{name!r}].status")
    if status not in OBSERVATION_STATUSES:
        raise MatrixError(f"regions[{name!r}].status is invalid")
    if limit < origin:
        raise MatrixError(f"regions[{name!r}] limit precedes origin")
    if capacity != limit - origin:
        raise MatrixError(
            f"regions[{name!r}].capacity_bytes must equal limit - origin"
        )
    if free != capacity - used:
        raise MatrixError(
            f"regions[{name!r}].free_bytes must equal capacity_bytes - used_bytes"
        )
    return {
        "origin": origin,
        "limit": limit,
        "capacity_bytes": capacity,
        "used_bytes": used,
        "free_bytes": free,
        "required_free_bytes": required,
        "status": status,
    }, True


def _normalize_heap(
    raw: Any, policy: dict[str, Any]
) -> tuple[dict[str, Any] | None, bool]:
    if not isinstance(raw, dict):
        raise MatrixError("heap must be an object")
    if not raw:
        return None, False
    if "capacity_bytes" not in raw or "status" not in raw:
        return None, False
    capacity = _integer(raw["capacity_bytes"], "heap.capacity_bytes")
    status = _text(raw["status"], "heap.status")
    if status not in OBSERVATION_STATUSES:
        raise MatrixError("heap.status is invalid")
    required = _integer(
        raw.get(
            "required_capacity_bytes",
            policy["minimum_heap_capacity_bytes"],
        ),
        "heap.required_capacity_bytes",
    )
    if required != policy["minimum_heap_capacity_bytes"]:
        raise MatrixError(
            "heap.required_capacity_bytes contradicts policy.minimum_heap_capacity_bytes"
        )
    result: dict[str, Any] = {
        "capacity_bytes": capacity,
        "required_capacity_bytes": required,
        "status": status,
    }
    for field in ("origin", "limit"):
        if field in raw:
            result[field] = _integer(raw[field], f"heap.{field}")
    if "origin" in result and "limit" in result:
        if result["limit"] < result["origin"]:
            raise MatrixError("heap.limit precedes heap.origin")
        if capacity != result["limit"] - result["origin"]:
            raise MatrixError("heap.capacity_bytes must equal limit - origin")
    return result, True


def _normalize_persistent(
    raw: Any, policy: dict[str, Any]
) -> tuple[dict[str, Any] | None, bool]:
    if not isinstance(raw, dict):
        raise MatrixError("persistent must be an object")
    if not raw:
        return None, False
    required_fields = {
        "used_bytes",
        "capacity_bytes",
        "free_bytes",
        "status",
    }
    if not required_fields.issubset(raw):
        return None, False
    used = _integer(raw["used_bytes"], "persistent.used_bytes")
    capacity = _integer(raw["capacity_bytes"], "persistent.capacity_bytes")
    free = _integer(raw["free_bytes"], "persistent.free_bytes", minimum=None)
    required = _integer(
        raw.get(
            "required_free_bytes",
            policy["required_free_bytes"].get("persistent", 0),
        ),
        "persistent.required_free_bytes",
    )
    status = _text(raw["status"], "persistent.status")
    if status not in OBSERVATION_STATUSES:
        raise MatrixError("persistent.status is invalid")
    if free != capacity - used:
        raise MatrixError(
            "persistent.free_bytes must equal capacity_bytes - used_bytes"
        )
    expected_required = policy["required_free_bytes"].get("persistent", 0)
    if required != expected_required:
        raise MatrixError(
            "persistent.required_free_bytes contradicts the policy"
        )
    result: dict[str, Any] = {
        "used_bytes": used,
        "capacity_bytes": capacity,
        "free_bytes": free,
        "required_free_bytes": required,
        "status": status,
    }
    if "ram_region" in raw:
        result["ram_region"] = _text(raw["ram_region"], "persistent.ram_region")
    return result, True


def _normalize_metrics(
    report: dict[str, Any], policy: dict[str, Any]
) -> dict[str, Any]:
    raw_regions = report.get("regions", {})
    if not isinstance(raw_regions, dict):
        raise MatrixError("regions must be an object")
    regions = {}
    regions_complete = set(raw_regions) == EXPECTED_REGION_NAMES
    for name, raw_region in raw_regions.items():
        name = _text(name, "region name")
        if name not in EXPECTED_REGION_NAMES:
            raise MatrixError(f"unknown region name: {name}")
        normalized, complete = _normalize_region(name, raw_region)
        regions_complete = regions_complete and complete
        if normalized is None:
            continue
        expected_required = policy["required_free_bytes"].get(name, 0)
        if normalized["required_free_bytes"] != expected_required:
            raise MatrixError(
                f"regions[{name!r}].required_free_bytes contradicts the policy"
            )
        regions[name] = normalized

    heap, heap_complete = _normalize_heap(report.get("heap", {}), policy)
    persistent, persistent_complete = _normalize_persistent(
        report.get("persistent", {}), policy
    )
    return {
        "regions": dict(sorted(regions.items())),
        "heap": heap,
        "persistent": persistent,
        "regions_complete": regions_complete,
        "heap_complete": heap_complete,
        "persistent_complete": persistent_complete,
    }


def _validate_accepted_metrics(record: dict[str, Any]) -> None:
    if record["status"] != "accepted":
        return
    if not record["complete"]:
        raise MatrixError("an accepted report must contain complete metrics")
    if record["violations"] or record["policy"].get("error"):
        raise MatrixError("an accepted report must not contain violations")
    for name, region in record["regions"].items():
        if region["status"] != "accepted":
            raise MatrixError(
                f"accepted report has rejected region {name!r}"
            )
        if region["free_bytes"] < region["required_free_bytes"]:
            raise MatrixError(
                f"accepted region {name!r} violates its policy minimum"
            )
    heap = record["heap"]
    if heap["status"] != "accepted":
        raise MatrixError("accepted report has a rejected heap")
    if heap["capacity_bytes"] < heap["required_capacity_bytes"]:
        raise MatrixError("accepted heap violates its policy minimum")
    persistent = record["persistent"]
    if persistent["status"] != "accepted":
        raise MatrixError("accepted report has rejected persistent storage")
    if persistent["free_bytes"] < persistent["required_free_bytes"]:
        raise MatrixError("accepted persistent storage violates its policy minimum")


def _validate_metric_statuses(record: dict[str, Any]) -> None:
    """Reject per-component statuses contradicted by their own metrics."""
    for name, region in record["regions"].items():
        if (
            region["status"] == "accepted"
            and region["free_bytes"] < region["required_free_bytes"]
        ):
            raise MatrixError(
                f"region {name!r} is accepted below its policy minimum"
            )
    heap = record["heap"]
    if (
        heap
        and heap["status"] == "accepted"
        and heap["capacity_bytes"] < heap["required_capacity_bytes"]
    ):
        raise MatrixError("heap is accepted below its policy minimum")
    persistent = record["persistent"]
    if (
        persistent
        and persistent["status"] == "accepted"
        and persistent["free_bytes"] < persistent["required_free_bytes"]
    ):
        raise MatrixError("persistent storage is accepted below its policy minimum")


def _validate_rejected_status(record: dict[str, Any]) -> None:
    """A rejected measurement needs concrete evidence, not just a label."""
    if record["status"] != "rejected":
        return
    if record["complete"] and not _rejected_measurement_eligible(record):
        raise MatrixError(
            "a complete rejected report must prove a resource failure"
        )
    if record["violations"]:
        return
    components = list(record["regions"].values())
    if record["heap"]:
        components.append(record["heap"])
    if record["persistent"]:
        components.append(record["persistent"])
    if components and all(item["status"] == "accepted" for item in components):
        raise MatrixError(
            "a rejected report must identify a failed component or violation"
        )


def _rejected_measurement_eligible(record: dict[str, Any]) -> bool:
    """Return whether normalized evidence proves a resource failure."""
    for region in record["regions"].values():
        if region["free_bytes"] < region["required_free_bytes"]:
            return True
    heap = record["heap"]
    if heap and heap["capacity_bytes"] < heap["required_capacity_bytes"]:
        return True
    persistent = record["persistent"]
    if persistent and (
        persistent["free_bytes"] < persistent["required_free_bytes"]
    ):
        return True
    return any(
        violation["code"] in FLASH_LOAD_RESERVATION_FAILURE_CODES
        for violation in record["violations"]
    )


def normalize_report(report: Any) -> dict[str, Any]:
    """Validate one resources.extract schema-1 report for the cache."""
    if not isinstance(report, dict):
        raise MatrixError("observation must be an object")
    schema_version = report.get("schema_version")
    if not _is_int(schema_version) or schema_version != SCHEMA_VERSION:
        raise MatrixError("observation schema_version must be integer 1")

    board_id = _text(report.get("board_id"), "board_id")
    cpu = _text(report.get("cpu"), "cpu")
    source_commit = _source_commit(report.get("source_commit"))
    status = _text(report.get("status"), "status")
    if status not in OBSERVATION_STATUSES:
        raise MatrixError("status must be accepted or rejected")
    toolchain = _normalize_toolchain(report.get("toolchain"))
    policy = normalize_policy(report.get("policy"))
    profile = normalize_profile(report.get("profile"))
    violations = _normalize_violations(report.get("violations"))

    elf_sha256 = report.get("elf_sha256")
    if elf_sha256 is not None:
        elf_sha256 = _sha256(elf_sha256, "elf_sha256")
    metrics = _normalize_metrics(report, policy)
    regions = metrics["regions"]
    region_names = set(regions)
    unknown_policy_regions = sorted(
        set(policy["required_free_bytes"]) - region_names - {"persistent"}
    )
    if metrics["regions_complete"] and unknown_policy_regions:
        raise MatrixError(
            "policy references regions absent from complete metrics: "
            + ", ".join(unknown_policy_regions)
        )
    structurally_complete = all(
        (
            metrics["regions_complete"],
            metrics["heap_complete"],
            metrics["persistent_complete"],
            set(regions) == EXPECTED_REGION_NAMES,
            elf_sha256 is not None,
            toolchain["version"] is not None,
            toolchain["sha256"] is not None,
            "error" not in policy,
        )
    )

    record: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "board_id": board_id,
        "cpu": cpu,
        "source_commit": source_commit,
        "toolchain": toolchain,
        "profile": profile,
        "elf_sha256": elf_sha256,
        "regions": regions,
        "heap": metrics["heap"] or {},
        "persistent": metrics["persistent"] or {},
        "policy": policy,
        "status": status,
        "violations": violations,
        "complete": False,
        "profile_key": profile_key(profile),
        "context_key": context_key(
            board_id,
            cpu,
            source_commit,
            toolchain["id"],
            policy["id"],
            profile,
        ),
    }
    complete = structurally_complete and (
        status == "accepted" or _rejected_measurement_eligible(record)
    )
    record["complete"] = complete
    record["observation_kind"] = (
        "measurement" if complete else "diagnostic"
    )
    _validate_metric_statuses(record)
    _validate_accepted_metrics(record)
    _validate_rejected_status(record)
    identity = {key: value for key, value in record.items() if key != "observation_id"}
    record["observation_id"] = hashlib.sha256(
        _canonical_json(identity).encode("utf-8")
    ).hexdigest()
    return record


def _empty_index() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "observations": [],
        "diagnostics": [],
    }


def normalize_index(index: Any) -> dict[str, Any]:
    """Strictly validate an in-memory index and return its canonical form."""
    if not isinstance(index, dict):
        raise MatrixError("resource index must be an object")
    schema_version = index.get("schema_version")
    if not _is_int(schema_version) or schema_version != SCHEMA_VERSION:
        raise MatrixError("resource index schema_version must be integer 1")
    allowed = {"schema_version", "observations", "diagnostics"}
    unknown = sorted(set(index) - allowed)
    if unknown:
        raise MatrixError(
            f"resource index has unknown fields: {', '.join(unknown)}"
        )
    observations = index.get("observations", [])
    diagnostics = index.get("diagnostics", [])
    if not isinstance(observations, list) or not isinstance(diagnostics, list):
        raise MatrixError("index observations and diagnostics must be lists")

    normalized_observations = []
    normalized_diagnostics = []
    for item in observations:
        record = normalize_report(item)
        if not record["complete"]:
            raise MatrixError("index observations must contain complete measurements")
        normalized_observations.append(record)
    for item in diagnostics:
        record = normalize_report(item)
        if record["complete"]:
            raise MatrixError("index diagnostics must contain incomplete records")
        normalized_diagnostics.append(record)

    return {
        "schema_version": SCHEMA_VERSION,
        "observations": sorted(
            normalized_observations,
            key=lambda record: (
                record["context_key"],
                record["profile_key"],
                record["status"],
                record["observation_id"],
            ),
        ),
        "diagnostics": sorted(
            normalized_diagnostics,
            key=lambda record: (
                record["context_key"],
                record["profile_key"],
                record["observation_id"],
            ),
        ),
    }


def merge_reports(
    reports: Iterable[Any], existing: Any | None = None
) -> dict[str, Any]:
    """Return a deterministic index containing old and new evidence."""
    if existing is None:
        document = _empty_index()
    else:
        document = normalize_index(existing)

    observations = {
        record["observation_id"]: record for record in document["observations"]
    }
    diagnostics = {
        record["observation_id"]: record for record in document["diagnostics"]
    }
    for report in reports:
        record = normalize_report(report)
        target = observations if record["complete"] else diagnostics
        target[record["observation_id"]] = record
    return normalize_index(
        {
            "schema_version": SCHEMA_VERSION,
            "observations": list(observations.values()),
            "diagnostics": list(diagnostics.values()),
        }
    )


def _read_json(path: str | Path) -> Any:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read {path}: {error}") from error


def load_index(path: str | Path) -> dict[str, Any]:
    return normalize_index(_read_json(path))


def _atomic_json(path: str | Path, document: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(document, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _result(
    status: str,
    reason: str,
    *,
    board_id: str | None,
    profile: dict[str, Any] | None,
    evidence_context: dict[str, Any] | None,
    message: str,
    diagnostics: list[dict[str, str]] | None = None,
) -> dict[str, Any]:
    if status not in RESULT_STATUSES:
        raise MatrixError(f"invalid result status {status!r}")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "reason": reason,
        "message": message,
        "board_id": board_id,
        "profile": profile,
        "evidence_context": evidence_context or {},
        "diagnostics": diagnostics or [],
    }


def _request_identity(
    resolved_request: Any,
    source_commit: Any,
    toolchain_id: Any,
    policy_id: Any,
) -> dict[str, Any]:
    if not isinstance(resolved_request, dict):
        raise MatrixError("resolved_request must be an object")
    board_id = _text(resolved_request.get("board_id"), "resolved_request.board_id")
    cpu = _text(resolved_request.get("cpu"), "resolved_request.cpu")
    profile = normalize_profile(resolved_request.get("profile"))
    source = _source_commit(source_commit)
    compiler = _text(toolchain_id, "toolchain_id")
    policy = _text(policy_id, "policy_id")
    return {
        "board_id": board_id,
        "cpu": cpu,
        "profile": profile,
        "source_commit": source,
        "toolchain_id": compiler,
        "policy_id": policy,
        "profile_key": profile_key(profile),
        "context_key": context_key(
            board_id, cpu, source, compiler, policy, profile
        ),
    }


def _evidence_context(identity: dict[str, Any], record_ids: list[str]) -> dict[str, Any]:
    return {
        "board_id": identity["board_id"],
        "cpu": identity["cpu"],
        "source_commit": identity["source_commit"],
        "toolchain_id": identity["toolchain_id"],
        "policy_id": identity["policy_id"],
        "profile_key": identity["profile_key"],
        "context_key": identity["context_key"],
        "observation_ids": sorted(record_ids),
    }


def _unavailable_estimate(reason: str, message: str) -> dict[str, Any]:
    return {
        "kind": "single_feature_delta_model",
        "known": False,
        "coverage_complete": False,
        "unknown": {"reason": reason, "message": message},
        "warnings": [
            "No usable resource evidence is present for this full context."
        ],
    }


def _measurement_signature(record: dict[str, Any]) -> str:
    measurable = {
        "status": record["status"],
        "regions": record["regions"],
        "heap": record["heap"],
        "persistent": record["persistent"],
        "policy": record["policy"],
    }
    return _canonical_json(measurable)


def _measurement(records: list[dict[str, Any]]) -> dict[str, Any]:
    first = records[0]
    return {
        "kind": "exact_resource_measurement",
        "status": first["status"],
        "sample_count": len(records),
        "profile": first["profile"],
        "elf_sha256": sorted(
            {
                record["elf_sha256"]
                for record in records
                if record["elf_sha256"] is not None
            }
        ),
        "regions": first["regions"],
        "heap": first["heap"],
        "persistent": first["persistent"],
        "policy": first["policy"],
    }


def _common_diagnostics(records: Iterable[dict[str, Any]]) -> list[dict[str, str]]:
    diagnostics = []
    seen = set()
    for record in records:
        for violation in record["violations"]:
            key = (violation["code"], violation["message"])
            if key not in seen:
                seen.add(key)
                diagnostics.append(dict(violation))
    return diagnostics


def _variant_delta(
    baseline: dict[str, Any], variant: dict[str, Any]
) -> dict[str, Any]:
    if baseline["policy"] != variant["policy"]:
        raise MatrixError("variant policy contents differ under the same policy id")
    baseline_regions = baseline["regions"]
    variant_regions = variant["regions"]
    if set(baseline_regions) != set(variant_regions):
        raise MatrixError("variant changes the regional bank layout")
    region_deltas = {}
    for name, base_region in baseline_regions.items():
        variant_region = variant_regions[name]
        for field in ("origin", "limit", "capacity_bytes", "required_free_bytes"):
            if base_region[field] != variant_region[field]:
                raise MatrixError(
                    f"variant changes region {name!r} field {field}"
                )
        region_deltas[name] = (
            variant_region["used_bytes"] - base_region["used_bytes"]
        )
    if (
        baseline["persistent"]["capacity_bytes"]
        != variant["persistent"]["capacity_bytes"]
    ):
        raise MatrixError("variant changes persistent capacity")
    return {
        "regions_used_bytes": dict(sorted(region_deltas.items())),
        "persistent_used_bytes": (
            variant["persistent"]["used_bytes"]
            - baseline["persistent"]["used_bytes"]
        ),
        "heap_capacity_bytes": (
            variant["heap"]["capacity_bytes"]
            - baseline["heap"]["capacity_bytes"]
        ),
    }


def _select_consistent(
    records: list[dict[str, Any]],
    signature,
) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    if not records:
        return None, []
    signatures = {signature(record) for record in records}
    if len(signatures) != 1:
        return None, records
    return records[0], records


def _predict_metrics(
    baseline: dict[str, Any],
    features: list[str],
    deltas: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    region_totals = {
        name: 0 for name in baseline["regions"]
    }
    persistent_total = 0
    heap_total = 0
    for feature in features:
        delta = deltas[feature]
        for name, amount in delta["regions_used_bytes"].items():
            region_totals[name] += amount
        persistent_total += delta["persistent_used_bytes"]
        heap_total += delta["heap_capacity_bytes"]

    regions = {}
    overbudget = []
    for name, base_region in baseline["regions"].items():
        used = base_region["used_bytes"] + region_totals[name]
        free = base_region["capacity_bytes"] - used
        within = free >= base_region["required_free_bytes"]
        if not within:
            overbudget.append(f"regions.{name}.free_bytes")
        regions[name] = {
            "origin": base_region["origin"],
            "limit": base_region["limit"],
            "capacity_bytes": base_region["capacity_bytes"],
            "estimated_used_bytes": used,
            "estimated_free_bytes": free,
            "required_free_bytes": base_region["required_free_bytes"],
            "estimated_within_policy": within,
        }

    persistent_used = (
        baseline["persistent"]["used_bytes"] + persistent_total
    )
    persistent_free = baseline["persistent"]["capacity_bytes"] - persistent_used
    persistent_within = (
        persistent_free >= baseline["persistent"]["required_free_bytes"]
    )
    if not persistent_within:
        overbudget.append("persistent.free_bytes")
    heap_capacity = baseline["heap"]["capacity_bytes"] + heap_total
    heap_within = (
        heap_capacity >= baseline["heap"]["required_capacity_bytes"]
    )
    if not heap_within:
        overbudget.append("heap.capacity_bytes")
    return {
        "regions": dict(sorted(regions.items())),
        "heap": {
            "estimated_capacity_bytes": heap_capacity,
            "required_capacity_bytes": baseline["heap"][
                "required_capacity_bytes"
            ],
            "estimated_within_policy": heap_within,
        },
        "persistent": {
            "estimated_used_bytes": persistent_used,
            "capacity_bytes": baseline["persistent"]["capacity_bytes"],
            "estimated_free_bytes": persistent_free,
            "required_free_bytes": baseline["persistent"][
                "required_free_bytes"
            ],
            "estimated_within_policy": persistent_within,
        },
        "predicted_overbudget_components": overbudget,
        "predicted_overbudget": bool(overbudget),
    }


def _interaction_diagnostics(
    records: list[dict[str, Any]],
    baseline: dict[str, Any],
    deltas: dict[str, dict[str, Any]],
    trim_count: int,
) -> list[dict[str, Any]]:
    diagnostics = []
    for record in records:
        record_trim_count = (
            trim_count
            if "cylinder_fuel_trim" in record["profile"]["modules"]
            else 0
        )
        if record["profile"]["cylinder_trim_count"] != record_trim_count:
            continue
        features = [
            module
            for module in record["profile"]["modules"]
            if module not in BASE_MODULES
        ]
        if len(features) < 2 or not set(features).issubset(deltas):
            continue
        try:
            predicted = _predict_metrics(baseline, features, deltas)
        except (KeyError, MatrixError):
            continue
        residual = {}
        if set(predicted["regions"]) == set(record["regions"]):
            for name, observed in record["regions"].items():
                residual[f"regions.{name}.used_bytes"] = (
                    observed["used_bytes"]
                    - predicted["regions"][name]["estimated_used_bytes"]
                )
        residual["persistent.used_bytes"] = (
            record["persistent"]["used_bytes"]
            - predicted["persistent"]["estimated_used_bytes"]
        )
        residual["heap.capacity_bytes"] = (
            record["heap"]["capacity_bytes"]
            - predicted["heap"]["estimated_capacity_bytes"]
        )
        diagnostics.append(
            {
                "observation_id": record["observation_id"],
                "features": sorted(features),
                "residual_observed_minus_estimate": dict(sorted(residual.items())),
                "note": (
                    "Residual is diagnostic only; no interaction fit, bound, "
                    "or confidence estimate is inferred."
                ),
            }
        )
    return diagnostics


def _estimate(
    index: dict[str, Any], identity: dict[str, Any]
) -> dict[str, Any]:
    target = identity["profile"]
    target_optional = [
        module for module in target["modules"] if module not in BASE_MODULES
    ]
    same_context = [
        record
        for record in index["observations"]
        if record["context_key"] != identity["context_key"]
        and all(
            record[field] == identity[field]
            for field in (
                "board_id",
                "cpu",
                "source_commit",
            )
        )
        and record["toolchain"]["id"] == identity["toolchain_id"]
        and record["policy"]["id"] == identity["policy_id"]
        and record["status"] == "accepted"
    ]
    # context_key includes modules, so select using the explicit identity
    # fields above rather than requiring the target profile key here.
    fixed_key = _fixed_profile_key(target)
    same_fixed = [
        record
        for record in same_context
        if _fixed_profile_key(record["profile"]) == fixed_key
    ]
    baseline_candidates = [
        record
        for record in same_fixed
        if set(record["profile"]["modules"]) == set(BASE_MODULES)
        and record["profile"]["cylinder_trim_count"] == 0
    ]
    baseline, baseline_records = _select_consistent(
        baseline_candidates, _measurement_signature
    )
    estimate_base = {
        "kind": "single_feature_delta_model",
        "known": False,
        "coverage_complete": False,
        "target_optional_modules": sorted(target_optional),
        "warnings": [
            "Single-feature deltas are not guaranteed additive under LTO and "
            "shared INI/configuration generation."
        ],
    }
    if baseline is None:
        estimate_base["unknown"] = {
            "reason": (
                "conflicting_baselines"
                if baseline_records
                else "missing_mandatory_baseline"
            ),
            "required_modules": list(BASE_MODULES),
        }
        return estimate_base

    variants = {}
    missing = []
    conflicts = []
    evidence = {}
    for feature in target_optional:
        feature_trim_count = (
            target["cylinder_trim_count"]
            if feature == "cylinder_fuel_trim"
            else 0
        )
        candidates = [
            record
            for record in same_fixed
            if set(record["profile"]["modules"])
            == set(BASE_MODULES) | {feature}
            and record["profile"]["cylinder_trim_count"] == feature_trim_count
        ]
        compatible = []
        signatures = set()
        incompatible_layout = False
        for record in candidates:
            try:
                delta = _variant_delta(baseline, record)
            except MatrixError:
                incompatible_layout = True
                break
            compatible.append(record)
            signatures.add(_canonical_json(delta))
        selected_records = compatible
        selected = (
            compatible[0]
            if compatible and len(signatures) == 1
            else None
        )
        if selected is None:
            if selected_records or incompatible_layout:
                if feature not in conflicts:
                    conflicts.append(feature)
            else:
                missing.append(feature)
            continue
        try:
            delta = _variant_delta(baseline, selected)
        except MatrixError:
            missing.append(feature)
            continue
        variants[feature] = delta
        evidence[feature] = {
            "observation_ids": sorted(
                record["observation_id"] for record in selected_records
            ),
            "elf_sha256": sorted(
                {
                    record["elf_sha256"]
                    for record in selected_records
                    if record["elf_sha256"] is not None
                }
            ),
        }

    if missing or conflicts:
        estimate_base["unknown"] = {
            "reason": "incomplete_single_feature_coverage",
            "missing_features": sorted(missing),
            "conflicting_features": sorted(conflicts),
        }
        return estimate_base

    try:
        predicted = _predict_metrics(baseline, target_optional, variants)
    except (KeyError, MatrixError) as error:
        estimate_base["unknown"] = {
            "reason": "incompatible_variant_layout",
            "message": str(error),
        }
        return estimate_base

    result = dict(estimate_base)
    result.update(
        {
            "known": True,
            "coverage_complete": True,
            "baseline_observation_ids": sorted(
                record["observation_id"] for record in baseline_records
            ),
            "baseline_profile": baseline["profile"],
            "deltas_by_feature": dict(sorted(variants.items())),
            "feature_evidence": evidence,
            "summed_deltas": {
                "regions_used_bytes": {
                    name: sum(
                        variants[feature]["regions_used_bytes"][name]
                        for feature in target_optional
                    )
                    for name in baseline["regions"]
                },
                "persistent_used_bytes": sum(
                    variants[feature]["persistent_used_bytes"]
                    for feature in target_optional
                ),
                "heap_capacity_bytes": sum(
                    variants[feature]["heap_capacity_bytes"]
                    for feature in target_optional
                ),
            },
            "predicted": predicted,
            "interaction_diagnostics": _interaction_diagnostics(
                same_fixed, baseline, variants, target["cylinder_trim_count"]
            ),
            "scope": (
                "Modules-only model at fixed VE RPM count, airmass blend size, "
                "engine preset selection/default, and cylinder trim count."
            ),
        }
    )
    result["warnings"].append(
        "Predicted budget exhaustion is a warning, not observed evidence of "
        "a failed build."
    )
    result["warnings"].append(
        "RAM alias banks are accounted for separately and are never aggregated."
    )
    return result


def _invalid_request_result(
    *,
    board_id: str | None,
    source_commit: str | None,
    toolchain_id: str | None,
    policy_id: str | None,
    message: str,
) -> dict[str, Any]:
    return _result(
        "incompatible",
        "invalid_canonical_request",
        board_id=board_id,
        profile=None,
        evidence_context={
            "board_id": board_id,
            "source_commit": source_commit,
            "toolchain_id": toolchain_id,
            "policy_id": policy_id,
        },
        message=message,
        diagnostics=[{"code": "invalid_canonical_request", "message": message}],
    )


def evaluate(
    index: Any,
    resolved_request: Any,
    source_commit: Any,
    toolchain_id: Any,
    policy_id: Any,
) -> dict[str, Any]:
    """Evaluate a canonical requested profile against measured evidence."""
    try:
        identity = _request_identity(
            resolved_request, source_commit, toolchain_id, policy_id
        )
    except MatrixError as error:
        return _invalid_request_result(
            board_id=(
                resolved_request.get("board_id")
                if isinstance(resolved_request, dict)
                else None
            ),
            source_commit=source_commit if isinstance(source_commit, str) else None,
            toolchain_id=toolchain_id if isinstance(toolchain_id, str) else None,
            policy_id=policy_id if isinstance(policy_id, str) else None,
            message=str(error),
        )

    profile = identity["profile"]
    base_context = {
        "board_id": identity["board_id"],
        "cpu": identity["cpu"],
        "source_commit": identity["source_commit"],
        "toolchain_id": identity["toolchain_id"],
        "policy_id": identity["policy_id"],
        "profile_key": identity["profile_key"],
        "context_key": identity["context_key"],
    }
    try:
        normalized_index = normalize_index(index)
    except MatrixError as error:
        message = f"Resource index is missing or malformed: {error}"
        result = _result(
            "requires_build",
            "resource_index_unavailable",
            board_id=identity["board_id"],
            profile=profile,
            evidence_context=base_context,
            message=message,
            diagnostics=[
                {
                    "code": "resource_index_unavailable",
                    "message": str(error),
                }
            ],
        )
        result["estimate"] = _unavailable_estimate(
            "resource_index_unavailable", message
        )
        return result

    exact = [
        record
        for record in normalized_index["observations"]
        if record["context_key"] == identity["context_key"]
    ]
    incomplete = [
        record
        for record in normalized_index["diagnostics"]
        if record["context_key"] == identity["context_key"]
    ]
    if exact:
        signatures = {_measurement_signature(record) for record in exact}
        statuses = {record["status"] for record in exact}
        record_ids = [record["observation_id"] for record in exact]
        evidence = _evidence_context(identity, record_ids)
        if len(signatures) != 1 or len(statuses) != 1:
            result = _result(
                "requires_build",
                "conflicting_observations",
                board_id=identity["board_id"],
                profile=profile,
                evidence_context=evidence,
                message=(
                    "Exact profile has conflicting resource measurements or "
                    "conflicting accepted/rejected statuses."
                ),
                diagnostics=_common_diagnostics(exact),
            )
            result["measurement"] = {
                "kind": "conflicting_exact_measurements",
                "sample_count": len(exact),
                "sample_ids": sorted(record_ids),
            }
            return result
        if exact[0]["status"] == "accepted":
            result = _result(
                "verified",
                "exact_measurement_verified",
                board_id=identity["board_id"],
                profile=profile,
                evidence_context=evidence,
                message=(
                    "An exact complete build observation satisfies the policy "
                    "for this full context."
                ),
            )
            result["measurement"] = _measurement(exact)
            return result
        result = _result(
            "incompatible",
            "exact_measurement_resource_failure",
            board_id=identity["board_id"],
            profile=profile,
            evidence_context=evidence,
            message=(
                "An exact complete build observation reports a resource failure."
            ),
            diagnostics=_common_diagnostics(exact),
        )
        result["measurement"] = _measurement(exact)
        return result

    estimate = _estimate(normalized_index, identity)
    if incomplete:
        result = _result(
            "requires_build",
            "incomplete_measurement_diagnostic",
            board_id=identity["board_id"],
            profile=profile,
            evidence_context=_evidence_context(
                identity, [record["observation_id"] for record in incomplete]
            ),
            message=(
                "An exact-context report exists but lacks complete metrics; "
                "its failure is not treated as resource incompatibility."
            ),
            diagnostics=_common_diagnostics(incomplete),
        )
        result["estimate"] = estimate
        return result

    evidence = _evidence_context(
        identity,
        [
            record["observation_id"]
            for record in normalized_index["observations"]
            if record["board_id"] == identity["board_id"]
            and record["cpu"] == identity["cpu"]
            and record["source_commit"] == identity["source_commit"]
            and record["toolchain"]["id"] == identity["toolchain_id"]
            and record["policy"]["id"] == identity["policy_id"]
        ],
    )
    if not estimate["known"]:
        unknown = estimate.get("unknown", {})
        reason = (
            "estimate_policy_overrun_warning"
            if estimate.get("predicted", {}).get("predicted_overbudget")
            else "feature_estimate_coverage_incomplete"
        )
        result = _result(
            "requires_build",
            reason,
            board_id=identity["board_id"],
            profile=profile,
            evidence_context=evidence,
            message=(
                "No exact measurement exists and the modules-only delta model "
                "does not have complete, consistent evidence."
            ),
            diagnostics=[
                {
                    "code": str(unknown.get("reason", "unknown_estimate")),
                    "message": _canonical_json(unknown),
                }
            ],
        )
        result["estimate"] = estimate
        return result

    overbudget = estimate["predicted"]["predicted_overbudget"]
    result = _result(
        "requires_build",
        (
            "estimate_policy_overrun_warning"
            if overbudget
            else "estimate_complete_not_measured"
        ),
        board_id=identity["board_id"],
        profile=profile,
        evidence_context=evidence,
        message=(
            "The modules-only estimate predicts a policy overrun; this is a "
            "build warning, not observed incompatibility."
            if overbudget
            else "A complete estimate exists, but only a build can verify it."
        ),
    )
    result["estimate"] = estimate
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    merge = commands.add_parser(
        "merge", help="merge resource extraction reports into an index"
    )
    merge.add_argument("--observations", nargs="+", required=True, metavar="PATH")
    merge.add_argument("--index", help="existing schema-1 resource index")
    merge.add_argument("--output", required=True, metavar="PATH")

    evaluation = commands.add_parser(
        "evaluate", help="preflight a canonical board/profile request"
    )
    evaluation.add_argument("--root", required=True)
    evaluation.add_argument("--index", required=True)
    evaluation.add_argument("--board", required=True)
    evaluation.add_argument("--features", required=True)
    evaluation.add_argument("--source-commit", required=True)
    evaluation.add_argument("--toolchain-id", required=True)
    evaluation.add_argument("--policy-id", required=True)
    evaluation.add_argument("--ve-rpm-count", type=int, default=16)
    evaluation.add_argument("--airmass-blend-size", type=int, default=8)
    evaluation.add_argument("--cylinder-trim-count", type=int, default=None)
    evaluation.add_argument("--engine-presets", default=None)
    evaluation.add_argument("--output", metavar="PATH")
    return parser


def _read_observation_documents(paths: Iterable[str]) -> list[Any]:
    reports = []
    for path in paths:
        document = _read_json(path)
        if isinstance(document, dict) and "observations" in document:
            try:
                nested = normalize_index(document)
            except MatrixError as error:
                raise MatrixError(f"{path}: {error}") from error
            reports.extend(nested["observations"])
            reports.extend(nested["diagnostics"])
        else:
            reports.append(document)
    return reports


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "merge":
        try:
            reports = _read_observation_documents(args.observations)
            existing = load_index(args.index) if args.index else None
            index = merge_reports(reports, existing)
            _atomic_json(args.output, index)
        except (MatrixError, OSError) as error:
            print(f"matrix merge: error: {error}", file=sys.stderr)
            return 2
        return 0

    result = None
    try:
        try:
            index = load_index(args.index)
        except MatrixError as error:
            index = {
                "schema_version": SCHEMA_VERSION,
                "observations": [],
                "diagnostics": [],
                "_load_error": str(error),
            }
        try:
            resolved = catalog.resolve(
                args.root,
                args.board,
                args.features,
                ve_rpm_count=args.ve_rpm_count,
                airmass_blend_size=args.airmass_blend_size,
                cylinder_trim_count=args.cylinder_trim_count,
                engine_presets=args.engine_presets,
            )
        except (catalog.CatalogError, OSError) as error:
            result = _invalid_request_result(
                board_id=args.board,
                source_commit=args.source_commit,
                toolchain_id=args.toolchain_id,
                policy_id=args.policy_id,
                message=str(error),
            )
        else:
            if "_load_error" in index:
                message = (
                    "Resource index is missing or malformed: "
                    + index["_load_error"]
                )
                result = _result(
                    "requires_build",
                    "resource_index_unavailable",
                    board_id=resolved["board_id"],
                    profile=normalize_profile(resolved["profile"]),
                    evidence_context={
                        "board_id": resolved["board_id"],
                        "source_commit": args.source_commit,
                        "toolchain_id": args.toolchain_id,
                        "policy_id": args.policy_id,
                    },
                    message=message,
                    diagnostics=[
                        {
                            "code": "resource_index_unavailable",
                            "message": index["_load_error"],
                        }
                    ],
                )
                result["estimate"] = _unavailable_estimate(
                    "resource_index_unavailable", message
                )
            else:
                result = evaluate(
                    index,
                    resolved,
                    args.source_commit,
                    args.toolchain_id,
                    args.policy_id,
                )
        if args.output:
            _atomic_json(args.output, result)
        else:
            print(json.dumps(result, indent=2, sort_keys=True))
    except (MatrixError, OSError) as error:
        print(f"matrix evaluate: error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
