#!/usr/bin/env python3
"""Fail-closed firmware resource extraction for custom ECU builds."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable

SCHEMA_VERSION = 1
FLASH_ORIGIN = 0x08000000
CPU_LIMITS = {
    "ARCH_STM32F4": {
        "config_start": 0x080C0000,
        "persistent_capacity": 128 * 1024,
        "persistent_ram": "ram4",
        "heap_region": "ram0",
    },
    "ARCH_STM32F7": {
        "config_start": 0x080C0000,
        "persistent_capacity": 32 * 1024,
        "persistent_ram": "ram3",
        "heap_region": "ram3",
    },
    "ARCH_STM32H7": {
        "config_start": 0x08100000,
        "persistent_capacity": 128 * 1024,
        "persistent_ram": "ram5",
        "heap_region": "ram0",
    },
}
PRESET_MODES = {"default_all", "default_none", "explicit", "explicit_all", "explicit_none"}
RAM_SYMBOL = re.compile(r"^__ram([0-7])_(base|end|free)__$")
ALL_RAM_REGIONS = tuple(f"ram{index}" for index in range(8))
RELEVANT_NM_SYMBOL = re.compile(
    r"^(?:"
    r"__ram[0-7]_(?:base|end|free)__|"
    r"__flash0_(?:base|end|free)__|"
    r"__heap_(?:base|end)__|"
    r"persistentState"
    r")$"
)
NM_ROW = re.compile(
    r"^\s*([0-9a-fA-F]+)\s+(?:([0-9a-fA-F]+)\s+)?([A-Za-z?])\s+(\S+)\s*$"
)
OBJDUMP_ROW = re.compile(
    r"^\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+"
    r"([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+(\S+)\s*$"
)


class ExtractionError(RuntimeError):
    """An input did not provide enough trustworthy evidence to accept a build."""


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_nm(text: str) -> dict[str, dict[str, int | None]]:
    """Parse the linker symbols from ``nm -S --defined-only`` output."""
    symbols: dict[str, dict[str, int | None]] = {}
    recognized = 0
    for line in text.splitlines():
        match = NM_ROW.match(line)
        if not match:
            continue
        recognized += 1
        address, size, _kind, name = match.groups()
        if not RELEVANT_NM_SYMBOL.fullmatch(name):
            continue
        if name in symbols:
            raise ExtractionError(f"duplicate defined symbol in nm output: {name}")
        symbols[name] = {
            "address": int(address, 16),
            "size": int(size, 16) if size is not None else None,
        }
    if not recognized:
        raise ExtractionError("nm output contained no recognizable defined symbols")
    return symbols


def parse_objdump_sections(text: str) -> list[dict[str, Any]]:
    """Parse the size/VMA/LMA/section table emitted by objdump -h."""
    sections: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    flag_lines: list[str] = []

    def finish() -> None:
        if current is not None:
            current["flags"] = " ".join(flag_lines)

    for line in text.splitlines():
        match = OBJDUMP_ROW.match(line)
        if match:
            finish()
            name, size, vma, lma, file_off, alignment = match.groups()
            current = {
                "name": name,
                "size": int(size, 16),
                "vma": int(vma, 16),
                "lma": int(lma, 16),
                "file_offset": int(file_off, 16),
                "alignment": alignment,
            }
            sections.append(current)
            flag_lines = []
        elif current is not None and line.strip():
            flag_lines.append(line.strip())
    finish()
    if not sections:
        raise ExtractionError("objdump output contained no recognizable sections")
    return sections


def parse_profile(text: str) -> dict[str, Any]:
    """Read the canonical generated/module_profile.txt fields used by a build."""
    values: dict[str, tuple[int, str]] = {}
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith(("#", "!")):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise ExtractionError(f"malformed profile line {line_number}: {raw_line!r}")
        key, value = parts
        if key in {
            "modules",
            "cylinder_fuel_trim_count",
            "ve_rpm_count",
            "airmass_blend_size",
            "engine_preset_mode",
            "engine_presets",
            "engine_preset_default",
        }:
            if key in values:
                raise ExtractionError(f"duplicate profile field: {key}")
            values[key] = (line_number, value)

    required = {
        "modules",
        "cylinder_fuel_trim_count",
        "ve_rpm_count",
        "airmass_blend_size",
        "engine_preset_mode",
        "engine_presets",
        "engine_preset_default",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise ExtractionError(f"profile is missing required fields: {', '.join(missing)}")

    modules = values["modules"][1].split()
    if not modules or len(modules) != len(set(modules)):
        raise ExtractionError("profile modules must be a non-empty unique list")

    numeric: dict[str, int] = {}
    for key in ("cylinder_fuel_trim_count", "ve_rpm_count", "airmass_blend_size"):
        try:
            numeric[key] = int(values[key][1], 10)
        except ValueError as error:
            raise ExtractionError(f"profile field {key} is not an integer") from error
        if numeric[key] < 0:
            raise ExtractionError(f"profile field {key} is negative")

    preset_mode = values["engine_preset_mode"][1]
    if preset_mode not in PRESET_MODES:
        raise ExtractionError(f"unknown engine_preset_mode: {preset_mode}")
    presets = values["engine_presets"][1].split()
    if not presets or len(presets) != len(set(presets)):
        raise ExtractionError("profile engine_presets must be a non-empty unique list")

    return {
        "modules": sorted(modules),
        "ve_rpm_count": numeric["ve_rpm_count"],
        "airmass_blend_size": numeric["airmass_blend_size"],
        "cylinder_trim_count": numeric["cylinder_fuel_trim_count"],
        "engine_preset_mode": preset_mode,
        "engine_presets": presets,
        "engine_preset_default": values["engine_preset_default"][1],
    }


def normalize_policy(
    value: Any | None, allowed_regions: Iterable[str]
) -> dict[str, Any]:
    """Validate and canonicalize a schema-version-1 resource policy."""
    if value is None:
        value = {"schema_version": SCHEMA_VERSION}
    if not isinstance(value, dict):
        raise ExtractionError("policy must be a JSON object")
    unknown = sorted(set(value) - {"schema_version", "required_free_bytes", "minimum_heap_capacity_bytes"})
    if unknown:
        raise ExtractionError(f"policy has unknown fields: {', '.join(unknown)}")
    if value.get("schema_version") != SCHEMA_VERSION or isinstance(
        value.get("schema_version"), bool
    ):
        raise ExtractionError("policy schema_version must be integer 1")

    requirements = value.get("required_free_bytes", {})
    if not isinstance(requirements, dict):
        raise ExtractionError("policy required_free_bytes must be an object")
    allowed = set(allowed_regions)
    unknown_regions = sorted(set(requirements) - allowed)
    if unknown_regions:
        raise ExtractionError(
            f"policy has unknown or unavailable regions: {', '.join(unknown_regions)}"
        )

    effective: dict[str, int] = {}
    for region in sorted(allowed):
        amount = requirements.get(region, 0)
        if isinstance(amount, bool) or not isinstance(amount, int) or amount < 0:
            raise ExtractionError(
                f"policy required_free_bytes[{region!r}] must be a non-negative integer"
            )
        effective[region] = amount

    heap = value.get("minimum_heap_capacity_bytes", 0)
    if isinstance(heap, bool) or not isinstance(heap, int) or heap < 0:
        raise ExtractionError("policy minimum_heap_capacity_bytes must be a non-negative integer")

    normalized = {
        "schema_version": SCHEMA_VERSION,
        "required_free_bytes": effective,
        "minimum_heap_capacity_bytes": heap,
    }
    identity = hashlib.sha256(_canonical_json(normalized).encode("utf-8")).hexdigest()
    return {"id": identity, **normalized}


def _add_violation(report: dict[str, Any], code: str, message: str) -> None:
    item = {"code": code, "message": message}
    if item not in report["violations"]:
        report["violations"].append(item)


def _symbol(
    symbols: dict[str, dict[str, int | None]], name: str, report: dict[str, Any]
) -> dict[str, int | None] | None:
    value = symbols.get(name)
    if value is None:
        _add_violation(report, "missing_symbol", f"required symbol is absent: {name}")
    return value


def _interval_overlap(first: tuple[int, int], second: tuple[int, int]) -> bool:
    return first[0] < second[1] and second[0] < first[1]


def _section_has_flag(section: dict[str, Any], flag: str) -> bool:
    flags = re.split(r"[\s,]+", section.get("flags", "").upper())
    return flag in flags


def _is_heap_reservation(section: dict[str, Any]) -> bool:
    name = section["name"]
    return name == ".heap" or name.startswith(".heap.")


def _union_size(ranges: Iterable[tuple[int, int]]) -> int:
    total = 0
    cursor: int | None = None
    end: int | None = None
    for start, stop in sorted(ranges):
        if stop <= start:
            continue
        if cursor is None:
            cursor, end = start, stop
        elif start <= end:
            end = max(end, stop)
        else:
            total += end - cursor
            cursor, end = start, stop
    if cursor is not None:
        total += end - cursor
    return total


def _overlap_groups(regions: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    names = sorted(name for name, region in regions.items() if name.startswith("ram"))
    neighbors: dict[str, set[str]] = {}
    for index, name in enumerate(names):
        for other in names[index + 1 :]:
            if _interval_overlap(
                (regions[name]["origin"], regions[name]["limit"]),
                (regions[other]["origin"], regions[other]["limit"]),
            ):
                neighbors.setdefault(name, set()).add(other)
                neighbors.setdefault(other, set()).add(name)

    groups: list[dict[str, Any]] = []
    visited: set[str] = set()
    for name in names:
        region = regions[name]
        region["overlaps"] = sorted(neighbors.get(name, set()))
        if name in visited or name not in neighbors:
            continue
        pending = [name]
        group: set[str] = set()
        while pending:
            member = pending.pop()
            if member in group:
                continue
            group.add(member)
            pending.extend(neighbors.get(member, set()) - group)
        visited.update(group)
        groups.append(
            {
                "regions": sorted(group),
                "accounting": "separate_alias_views",
                "aggregate_bytes": None,
            }
        )
    return groups


def assemble_report(
    *,
    board_id: str,
    cpu: str,
    source_commit: str,
    profile: dict[str, Any],
    symbols: dict[str, dict[str, int | None]],
    sections: list[dict[str, Any]],
    toolchain: dict[str, str],
    elf_sha256: str,
    policy_data: Any | None = None,
) -> dict[str, Any]:
    """Build a resource report from already-collected tool evidence."""
    if cpu not in CPU_LIMITS:
        raise ExtractionError(f"unsupported CPU: {cpu}")
    limits = CPU_LIMITS[cpu]
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "board_id": board_id,
        "cpu": cpu,
        "source_commit": source_commit,
        "toolchain": toolchain,
        "profile": profile,
        "elf_sha256": elf_sha256,
        "regions": {},
        "heap": {},
        "persistent": {},
        "policy": {},
        "status": "rejected",
        "violations": [],
        "hardware_guarantee": False,
        "note": "Build-time technical-fit result only; not a runtime or hardware guarantee.",
    }

    # Every RAM alias must provide complete linker evidence. Empty aliases are
    # valid views, but an absent alias or a negative-sized one is not.
    ram_parts: dict[str, dict[str, dict[str, int | None]]] = {}
    for name in symbols:
        match = RAM_SYMBOL.match(name)
        if match:
            ram_parts.setdefault(f"ram{match.group(1)}", {})[match.group(2)] = symbols[name]
    alloc_sections = [
        section
        for section in sections
        if section["size"] > 0
        and _section_has_flag(section, "ALLOC")
        and not _is_heap_reservation(section)
    ]
    for region_name in ALL_RAM_REGIONS:
        parts = ram_parts.get(region_name, {})
        missing = sorted({"base", "end", "free"} - parts.keys())
        if missing:
            _add_violation(
                report,
                "missing_ram_symbols",
                f"{region_name} is missing linker symbols: {', '.join(missing)}",
            )
            continue
        base = int(parts["base"]["address"])
        end = int(parts["end"]["address"])
        free_boundary = int(parts["free"]["address"])
        capacity = end - base
        if capacity < 0:
            _add_violation(
                report,
                "malformed_ram_symbols",
                f"{region_name} end precedes its base",
            )
            report["regions"][region_name] = {
                "origin": base,
                "limit": end,
                "capacity_bytes": capacity,
                "linker_free_address": free_boundary,
                "linker_free_bytes": end - free_boundary,
                "used_bytes": 0,
                "free_bytes": 0,
                "required_free_bytes": 0,
                "status": "rejected",
            }
            continue
        free_boundary_ok = base <= free_boundary <= end
        if not free_boundary_ok:
            _add_violation(
                report,
                "malformed_ram_symbols",
                f"{region_name} free boundary is outside its base/end range",
            )
        alloc_highwater = max(
            (
                min(end, section["vma"] + section["size"])
                for section in alloc_sections
                if max(base, section["vma"]) < min(end, section["vma"] + section["size"])
            ),
            default=base,
        )
        highwater = max(base, min(end, free_boundary), alloc_highwater)
        report["regions"][region_name] = {
            "origin": base,
            "limit": end,
            "capacity_bytes": capacity,
            "linker_free_address": free_boundary,
            "linker_free_bytes": end - free_boundary,
            "used_bytes": highwater - base,
            "free_bytes": end - highwater,
            "required_free_bytes": 0,
            "status": "accepted" if free_boundary_ok else "rejected",
        }

    # Default heap region symbols give a reserved capacity, not measured runtime availability.
    heap_base = _symbol(symbols, "__heap_base__", report)
    heap_end = _symbol(symbols, "__heap_end__", report)
    if heap_base is not None and heap_end is not None:
        heap_origin = int(heap_base["address"])
        heap_limit = int(heap_end["address"])
        heap_capacity = heap_limit - heap_origin
        heap_region_name = limits["heap_region"]
        heap_region = report["regions"].get(heap_region_name)
        if heap_capacity < 0:
            _add_violation(report, "malformed_heap", "heap_end precedes heap_base")
        heap_location_ok = heap_region is not None and (
            heap_region["origin"] <= heap_origin <= heap_region["limit"]
            and heap_region["origin"] <= heap_limit <= heap_region["limit"]
        )
        if not heap_location_ok:
            _add_violation(
                report,
                "malformed_heap",
                f"heap symbols are outside the expected {heap_region_name} region",
            )
        report["heap"] = {
            "origin": heap_origin,
            "limit": heap_limit,
            "capacity_bytes": heap_capacity,
            "note": "reserved capacity; not runtime free",
            "required_capacity_bytes": 0,
            "status": "accepted" if heap_capacity >= 0 and heap_location_ok else "rejected",
        }

    # Flash symbols are also boundary addresses.
    flash_parts = {
        suffix: symbols.get(f"__flash0_{suffix}__")
        for suffix in ("base", "end", "free")
    }
    for suffix, value in flash_parts.items():
        if value is None:
            _add_violation(
                report,
                "missing_symbol",
                f"required symbol is absent: __flash0_{suffix}__",
            )
    if all(value is not None for value in flash_parts.values()):
        flash_base = int(flash_parts["base"]["address"])
        linker_limit = int(flash_parts["end"]["address"])
        free_boundary = int(flash_parts["free"]["address"])
        config_start = int(limits["config_start"])
        safe_end = min(linker_limit, config_start)

        if flash_base < FLASH_ORIGIN:
            _add_violation(report, "malformed_flash", "flash0 starts below STM32 flash")
        if safe_end <= flash_base:
            _add_violation(
                report, "malformed_flash", "flash safe end does not leave application capacity"
            )
        if not flash_base <= free_boundary <= linker_limit:
            _add_violation(
                report,
                "malformed_flash",
                "flash free boundary is outside the linker flash range",
            )

        load_sections = [
            section for section in sections if _section_has_flag(section, "LOAD")
        ]
        if not load_sections:
            _add_violation(report, "missing_flash_load", "ELF has no LOAD sections")

        boot_origin = FLASH_ORIGIN
        boot_limit = max(boot_origin, min(flash_base, config_start))
        boot_sections = [
            section
            for section in load_sections
            if section["name"] == ".bl"
            or section["name"].startswith(".bl.")
            or section["name"] == ".bootloader"
        ]
        boot_ranges = []
        for section in boot_sections:
            section_range = (section["lma"], section["lma"] + section["size"])
            if section["size"] and (
                section_range[0] < boot_origin or section_range[1] > boot_limit
            ):
                _add_violation(
                    report,
                    "bootloader_section_overlap",
                    f"{section['name']} extends outside reserved bootloader flash",
                )
            if section["size"]:
                boot_ranges.append(section_range)

        app_sections: list[dict[str, Any]] = []
        for section in load_sections:
            if section["size"] == 0 or section in boot_sections:
                continue
            start = section["lma"]
            stop = start + section["size"]
            if start < flash_base:
                _add_violation(
                    report,
                    "flash_load_in_reserved_range",
                    f"{section['name']} LOAD starts below application flash",
                )
                continue
            app_sections.append(section)
            if start < safe_end < stop or (start >= safe_end):
                _add_violation(
                    report,
                    "flash_load_overlaps_config",
                    f"{section['name']} LOAD [{start:#x},{stop:#x}) exceeds safe end {safe_end:#x}",
                )

        load_ranges = [(section["lma"], section["lma"] + section["size"]) for section in app_sections]
        load_end = max((stop for _start, stop in load_ranges), default=flash_base)
        if free_boundary < load_end:
            _add_violation(
                report,
                "flash_symbol_free_before_load",
                "flash free boundary precedes an application LOAD section",
            )
        used = max(0, free_boundary - flash_base, load_end - flash_base)
        capacity = max(0, safe_end - flash_base)
        free = safe_end - (flash_base + used)
        status = "accepted"
        if (
            safe_end <= flash_base
            or not flash_base <= free_boundary <= linker_limit
            or free < 0
        ):
            status = "rejected"
        report["regions"]["flash"] = {
            "origin": flash_base,
            "limit": safe_end,
            "linker_limit": linker_limit,
            "config_limit": config_start,
            "capacity_bytes": capacity,
            "used_bytes": used,
            "free_bytes": free,
            "required_free_bytes": 0,
            "status": status,
        }

        ram_intervals = [
            (region["origin"], region["limit"])
            for name, region in report["regions"].items()
            if name.startswith("ram")
        ]
        initialized_ram = []
        for section in app_sections:
            if any(
                region_origin <= section["vma"] < region_limit
                for region_origin, region_limit in ram_intervals
            ):
                initialized_ram.append(
                    {
                        "name": section["name"],
                        "vma": section["vma"],
                        "lma": section["lma"],
                        "size": section["size"],
                    }
                )
        report["flash_load_sections"] = [
            {
                "name": section["name"],
                "vma": section["vma"],
                "lma": section["lma"],
                "size": section["size"],
            }
            for section in app_sections
        ]
        report["initialized_ram_load_sections"] = initialized_ram
        report["initialized_ram_load_bytes"] = _union_size(
            (section["lma"], section["lma"] + section["size"])
            for section in initialized_ram
        )
        report["bootloader_reserved"] = {
            "origin": boot_origin,
            "limit": boot_limit,
            "capacity_bytes": boot_limit - boot_origin,
            "elf_load_bytes": _union_size(boot_ranges),
            "note": "low-flash bootloader reservation; excluded from application range",
        }
        report["config_reserved"] = {
            "origin": config_start,
            "limit": config_start + int(limits["persistent_capacity"]),
            "capacity_bytes": int(limits["persistent_capacity"]),
            "note": "first persistent configuration slot boundary",
        }

    persistent = symbols.get("persistentState")
    if persistent is None:
        _add_violation(report, "missing_symbol", "required symbol is absent: persistentState")
    elif persistent.get("size") is None or int(persistent["size"]) <= 0:
        _add_violation(
            report,
            "missing_persistent_size",
            "persistentState must have a positive ELF symbol size",
        )
    else:
        used = int(persistent["size"])
        capacity = int(limits["persistent_capacity"])
        free = capacity - used
        persistent_ram = limits["persistent_ram"]
        persistent_region = report["regions"].get(persistent_ram)
        persistent_address = int(persistent["address"])
        persistent_location_ok = persistent_region is not None and (
            persistent_region["origin"]
            <= persistent_address
            < persistent_region["limit"]
        )
        if not persistent_location_ok:
            _add_violation(
                report,
                "malformed_persistent_location",
                f"persistentState is not inside the expected {persistent_ram} region",
            )
        report["persistent"] = {
            "ram_region": persistent_ram,
            "used_bytes": used,
            "capacity_bytes": capacity,
            "free_bytes": free,
            "required_free_bytes": 0,
            "status": "accepted" if free >= 0 and persistent_location_ok else "rejected",
        }

    allowed_regions = set(report["regions"]) | ({"persistent"} if report["persistent"] else set())
    try:
        policy = normalize_policy(policy_data, allowed_regions)
    except ExtractionError as error:
        policy = normalize_policy(None, allowed_regions)
        policy["error"] = str(error)
        _add_violation(report, "invalid_policy", str(error))
    report["policy"] = policy

    for name, region in report["regions"].items():
        required = policy["required_free_bytes"].get(name, 0)
        region["required_free_bytes"] = required
        if region["free_bytes"] < required:
            region["status"] = "rejected"
            _add_violation(
                report,
                "policy_free_below_minimum",
                f"{name} has {region['free_bytes']} free bytes, policy requires {required}",
            )
    if report["persistent"]:
        required = policy["required_free_bytes"].get("persistent", 0)
        report["persistent"]["required_free_bytes"] = required
        if report["persistent"]["free_bytes"] < required:
            report["persistent"]["status"] = "rejected"
            _add_violation(
                report,
                "policy_free_below_minimum",
                f"persistent has {report['persistent']['free_bytes']} free bytes, "
                f"policy requires {required}",
            )
    if report["heap"]:
        required = policy["minimum_heap_capacity_bytes"]
        report["heap"]["required_capacity_bytes"] = required
        if report["heap"]["capacity_bytes"] < required:
            report["heap"]["status"] = "rejected"
            _add_violation(
                report,
                "policy_heap_below_minimum",
                f"heap capacity is {report['heap']['capacity_bytes']} bytes, "
                f"policy requires {required}",
            )

    report["ram_alias_groups"] = _overlap_groups(report["regions"])
    statuses = [
        region["status"]
        for region in report["regions"].values()
        if isinstance(region, dict) and "status" in region
    ]
    if report["heap"]:
        statuses.append(report["heap"]["status"])
    if report["persistent"]:
        statuses.append(report["persistent"]["status"])
    if not report["violations"] and statuses and all(value == "accepted" for value in statuses):
        report["status"] = "accepted"
    return report


def _run_tool(command: list[str]) -> str:
    try:
        result = subprocess.run(
            command, check=False, capture_output=True, text=True
        )
    except OSError as error:
        raise ExtractionError(f"could not execute {command[0]}: {error}") from error
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise ExtractionError(
            f"tool failed ({result.returncode}): {' '.join(command)}: {detail}"
        )
    return result.stdout


def toolchain_identity(compiler: Path) -> dict[str, str]:
    """Return a stable identity for the exact compiler binary and version."""
    version_output = _run_tool([str(compiler), "--version"])
    version = next(
        (line.strip() for line in version_output.splitlines() if line.strip()),
        "",
    )
    if not version:
        raise ExtractionError("compiler --version returned no identity")
    compiler_sha256 = sha256_file(compiler)
    identity_input = f"{compiler_sha256}\0{version}".encode("utf-8")
    return {
        "id": hashlib.sha256(identity_input).hexdigest(),
        "version": version,
        "sha256": compiler_sha256,
    }


def discover_root(script_path: Path) -> Path:
    for parent in script_path.resolve().parents:
        if (parent / "firmware" / "hw_layer" / "ports" / "stm32").is_dir():
            return parent
    raise ExtractionError("could not autodiscover repository root; pass --root")


def default_tool_paths(root: Path) -> dict[str, Path]:
    """Select tools only below firmware/ext/build-tools, never from PATH."""
    build_tools = root / "firmware" / "ext" / "build-tools"
    candidates: list[Path] = []
    machine = platform.machine().lower()
    if sys.platform == "darwin" and machine in {"x86_64", "amd64"}:
        candidates.append(
            build_tools / "arm-gnu-toolchain-11.3.rel1-darwin-x86_64-arm-none-eabi" / "bin"
        )
    elif sys.platform.startswith("linux") and machine in {"x86_64", "amd64"}:
        candidates.append(
            build_tools / "arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-eabi" / "bin"
        )
    if build_tools.is_dir():
        candidates.extend(sorted(path for path in build_tools.glob("*/bin") if path.is_dir()))
    candidates.append(build_tools / "bin")
    for directory in candidates:
        tools = {
            "nm": directory / "arm-none-eabi-nm",
            "objdump": directory / "arm-none-eabi-objdump",
            "compiler": directory / "arm-none-eabi-g++",
        }
        if all(path.is_file() for path in tools.values()):
            return tools
    fallback = candidates[0] if candidates else build_tools / "bin"
    return {
        "nm": fallback / "arm-none-eabi-nm",
        "objdump": fallback / "arm-none-eabi-objdump",
        "compiler": fallback / "arm-none-eabi-g++",
    }


def resolve_tools(
    root: Path,
    nm: str | None = None,
    objdump: str | None = None,
    compiler: str | None = None,
) -> dict[str, Path]:
    defaults = default_tool_paths(root)
    selected = {
        "nm": Path(nm).expanduser() if nm else defaults["nm"],
        "objdump": Path(objdump).expanduser() if objdump else defaults["objdump"],
        "compiler": Path(compiler).expanduser() if compiler else defaults["compiler"],
    }
    missing = [str(path) for path in selected.values() if not path.is_file()]
    if missing:
        raise ExtractionError(f"tool not found: {', '.join(missing)}")
    return selected


def extract_report(
    *,
    elf: Path,
    profile_path: Path,
    board_id: str,
    cpu: str,
    source_commit: str,
    tools: dict[str, Path],
    policy_path: Path | None = None,
) -> dict[str, Any]:
    if cpu not in CPU_LIMITS:
        raise ExtractionError(f"unsupported CPU: {cpu}")
    if not elf.is_file():
        raise ExtractionError(f"ELF not found: {elf}")
    if not profile_path.is_file():
        raise ExtractionError(f"profile not found: {profile_path}")
    if not re.fullmatch(r"[0-9a-fA-F]{7,64}", source_commit):
        raise ExtractionError("source_commit must be a 7-64 character hexadecimal SHA")

    profile = parse_profile(profile_path.read_text(encoding="utf-8"))
    symbols = parse_nm(
        _run_tool([str(tools["nm"]), "-S", "--defined-only", str(elf)])
    )
    sections = parse_objdump_sections(
        _run_tool([str(tools["objdump"]), "-h", str(elf)])
    )
    policy_data = None
    if policy_path is not None:
        if not policy_path.is_file():
            raise ExtractionError(f"policy not found: {policy_path}")
        try:
            policy_data = json.loads(policy_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise ExtractionError(f"policy is not valid JSON: {error}") from error

    return assemble_report(
        board_id=board_id,
        cpu=cpu,
        source_commit=source_commit,
        profile=profile,
        symbols=symbols,
        sections=sections,
        toolchain=toolchain_identity(tools["compiler"]),
        elf_sha256=sha256_file(elf),
        policy_data=policy_data,
    )


def _rejection_report(args: argparse.Namespace, message: str) -> dict[str, Any]:
    report: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "board_id": args.board,
        "cpu": args.cpu,
        "source_commit": args.source_commit,
        "toolchain": {},
        "profile": None,
        "elf_sha256": None,
        "regions": {},
        "heap": {},
        "persistent": {},
        "policy": {},
        "status": "rejected",
        "violations": [{"code": "extraction_failed", "message": message}],
        "hardware_guarantee": False,
        "note": "Build-time technical-fit result only; not a runtime or hardware guarantee.",
    }
    try:
        report["profile"] = parse_profile(Path(args.profile).read_text(encoding="utf-8"))
    except (OSError, ExtractionError, UnicodeError):
        pass
    try:
        report["elf_sha256"] = sha256_file(Path(args.elf))
    except OSError:
        pass
    return report


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    extract = subparsers.add_parser("extract", help="extract and gate one firmware ELF")
    extract.add_argument("--elf", required=True)
    extract.add_argument("--profile", required=True)
    extract.add_argument("--board", required=True)
    extract.add_argument("--cpu", required=True, choices=sorted(CPU_LIMITS))
    extract.add_argument("--source-commit", required=True)
    extract.add_argument("--output", required=True)
    extract.add_argument("--nm")
    extract.add_argument("--objdump")
    extract.add_argument("--compiler")
    extract.add_argument("--policy")
    extract.add_argument("--root")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = (
        Path(args.root).expanduser().resolve()
        if args.root
        else discover_root(Path(__file__))
    )
    try:
        tools = resolve_tools(root, args.nm, args.objdump, args.compiler)
        report = extract_report(
            elf=Path(args.elf).expanduser(),
            profile_path=Path(args.profile).expanduser(),
            board_id=args.board,
            cpu=args.cpu,
            source_commit=args.source_commit,
            tools=tools,
            policy_path=Path(args.policy).expanduser() if args.policy else None,
        )
    except (ExtractionError, OSError, UnicodeError) as error:
        report = _rejection_report(args, str(error))
    _write_report(Path(args.output).expanduser(), report)
    return 0 if report["status"] == "accepted" else 1


if __name__ == "__main__":
    raise SystemExit(main())
