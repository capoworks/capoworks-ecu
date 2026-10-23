#!/usr/bin/env python3
"""Validate a firmware build and publish one self-contained package."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

try:
    from . import catalog, matrix
except ImportError:  # Allow direct execution from this directory.
    import catalog  # type: ignore[no-redef]
    import matrix  # type: ignore[no-redef]


MANIFEST_SCHEMA_VERSION = 2
SOURCE_RE = re.compile(r"^[0-9a-fA-F]{7,64}$")
ZIP_EOCD = b"PK\x05\x06"
MAX_EMBEDDED_INI_BYTES = 8 * 1024 * 1024
PERSISTENT_WRAPPER_BYTES = 20


class PackageError(RuntimeError):
    """A build does not satisfy the package publication contract."""


def _load_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PackageError(f"cannot read {label} {path}: {error}") from error


def _write_json(path: Path, document: Any) -> None:
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_head(root: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as error:
        raise PackageError(f"could not execute git: {error}") from error
    commit = result.stdout.strip()
    if result.returncode or not SOURCE_RE.fullmatch(commit):
        detail = (result.stderr or result.stdout).strip()
        raise PackageError(f"could not read repository HEAD: {detail}")
    return commit.lower()


def _git_ref(root: Path) -> str | None:
    """Return the checked-out symbolic ref, or ``None`` for detached HEAD."""
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "symbolic-ref", "--quiet", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as error:
        raise PackageError(f"could not execute git: {error}") from error
    if result.returncode == 1:
        return None
    ref = result.stdout.strip()
    if (
        result.returncode
        or not ref
        or any(character in ref for character in ("\x00", "\n", "\r"))
    ):
        detail = (result.stderr or result.stdout).strip()
        raise PackageError(f"could not read the checked-out Git ref: {detail}")
    return ref


def _safe_name(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or Path(value).name != value
        or any(character in value for character in ("\x00", "\n", "\r"))
    ):
        raise PackageError(f"{label} must be a safe file name")
    return value


def _string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list):
        raise PackageError(f"{label} must be a list")
    result = []
    for index, item in enumerate(value):
        if (
            not isinstance(item, str)
            or not item
            or any(character in item for character in ("\x00", "\n", "\r"))
        ):
            raise PackageError(f"{label}[{index}] must be a safe non-empty string")
        result.append(item)
    return result


def _optional_run_value(value: str | None, label: str) -> str | None:
    if value is None:
        return None
    if not value or any(character in value for character in ("\x00", "\n", "\r")):
        raise PackageError(f"{label} must be a non-empty single-line value")
    return value


def _nonempty_file(path: Path, label: str) -> None:
    try:
        size = path.stat().st_size
    except OSError as error:
        raise PackageError(f"{label} not found: {path}") from error
    if not path.is_file() or size <= 0:
        raise PackageError(f"{label} is empty or not a regular file: {path}")


def _catalog_board(document: Any, board_id: str) -> dict[str, Any]:
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != catalog.SCHEMA_VERSION
        or not isinstance(document.get("boards"), list)
    ):
        raise PackageError("canonical catalog has an unsupported document shape")
    matches = [
        board
        for board in document["boards"]
        if isinstance(board, dict) and board.get("id") == board_id
    ]
    if len(matches) != 1:
        raise PackageError(f"unknown or duplicate canonical board id {board_id!r}")
    return matches[0]


def _normalized_board_path(root: Path, value: Any, label: str) -> Path:
    if (
        not isinstance(value, str)
        or not value
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise PackageError(f"{label} must be a safe non-empty path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise PackageError(f"{label} must stay inside the repository")
    try:
        return (root / path).resolve()
    except (OSError, ValueError) as error:
        raise PackageError(f"cannot normalize {label}: {error}") from error


def _validate_generated_profile(
    *,
    root: Path,
    board: dict[str, Any],
    resolved_board_dir: str,
    resolved_cpu: str,
    resolved_profile_key: str,
) -> None:
    board_dir = board.get("board_dir")
    board_cpu = board.get("cpu")
    if not isinstance(board_dir, str) or not isinstance(board_cpu, str):
        raise PackageError("canonical catalog board has no board_dir or cpu")
    expected_board = _normalized_board_path(root, board_dir, "catalog board_dir")
    resolved_board = _normalized_board_path(
        root, resolved_board_dir, "resolved board_dir"
    )
    if resolved_board != expected_board:
        raise PackageError("resolved board_dir does not match canonical catalog board")
    if resolved_cpu != board_cpu:
        raise PackageError(
            f"resolved cpu {resolved_cpu!r} does not match canonical catalog CPU {board_cpu!r}"
        )

    profile_path = root / "firmware" / "generated" / "module_profile.txt"
    try:
        generated_profile, generated_board, generated_cpu = catalog._parse_profile(
            profile_path
        )
    except (catalog.CatalogError, OSError, UnicodeError, ValueError) as error:
        raise PackageError(f"cannot validate generated module profile: {error}") from error

    try:
        generated_board_path = Path(generated_board).expanduser()
        if not generated_board_path.is_absolute():
            generated_board_path = root / generated_board_path
        generated_board_path = generated_board_path.resolve()
    except (OSError, ValueError) as error:
        raise PackageError(f"cannot normalize generated profile board: {error}") from error
    if generated_board_path != expected_board:
        raise PackageError(
            f"generated module profile board {generated_board_path} "
            f"does not match canonical catalog board {expected_board}"
        )
    if generated_cpu != resolved_cpu:
        raise PackageError(
            f"generated module profile CPU {generated_cpu!r} "
            f"does not match resolved CPU {resolved_cpu!r}"
        )
    try:
        generated_profile_key = matrix.profile_key(generated_profile)
    except matrix.MatrixError as error:
        raise PackageError(f"invalid generated module profile: {error}") from error
    if generated_profile_key != resolved_profile_key:
        raise PackageError("generated module profile does not match resolved profile")


def _ini_value(text: str, key: str) -> str:
    values = []
    in_constants = False
    assignment = re.compile(rf"^{re.escape(key)}\s*=\s*(.*?)\s*$")
    section = re.compile(r"^\[(.*?)\]\s*(?:[;#].*)?$")
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith(("#", ";")):
            continue
        section_match = section.fullmatch(stripped)
        if section_match:
            in_constants = section_match.group(1).strip() == "Constants"
            continue
        if not in_constants:
            continue
        match = assignment.fullmatch(stripped)
        if match:
            values.append(match.group(1))
    if len(values) != 1:
        raise PackageError(f"generated INI must contain exactly one {key} field")
    return values[0]


def _ini_array_geometry(text: str, key: str) -> tuple[int, int]:
    value = _ini_value(text, key)
    fields = [field.strip() for field in value.split(",")]
    if len(fields) < 4 or fields[0].lower() != "array":
        raise PackageError(f"generated INI {key} must be an array field")
    match = re.fullmatch(r"\[(\d+)x(\d+)\]", fields[3])
    if not match:
        raise PackageError(f"generated INI {key} has malformed geometry")
    return int(match.group(1)), int(match.group(2))


def _validate_ini_layout(
    *,
    ini_bytes: bytes,
    persistent_size_bytes: int,
    ve_rpm_count: int,
    airmass_blend_size: int,
) -> None:
    try:
        text = ini_bytes.decode("utf-8")
    except UnicodeDecodeError as error:
        raise PackageError(f"generated INI is not UTF-8: {error}") from error

    raw_page_size = _ini_value(text, "pageSize")
    if not re.fullmatch(r"[0-9]+", raw_page_size):
        raise PackageError("generated INI pageSize must be a non-negative integer")
    expected_page_size = persistent_size_bytes - PERSISTENT_WRAPPER_BYTES
    if expected_page_size <= 0:
        raise PackageError("persistentState is too small for a pageSize")
    if int(raw_page_size) != expected_page_size:
        raise PackageError(
            f"generated INI pageSize {raw_page_size} does not match "
            f"persistent size {persistent_size_bytes} minus "
            f"{PERSISTENT_WRAPPER_BYTES}"
        )

    if _ini_array_geometry(text, "veTable") != (ve_rpm_count, 16):
        raise PackageError("generated INI VE table geometry does not match resolved profile")
    if _ini_array_geometry(text, "airmassBlendTable") != (
        airmass_blend_size,
        airmass_blend_size,
    ):
        raise PackageError(
            "generated INI airmass blend geometry does not match resolved profile"
        )


def _embedded_ini(binary_path: Path, ini_file: str) -> bytes | None:
    try:
        binary = binary_path.read_bytes()
    except OSError as error:
        raise PackageError(f"cannot read firmware binary: {error}") from error

    positions = []
    start = 0
    while True:
        position = binary.find(ZIP_EOCD, start)
        if position < 0:
            break
        positions.append(position)
        start = position + 1
    if not positions:
        return None

    archive_ends = []
    for position in positions:
        if position + 22 > len(binary):
            continue
        comment_length = int.from_bytes(binary[position + 20 : position + 22], "little")
        archive_end = position + 22 + comment_length
        if archive_end <= len(binary):
            archive_ends.append(archive_end)
    if not archive_ends:
        raise PackageError("malformed embedded ZIP archive")

    archive_found = False
    for archive_end in reversed(archive_ends):
        try:
            archive = zipfile.ZipFile(io.BytesIO(binary[:archive_end]))
        except (zipfile.BadZipFile, zipfile.LargeZipFile, EOFError, OSError, ValueError):
            continue
        with archive:
            archive_found = True
            matches = [
                entry for entry in archive.infolist() if entry.filename == ini_file
            ]
            if not matches:
                continue
            if len(matches) != 1 or matches[0].is_dir():
                raise PackageError(
                    f"embedded ZIP contains an ambiguous {ini_file} entry"
                )
            entry = matches[0]
            if entry.file_size > MAX_EMBEDDED_INI_BYTES:
                raise PackageError(
                    f"embedded INI exceeds {MAX_EMBEDDED_INI_BYTES} bytes"
                )
            try:
                with archive.open(entry) as handle:
                    payload = handle.read(MAX_EMBEDDED_INI_BYTES + 1)
            except (
                zipfile.BadZipFile,
                RuntimeError,
                NotImplementedError,
                OSError,
                EOFError,
                ValueError,
            ) as error:
                raise PackageError(f"cannot read embedded INI: {error}") from error
            if len(payload) > MAX_EMBEDDED_INI_BYTES:
                raise PackageError(
                    f"embedded INI exceeds {MAX_EMBEDDED_INI_BYTES} bytes"
                )
            return payload

    if not archive_found:
        raise PackageError("malformed embedded ZIP archive")
    raise PackageError(f"embedded ZIP is missing expected INI entry {ini_file}")


def _artifact_record(path: Path) -> dict[str, Any]:
    return {
        "file": path.name,
        "sha256": _sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _resource_summary(record: dict[str, Any]) -> dict[str, Any]:
    return {
        "status": record["status"],
        "complete": record["complete"],
        "observation_id": record["observation_id"],
        "profile_key": record["profile_key"],
        "context_key": record["context_key"],
        "violations": record["violations"],
    }


def _resource_context(record: dict[str, Any]) -> dict[str, Any]:
    return {
        "board_id": record["board_id"],
        "cpu": record["cpu"],
        "source_commit": record["source_commit"],
        "toolchain": record["toolchain"],
        "policy": record["policy"],
        "elf_sha256": record["elf_sha256"],
        "profile_key": record["profile_key"],
        "context_key": record["context_key"],
    }


def _resource_measurements(record: dict[str, Any]) -> dict[str, Any]:
    return {
        "profile": record["profile"],
        "regions": record["regions"],
        "heap": record["heap"],
        "persistent": record["persistent"],
    }


def package(
    *,
    root: str | Path,
    resolved_path: str | Path,
    resources_path: str | Path,
    output: str | Path,
    request_id: str,
    run_id: str | None = None,
    run_attempt: str | None = None,
) -> dict[str, Any]:
    """Validate inputs and atomically publish a schema-2 firmware package."""
    root_path = Path(root).expanduser().resolve()
    resolved_file = Path(resolved_path).expanduser().resolve()
    resources_file = Path(resources_path).expanduser().resolve()
    output_path = Path(output).expanduser().resolve()
    if output_path.exists() or output_path.is_symlink():
        raise PackageError(f"output already exists; refusing to replace it: {output_path}")
    try:
        catalog.validate_request_id(request_id)
    except catalog.CatalogError as error:
        raise PackageError(str(error)) from error
    run_id = _optional_run_value(run_id, "run id")
    run_attempt = _optional_run_value(run_attempt, "run attempt")

    resolved = _load_json(resolved_file, "resolved profile")
    raw_resource_report = _load_json(resources_file, "resource report")
    if not isinstance(resolved, dict):
        raise PackageError("resolved profile must be an object")
    if resolved.get("schema_version") != catalog.SCHEMA_VERSION:
        raise PackageError("resolved profile has an unsupported schema_version")

    board_id = resolved.get("board_id")
    cpu = resolved.get("cpu")
    if not isinstance(board_id, str) or not board_id:
        raise PackageError("resolved profile has no board_id")
    if not isinstance(cpu, str) or not cpu:
        raise PackageError("resolved profile has no cpu")
    resolved_board_dir = resolved.get("board_dir")
    if not isinstance(resolved_board_dir, str) or not resolved_board_dir:
        raise PackageError("resolved profile has no board_dir")
    ini_file = _safe_name(resolved.get("ini_file"), "resolved ini_file")
    base_modules = _string_list(resolved.get("base_modules"), "resolved base_modules")
    optional_modules = _string_list(
        resolved.get("optional_modules"), "resolved optional_modules"
    )
    required_modules = _string_list(
        resolved.get("required_modules", []), "resolved required_modules"
    )
    try:
        resolved_profile = matrix.normalize_profile(resolved.get("profile"))
        resolved_profile_key = matrix.profile_key(resolved_profile)
        record = matrix.normalize_report(raw_resource_report)
    except matrix.MatrixError as error:
        raise PackageError(f"invalid resolved profile or resource report: {error}") from error

    if record["status"] != "accepted":
        violations = ", ".join(
            f"{item['code']}: {item['message']}" for item in record["violations"]
        )
        raise PackageError(
            "resource gate rejected the build"
            + (f": {violations}" if violations else "")
        )
    if not record["complete"]:
        raise PackageError("resource report is incomplete")
    if record["board_id"] != board_id:
        raise PackageError(
            f"resource board {record['board_id']!r} does not match resolved {board_id!r}"
        )
    if record["cpu"] != cpu:
        raise PackageError(
            f"resource cpu {record['cpu']!r} does not match resolved {cpu!r}"
        )
    if record["profile_key"] != resolved_profile_key:
        raise PackageError("resource profile does not match resolved profile")

    try:
        catalog_snapshot = catalog.catalog_document(root_path)
    except (catalog.CatalogError, OSError, UnicodeError) as error:
        raise PackageError(f"cannot load canonical catalog: {error}") from error
    board_record = _catalog_board(catalog_snapshot, board_id)
    catalog_ini_file = board_record.get("ini_file")
    if not isinstance(catalog_ini_file, str) or ini_file != catalog_ini_file:
        raise PackageError("resolved ini_file does not match canonical catalog board")
    _validate_generated_profile(
        root=root_path,
        board=board_record,
        resolved_board_dir=resolved_board_dir,
        resolved_cpu=cpu,
        resolved_profile_key=resolved_profile_key,
    )

    source_commit = _git_head(root_path)
    if record["source_commit"] != source_commit:
        raise PackageError(
            "resource source commit does not match repository HEAD"
        )

    elf_path = root_path / "firmware" / "build" / "fome.elf"
    _nonempty_file(elf_path, "firmware ELF")
    if _sha256_file(elf_path) != record["elf_sha256"]:
        raise PackageError("resource ELF SHA-256 does not match firmware/build/fome.elf")

    binary_path = root_path / "firmware" / "deliver" / "fome.bin"
    ini_path = root_path / "firmware" / "tunerstudio" / "generated" / ini_file
    _nonempty_file(binary_path, "firmware binary")
    _nonempty_file(ini_path, "generated INI")
    try:
        ini_bytes = ini_path.read_bytes()
    except OSError as error:
        raise PackageError(f"cannot read generated INI: {error}") from error
    persistent = record.get("persistent")
    persistent_size = persistent.get("used_bytes") if isinstance(persistent, dict) else None
    if type(persistent_size) is not int or persistent_size <= PERSISTENT_WRAPPER_BYTES:
        raise PackageError(
            "resource report has no valid persistentState size for INI validation"
        )
    _validate_ini_layout(
        ini_bytes=ini_bytes,
        persistent_size_bytes=persistent_size,
        ve_rpm_count=resolved_profile["ve_rpm_count"],
        airmass_blend_size=resolved_profile["airmass_blend_size"],
    )
    embedded_ini = _embedded_ini(binary_path, ini_file)
    if embedded_ini is not None and embedded_ini != ini_bytes:
        raise PackageError("embedded INI does not match generated INI byte-for-byte")

    optional_srecs: list[tuple[Path, str]] = []
    for source_name, package_name in (
        ("fome_update.srec", "fome_update.srec"),
        ("fome_bl.srec", "fome_bl.srec"),
    ):
        candidate = root_path / "firmware" / "deliver" / source_name
        if candidate.is_file() and candidate.stat().st_size > 0:
            optional_srecs.append((candidate, package_name))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{output_path.name}.tmp-", dir=output_path.parent)
    )
    try:
        shutil.copyfile(binary_path, temporary / "fome.bin")
        shutil.copyfile(ini_path, temporary / ini_file)
        for source, name in optional_srecs:
            shutil.copyfile(source, temporary / name)
        shutil.copyfile(resources_file, temporary / "resources.json")
        shutil.copyfile(resolved_file, temporary / "resolved-profile.json")

        resource_index = matrix.merge_reports([record])
        _write_json(temporary / "resource-index.json", resource_index)
        _write_json(temporary / "catalog.json", catalog_snapshot)

        artifacts = [
            _artifact_record(path)
            for path in sorted(temporary.iterdir(), key=lambda item: item.name)
            if path.is_file()
        ]
        manifest: dict[str, Any] = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "workflow": "custom-firmware-build.yaml",
            "requested": {
                "request_id": request_id,
                "board": board_id,
                "features": ",".join(
                    module for module in optional_modules if module != "sd_card"
                ),
                "sd_card": "sd_card" in resolved_profile["modules"],
            },
            "resolved": {
                "board_id": board_id,
                "base_modules": base_modules,
                "optional_modules": optional_modules,
                "required_modules": required_modules,
                "modules": resolved_profile["modules"],
                "profile": resolved_profile,
            },
            "source": {
                "repository": os.environ.get(
                    "GITHUB_REPOSITORY", "capoworks/capoworks-ecu"
                ),
                "ref": _git_ref(root_path),
                "commit": source_commit,
            },
            "run": {
                "id": run_id,
                "attempt": run_attempt,
            },
            "artifacts": artifacts,
            "resource_check": _resource_summary(record),
            "resource_context": _resource_context(record),
            "measurements": _resource_measurements(record),
        }
        _write_json(temporary / "manifest.json", manifest)

        if output_path.exists() or output_path.is_symlink():
            raise PackageError(
                f"output appeared during packaging; refusing to replace it: {output_path}"
            )
        os.rename(temporary, output_path)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    package_parser = commands.add_parser(
        "package", help="validate and publish one firmware package"
    )
    package_parser.add_argument("--root", required=True)
    package_parser.add_argument("--resolved", required=True)
    package_parser.add_argument("--resources", required=True)
    package_parser.add_argument("--output", required=True)
    package_parser.add_argument("--request-id", required=True)
    package_parser.add_argument("--run-id")
    package_parser.add_argument("--run-attempt")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        package(
            root=args.root,
            resolved_path=args.resolved,
            resources_path=args.resources,
            output=args.output,
            request_id=args.request_id,
            run_id=args.run_id,
            run_attempt=args.run_attempt,
        )
    except (PackageError, OSError) as error:
        print(f"pipeline: error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
