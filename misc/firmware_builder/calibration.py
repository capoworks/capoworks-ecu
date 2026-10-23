#!/usr/bin/env python3
"""Plan firmware resource calibration builds and collect their ELF evidence."""

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
    from . import catalog, matrix, pipeline, resources
except ImportError:  # Allow direct execution from this directory.
    import catalog  # type: ignore[no-redef]
    import matrix  # type: ignore[no-redef]
    import pipeline  # type: ignore[no-redef]
    import resources  # type: ignore[no-redef]


COVERAGES = ("baseline", "singles", "full", "expanded")
EXPANDED_DIMENSIONS = ((16, 8), (32, 8), (16, 16), (32, 16))
PROFILE_ID_RE = re.compile(r"^[A-Za-z0-9._~-]+$")


class CalibrationError(RuntimeError):
    """A calibration plan or collection request violates its contract."""


def _write_json(path: str | Path, document: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", dir=destination.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(document, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _document_catalog(root: str | Path) -> dict[str, Any]:
    try:
        document = catalog.catalog_document(Path(root).expanduser().resolve())
    except (catalog.CatalogError, OSError, UnicodeError) as error:
        raise CalibrationError(f"cannot load source catalog: {error}") from error
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != catalog.SCHEMA_VERSION
        or not isinstance(document.get("boards"), list)
    ):
        raise CalibrationError("source catalog has an unsupported document shape")
    return document


def _board(
    document: dict[str, Any], board_id: str
) -> dict[str, Any]:
    matches = [
        board
        for board in document["boards"]
        if isinstance(board, dict) and board.get("id") == board_id
    ]
    if len(matches) != 1:
        raise CalibrationError(f"unknown board id {board_id!r}")
    return matches[0]


def _supported_optional(
    document: dict[str, Any], board: dict[str, Any]
) -> list[str]:
    optional = document.get("optional_modules")
    supported = board.get("supported_optional_modules")
    if not isinstance(optional, list) or not isinstance(supported, list):
        raise CalibrationError("source catalog has no canonical optional module lists")
    optional_set = set(optional)
    if len(optional_set) != len(optional):
        raise CalibrationError("source catalog optional module list contains duplicates")
    supported_set = set(supported)
    if not supported_set.issubset(optional_set):
        unexpected = sorted(supported_set - optional_set)
        raise CalibrationError(
            f"board {board.get('id')!r} has modules outside the source catalog: "
            + ", ".join(unexpected)
        )
    return [module for module in optional if module in supported_set]


def _required_modules(
    document: dict[str, Any], board: dict[str, Any]
) -> list[str]:
    optional = document.get("optional_modules")
    required = board.get("required_modules")
    if not isinstance(optional, list) or not isinstance(required, list):
        raise CalibrationError(
            "source catalog has no canonical required module metadata"
        )
    if any(not isinstance(module, str) for module in required):
        raise CalibrationError(
            f"board {board.get('id')!r} has invalid required module metadata"
        )
    if len(required) != len(set(required)):
        raise CalibrationError(
            f"board {board.get('id')!r} has duplicate required modules"
        )
    required_set = set(required)
    if not required_set.issubset(set(optional)):
        unexpected = sorted(required_set - set(optional))
        raise CalibrationError(
            f"board {board.get('id')!r} requires unknown modules: "
            + ", ".join(unexpected)
        )
    return [module for module in optional if module in required_set]


def _board_modules(
    document: dict[str, Any], board: dict[str, Any]
) -> tuple[list[str], list[str]]:
    supported = _supported_optional(document, board)
    required = _required_modules(document, board)
    missing = [module for module in required if module not in set(supported)]
    if missing:
        raise CalibrationError(
            f"board {board.get('id')!r} requires unsupported modules: "
            + ", ".join(missing)
        )
    return supported, required


def _feature_csv(
    document: dict[str, Any],
    selected: Iterable[str],
    sd_card: bool,
) -> str:
    optional = document.get("optional_modules")
    if not isinstance(optional, list):
        raise CalibrationError("source catalog has no canonical optional module list")
    selected_values = list(selected)
    selected_set = set(selected_values)
    if len(selected_set) != len(selected_values):
        raise CalibrationError("calibration profile contains duplicate modules")
    if not selected_set.issubset(set(optional)):
        raise CalibrationError("calibration profile contains an unknown module")
    if ("sd_card" in selected_set) is not sd_card:
        raise CalibrationError(
            "calibration profile sd_card flag does not match its effective modules"
        )
    return ",".join(
        module
        for module in optional
        if module in selected_set and module != "sd_card"
    )


def _preset_value(profile: dict[str, Any]) -> str:
    mode = profile.get("engine_preset_mode")
    presets = profile.get("engine_presets")
    if mode in {"default_all", "explicit_all"}:
        return "all"
    if presets == ["MINIMAL_PINS"]:
        return "none"
    if not isinstance(presets, list) or not presets:
        raise CalibrationError("resolved profile has no engine preset selection")
    return " ".join(presets)


def _profile_id(
    board_id: str,
    coverage: str,
    selected: list[str],
    required: list[str],
    ve_rpm_count: int,
    airmass_blend_size: int,
    expanded_full: bool,
) -> str:
    if coverage == "baseline" or (
        coverage in {"singles", "full"}
        and set(selected) == set(required)
    ):
        suffix = "baseline"
    elif coverage == "singles":
        required_set = set(required)
        single = [module for module in selected if module not in required_set]
        if len(single) != 1:
            raise CalibrationError("a singles profile must select exactly one module")
        suffix = "single-" + single[0].replace("_", "-")
    elif coverage == "full":
        suffix = "full"
    elif expanded_full:
        suffix = f"expanded-ve{ve_rpm_count}-am{airmass_blend_size}-full"
    else:
        suffix = f"expanded-ve{ve_rpm_count}-am{airmass_blend_size}-base"
    profile_id = f"{board_id}-{suffix}"
    if not PROFILE_ID_RE.fullmatch(profile_id):
        raise CalibrationError(f"profile id is not URL-safe: {profile_id!r}")
    return profile_id


def _profile_specs(
    document: dict[str, Any],
    board: dict[str, Any],
    coverage: str,
) -> list[dict[str, Any]]:
    supported, required = _board_modules(document, board)
    required_set = set(required)

    def selected_with(module: str | None = None) -> list[str]:
        selected = set(required_set)
        if module is not None:
            selected.add(module)
        return [item for item in supported if item in selected]

    def spec(
        selected: list[str],
        ve_rpm_count: int,
        airmass_blend_size: int,
        cylinder_trim_count: int,
        engine_presets: str | None,
        *,
        expanded_full: bool = False,
    ) -> dict[str, Any]:
        return {
            "selected": selected,
            "sd_card": "sd_card" in selected,
            "ve_rpm_count": ve_rpm_count,
            "airmass_blend_size": airmass_blend_size,
            "cylinder_trim_count": cylinder_trim_count,
            "engine_presets": engine_presets,
            **({"expanded_full": True} if expanded_full else {}),
        }

    if coverage == "baseline":
        return [
            spec(selected_with(), 16, 8, 0, None)
        ]
    if coverage == "singles":
        specs = [
            spec(selected_with(), 16, 8, 0, None)
        ]
        for module in supported:
            if module in required_set:
                continue
            specs.append(
                spec(
                    selected_with(module),
                    16,
                    8,
                    (
                        6 if module == "cylinder_fuel_trim" else 0
                    ),
                    None,
                )
            )
        return specs
    if coverage == "full":
        return [
            spec(selected_with(), 16, 8, 0, None),
            spec(
                supported,
                16,
                8,
                6 if "cylinder_fuel_trim" in supported else 0,
                None,
            ),
        ]
    if coverage != "expanded":
        raise CalibrationError(f"unknown calibration coverage {coverage!r}")

    specs = [
        spec(
            selected_with(),
            ve_rpm_count,
            airmass_blend_size,
            0,
            "none",
        )
        for ve_rpm_count, airmass_blend_size in EXPANDED_DIMENSIONS
    ]
    specs.append(
        spec(
            supported,
            32,
            16,
            6 if "cylinder_fuel_trim" in supported else 0,
            "none",
            expanded_full=True,
        )
    )
    return specs


def _resolve_record(
    root: Path,
    document: dict[str, Any],
    board: dict[str, Any],
    coverage: str,
    spec: dict[str, Any],
) -> dict[str, Any]:
    board_id = board["id"]
    features = _feature_csv(document, spec["selected"], spec["sd_card"])
    try:
        resolved = catalog.resolve(
            root,
            board_id,
            features,
            ve_rpm_count=spec["ve_rpm_count"],
            airmass_blend_size=spec["airmass_blend_size"],
            cylinder_trim_count=(
                spec["cylinder_trim_count"]
                if spec["cylinder_trim_count"] != 0
                else None
            ),
            engine_presets=spec["engine_presets"],
            sd_card=spec["sd_card"],
        )
    except (catalog.CatalogError, OSError, UnicodeError) as error:
        raise CalibrationError(
            f"could not resolve calibration profile for {board_id!r}: {error}"
        ) from error

    if resolved.get("board_id") != board_id:
        raise CalibrationError(
            f"canonical resolver returned board {resolved.get('board_id')!r} "
            f"for requested board {board_id!r}"
        )
    if resolved.get("cpu") != board.get("cpu"):
        raise CalibrationError(
            f"canonical resolver returned CPU {resolved.get('cpu')!r} "
            f"for board {board_id!r}"
        )
    if resolved.get("required_modules") != board.get("required_modules"):
        raise CalibrationError(
            f"canonical resolver returned unexpected required modules for "
            f"{board_id!r}"
        )
    expected_optional = [
        module
        for module in document["optional_modules"]
        if module in set(spec["selected"])
    ]
    if expected_optional != spec["selected"]:
        raise CalibrationError(
            f"calibration profile for {board_id!r} is not in catalog order"
        )
    if resolved.get("optional_modules") != expected_optional:
        raise CalibrationError(
            f"canonical resolver selected unexpected modules for {board_id!r}"
        )
    profile = resolved.get("profile")
    if not isinstance(profile, dict):
        raise CalibrationError(f"canonical resolver returned no profile for {board_id!r}")
    expected_modules = list(document["base_modules"]) + expected_optional
    if profile.get("modules") != expected_modules:
        raise CalibrationError(
            f"canonical profile modules do not match the calibration selection "
            f"for {board_id!r}"
        )
    expected_values = {
        "ve_rpm_count": spec["ve_rpm_count"],
        "airmass_blend_size": spec["airmass_blend_size"],
        "cylinder_trim_count": spec["cylinder_trim_count"],
    }
    for key, expected in expected_values.items():
        if profile.get(key) != expected:
            raise CalibrationError(
                f"canonical profile {key} is {profile.get(key)!r}, expected {expected!r}"
            )
    presets = _preset_value(profile)
    expected_presets = spec["engine_presets"] or "all"
    if presets != expected_presets:
        raise CalibrationError(
            f"canonical profile presets are {presets!r}, expected {expected_presets!r}"
        )

    profile_id = _profile_id(
        board_id,
        coverage,
        spec["selected"],
        board.get("required_modules", []),
        spec["ve_rpm_count"],
        spec["airmass_blend_size"],
        bool(spec.get("expanded_full", False)),
    )
    return {
        "board_id": board_id,
        "profile_id": profile_id,
        "features": features,
        "sd_card": spec["sd_card"],
        "ve_rpm_count": profile["ve_rpm_count"],
        "airmass_blend_size": profile["airmass_blend_size"],
        "cylinder_trim_count": profile["cylinder_trim_count"],
        "engine_presets": presets,
    }


def plan(
    *,
    root: str | Path,
    board: str,
    coverage: str,
    output: str | Path | None = None,
) -> dict[str, Any]:
    """Return a deterministic GitHub matrix for bounded resource calibration."""
    if coverage not in COVERAGES:
        raise CalibrationError(f"unknown calibration coverage {coverage!r}")
    root_path = Path(root).expanduser().resolve()
    document = _document_catalog(root_path)
    if board == "all":
        boards = document["boards"]
    else:
        boards = [_board(document, board)]

    records = []
    identities = set()
    profile_ids = set()
    for board_record in boards:
        for spec in _profile_specs(document, board_record, coverage):
            record = _resolve_record(
                root_path, document, board_record, coverage, spec
            )
            identity = (
                record["board_id"],
                record["features"],
                record["sd_card"],
                record["ve_rpm_count"],
                record["airmass_blend_size"],
                record["cylinder_trim_count"],
                record["engine_presets"],
            )
            if identity in identities:
                raise CalibrationError(
                    f"duplicate calibration profile for {record['board_id']!r}"
                )
            if record["profile_id"] in profile_ids:
                raise CalibrationError(
                    f"duplicate calibration profile id {record['profile_id']!r}"
                )
            identities.add(identity)
            profile_ids.add(record["profile_id"])
            records.append(record)

    document_out = {"include": records}
    if output is not None:
        _write_json(output, document_out)
    return document_out


def _read_policy(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CalibrationError(f"cannot read resource policy {path}: {error}") from error


def _policy_for(root: Path, board_id: str) -> Path:
    policies = root / "misc" / "firmware_builder" / "policies"
    board_policy = policies / f"{board_id}.json"
    return board_policy if board_policy.is_file() else policies / "default.json"


def _generated_profile(
    root: Path,
    board: dict[str, Any],
    document: dict[str, Any],
) -> str:
    """Parse and validate the build's canonical module profile."""
    board_id = board.get("id")
    board_dir = board.get("board_dir")
    expected_cpu = board.get("cpu")
    if (
        not isinstance(board_id, str)
        or not isinstance(board_dir, str)
        or not isinstance(expected_cpu, str)
    ):
        raise CalibrationError(f"source catalog has an incomplete board {board_id!r}")

    profile_path = root / "firmware" / "generated" / "module_profile.txt"
    try:
        profile, generated_board, generated_cpu = catalog._parse_profile(profile_path)
    except (catalog.CatalogError, OSError, UnicodeError, ValueError) as error:
        raise CalibrationError(
            f"cannot validate generated module profile: {error}"
        ) from error

    required = board.get("required_modules")
    unsupported = board.get("unsupported_modules")
    base_modules = document.get("base_modules")
    optional_modules = document.get("optional_modules")
    if (
        not isinstance(required, list)
        or not isinstance(unsupported, list)
        or not isinstance(base_modules, list)
        or not isinstance(optional_modules, list)
    ):
        raise CalibrationError(
            f"source catalog has incomplete module metadata for board {board_id!r}"
        )
    missing_required = [
        module for module in required if module not in profile["modules"]
    ]
    if missing_required:
        raise CalibrationError(
            f"generated module profile is missing required modules for "
            f"{board_id!r}: {', '.join(missing_required)}"
        )
    unsupported_selected = [
        module for module in profile["modules"] if module in set(unsupported)
    ]
    if unsupported_selected:
        raise CalibrationError(
            f"generated module profile selects unsupported modules for "
            f"{board_id!r}: {', '.join(unsupported_selected)}"
        )
    module_catalog = base_modules + optional_modules
    unknown_modules = [
        module for module in profile["modules"] if module not in set(module_catalog)
    ]
    if unknown_modules:
        raise CalibrationError(
            f"generated module profile has unknown modules: "
            f"{', '.join(unknown_modules)}"
        )
    selected_modules = set(profile["modules"])
    expected_order = [
        module for module in module_catalog if module in selected_modules
    ]
    if profile["modules"] != expected_order:
        raise CalibrationError(
            "generated module profile is not in canonical catalog order"
        )

    try:
        generated_board_path = Path(generated_board).expanduser()
        if not generated_board_path.is_absolute():
            generated_board_path = root / generated_board_path
        generated_board_path = generated_board_path.resolve()
        expected_board_path = (root / board_dir).resolve()
    except (OSError, ValueError) as error:
        raise CalibrationError(
            f"cannot normalize generated module profile board: {error}"
        ) from error
    if generated_board_path != expected_board_path:
        raise CalibrationError(
            f"generated module profile board {generated_board_path} "
            f"does not match catalog board {expected_board_path}"
        )
    if generated_cpu != expected_cpu:
        raise CalibrationError(
            f"generated module profile CPU {generated_cpu!r} "
            f"does not match catalog CPU {expected_cpu!r}"
        )
    try:
        profile_key = matrix.profile_key(profile)
    except matrix.MatrixError as error:
        raise CalibrationError(f"invalid generated module profile: {error}") from error
    return profile_key


def _diagnostic_report(
    *,
    board_id: str,
    cpu: str,
    source_commit: str,
    profile_path: Path,
    elf_path: Path,
    policy_data: Any,
    message: str,
) -> dict[str, Any] | None:
    try:
        profile = resources.parse_profile(
            profile_path.read_text(encoding="utf-8")
        )
    except (OSError, UnicodeError, resources.ExtractionError):
        return None

    allowed_regions = set(resources.ALL_RAM_REGIONS) | {"flash", "persistent"}
    try:
        policy = resources.normalize_policy(policy_data, allowed_regions)
    except resources.ExtractionError as error:
        policy = resources.normalize_policy(None, allowed_regions)
        policy["error"] = str(error)
    try:
        elf_sha256 = resources.sha256_file(elf_path)
    except OSError:
        elf_sha256 = None
    toolchain_id = hashlib.sha256(
        f"unavailable\0{board_id}\0{cpu}".encode("utf-8")
    ).hexdigest()
    return {
        "schema_version": resources.SCHEMA_VERSION,
        "board_id": board_id,
        "cpu": cpu,
        "source_commit": source_commit,
        "toolchain": {
            "id": toolchain_id,
            "version": None,
            "sha256": None,
        },
        "profile": profile,
        "elf_sha256": elf_sha256,
        "regions": {},
        "heap": {},
        "persistent": {},
        "policy": policy,
        "status": "rejected",
        "violations": [{"code": "extraction_failed", "message": message}],
        "hardware_guarantee": False,
        "note": "Build-time technical-fit result only; not a runtime or hardware guarantee.",
    }


def collect(
    *,
    root: str | Path,
    board: str,
    output: str | Path,
) -> dict[str, Any]:
    """Extract the current build's resources and write its evidence artifacts."""
    root_path = Path(root).expanduser().resolve()
    output_path = Path(output).expanduser().resolve()
    output_path.mkdir(parents=True, exist_ok=True)

    document = _document_catalog(root_path)
    board_record = _board(document, board)
    cpu = board_record.get("cpu")
    if not isinstance(cpu, str) or cpu not in resources.CPU_LIMITS:
        raise CalibrationError(f"source catalog has an invalid CPU for board {board!r}")
    generated_profile_key = _generated_profile(
        root_path, board_record, document
    )
    _write_json(output_path / "catalog.json", document)

    elf_path = root_path / "firmware" / "build" / "fome.elf"
    profile_path = root_path / "firmware" / "generated" / "module_profile.txt"
    policy_path = _policy_for(root_path, board)
    try:
        policy_data = _read_policy(policy_path)
    except CalibrationError:
        # Extraction reports the policy failure below; keep collecting so the
        # diagnostic artifact is still written.
        policy_data = None
    try:
        source_commit = pipeline._git_head(root_path)
    except pipeline.PackageError as error:
        raise CalibrationError(str(error)) from error

    report = None
    try:
        tools = resources.resolve_tools(root_path)
        report = resources.extract_report(
            elf=elf_path,
            profile_path=profile_path,
            board_id=board,
            cpu=cpu,
            source_commit=source_commit,
            tools=tools,
            policy_path=policy_path,
        )
    except (resources.ExtractionError, OSError, UnicodeError) as error:
        report = _diagnostic_report(
            board_id=board,
            cpu=cpu,
            source_commit=source_commit,
            profile_path=profile_path,
            elf_path=elf_path,
            policy_data=policy_data,
            message=str(error),
        )
        if report is None:
            report = {
                "schema_version": resources.SCHEMA_VERSION,
                "board_id": board,
                "cpu": cpu,
                "source_commit": source_commit,
                "status": "rejected",
                "violations": [
                    {
                        "code": "extraction_failed",
                        "message": str(error),
                    }
                ],
                "hardware_guarantee": False,
            }

    if report.get("board_id") != board:
        raise CalibrationError(
            f"resource report board {report.get('board_id')!r} "
            f"does not match requested board {board!r}"
        )
    if report.get("cpu") != cpu:
        raise CalibrationError(
            f"resource report CPU {report.get('cpu')!r} does not match catalog CPU {cpu!r}"
        )
    report_profile = report.get("profile")
    if isinstance(report_profile, dict):
        try:
            report_profile_key = matrix.profile_key(report_profile)
        except matrix.MatrixError as error:
            raise CalibrationError(
                f"resource report has an invalid generated profile: {error}"
            ) from error
        if report_profile_key != generated_profile_key:
            raise CalibrationError(
                "resource report profile does not match generated module profile"
            )

    _write_json(output_path / "resources.json", report)
    try:
        index = matrix.merge_reports([report])
    except matrix.MatrixError as error:
        if report.get("status") == "accepted":
            raise CalibrationError(
                f"accepted resource evidence cannot be indexed: {error}"
            ) from error
        index = matrix.merge_reports([])
    _write_json(output_path / "resource-index.json", index)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    plan_parser = commands.add_parser(
        "plan", help="emit a resource calibration GitHub matrix"
    )
    plan_parser.add_argument("--root", required=True)
    plan_parser.add_argument("--board", required=True)
    plan_parser.add_argument("--coverage", choices=COVERAGES, default="singles")
    plan_parser.add_argument("--output")

    collect_parser = commands.add_parser(
        "collect", help="collect the current firmware build's resource evidence"
    )
    collect_parser.add_argument("--root", required=True)
    collect_parser.add_argument("--board", required=True)
    collect_parser.add_argument("--output", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "plan":
            document = plan(
                root=args.root,
                board=args.board,
                coverage=args.coverage,
                output=args.output,
            )
            if args.output is None:
                print(json.dumps(document, indent=2, sort_keys=True))
            return 0
        report = collect(root=args.root, board=args.board, output=args.output)
        return 0 if report.get("status") == "accepted" else 1
    except (
        CalibrationError,
        catalog.CatalogError,
        matrix.MatrixError,
        pipeline.PackageError,
        resources.ExtractionError,
        OSError,
        UnicodeError,
    ) as error:
        print(f"calibration: error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
