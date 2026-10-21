#!/usr/bin/env python3
"""Bounded canonical board/profile resolver for custom web builds."""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCHEMA_VERSION = 1
BASE_MODULES = (
    "fan",
    "fuel_pump",
    "gear_detector",
    "tachometer",
    "trip_odometer",
    "map_averaging",
)
CPU_VALUES = {"ARCH_STM32F4", "ARCH_STM32F7", "ARCH_STM32H7"}
FEATURE_DEPENDENCIES = {"emp_pump": ("fan",)}
FEATURE_CONSTRAINTS = {
    "shock_preload": {
        "capabilities": {"can2": True},
        "message": "shock_preload requires CAN2 and is not supported on {board_id}",
    }
}
PROFILE_KEYS = {
    "modules",
    "cylinder_fuel_trim_count",
    "ve_rpm_count",
    "airmass_blend_size",
    "engine_preset_mode",
    "engine_presets",
    "engine_preset_default",
    "board",
    "cpu",
}
FRAGMENT_KEYS = {"config", "persistent", "output", "ini"}
ENGINE_PRESET_MODES = {
    "default_all",
    "default_none",
    "explicit_all",
    "explicit_none",
    "explicit",
}
TOKEN_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
HERE = Path(__file__).resolve().parent
DEFAULT_ROOT = HERE.parents[1]


class CatalogError(ValueError):
    """An invalid catalog, request, or canonical Make result."""


def _json_file(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CatalogError(f"cannot read {path}: {error}") from error


def _load_boards():
    document = _json_file(HERE / "boards.json")
    if document.get("schema_version") != SCHEMA_VERSION:
        raise CatalogError("boards.json has an unsupported schema_version")
    records = document.get("boards")
    if not isinstance(records, list) or not records:
        raise CatalogError("boards.json must contain a non-empty boards list")

    boards = {}
    for raw in records:
        if not isinstance(raw, dict):
            raise CatalogError("every board mapping must be an object")
        required = {"id", "board_dir", "compile_script", "ini_file", "cpu", "capabilities"}
        if set(raw) != required:
            raise CatalogError(f"board mapping has invalid fields: {sorted(raw)}")
        board_id = raw["id"]
        if not isinstance(board_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", board_id):
            raise CatalogError(f"invalid board id {board_id!r}")
        if board_id in boards:
            raise CatalogError(f"duplicate board id {board_id!r}")
        for field in ("board_dir", "compile_script", "ini_file"):
            value = raw[field]
            if not isinstance(value, str) or not value or "\x00" in value:
                raise CatalogError(f"invalid {field} for board {board_id!r}")
        relative = Path(raw["board_dir"])
        if relative.is_absolute() or ".." in relative.parts:
            raise CatalogError(f"board_dir for {board_id!r} must stay inside the repository")
        if Path(raw["compile_script"]).name != raw["compile_script"]:
            raise CatalogError(f"compile_script for {board_id!r} must be a file name")
        if Path(raw["ini_file"]).name != raw["ini_file"]:
            raise CatalogError(f"ini_file for {board_id!r} must be a file name")
        if raw["cpu"] not in CPU_VALUES:
            raise CatalogError(f"unsupported CPU {raw['cpu']!r} for board {board_id!r}")
        capabilities = raw["capabilities"]
        if not isinstance(capabilities, dict) or any(
            not isinstance(value, bool) for value in capabilities.values()
        ):
            raise CatalogError(f"capabilities for {board_id!r} must be boolean values")
        boards[board_id] = raw
    return boards


def _resolve_root(root):
    resolved = Path(root).expanduser().resolve() if root else DEFAULT_ROOT
    required = (
        resolved / "firmware/controllers/modules/module_profile.mk",
        resolved / "firmware/controllers/modules/module_profile_host.mk",
        resolved / "firmware/controllers/algo/engine_configuration.cpp",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise CatalogError(f"root is not a firmware checkout; missing: {', '.join(missing)}")
    return resolved


def _read_module_catalog(root):
    source = root / "firmware/controllers/modules/module_profile.mk"
    matches = re.findall(
        r"(?m)^\s*FOME_MODULE_CATALOG\s*:?=\s*([^#\r\n]+)",
        source.read_text(encoding="utf-8"),
    )
    if len(matches) != 1:
        raise CatalogError("module_profile.mk must define FOME_MODULE_CATALOG exactly once")
    modules = tuple(matches[0].split())
    if not modules or len(set(modules)) != len(modules):
        raise CatalogError("FOME_MODULE_CATALOG must contain unique module names")
    if any(not TOKEN_RE.fullmatch(module) for module in modules):
        raise CatalogError("FOME_MODULE_CATALOG contains an invalid module name")
    missing = [module for module in BASE_MODULES if module not in modules]
    if missing:
        raise CatalogError(f"FOME_MODULE_CATALOG is missing base modules: {' '.join(missing)}")
    return modules


def _engine_preset_symbols(root):
    source = (root / "firmware/controllers/algo/engine_configuration.cpp").read_text(
        encoding="utf-8"
    )
    start = source.find("switch (engineType)")
    if start < 0:
        raise CatalogError("engine_configuration.cpp has no engine preset dispatch")
    default = re.search(r"(?m)^\s*default\s*:", source[start:])
    dispatch = source[start : start + default.start()] if default else source[start:]
    symbols = []
    for symbol in re.findall(r"engine_type_e::([A-Za-z_][A-Za-z0-9_]*)", dispatch):
        if symbol not in symbols:
            symbols.append(symbol)
    if not symbols:
        raise CatalogError("could not derive engine preset symbols")
    return symbols


def _feature_capability_blocked(board, feature):
    constraint = FEATURE_CONSTRAINTS.get(feature)
    if not constraint:
        return False
    return any(
        board["capabilities"].get(capability) is not required
        for capability, required in constraint["capabilities"].items()
    )


def catalog_document(root):
    root = _resolve_root(root)
    catalog = _read_module_catalog(root)
    optional = tuple(module for module in catalog if module not in BASE_MODULES)
    boards = []
    for board in _load_boards().values():
        unsupported = _query_board_unsupported_modules(root, board, catalog)
        unsupported_set = set(unsupported)
        supported_optional = tuple(
            module
            for module in optional
            if module not in unsupported_set
            and not _feature_capability_blocked(board, module)
        )
        boards.append(
            {
                **board,
                "unsupported_modules": list(unsupported),
                "supported_optional_modules": list(supported_optional),
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "boards": boards,
        "base_modules": list(BASE_MODULES),
        "optional_modules": list(optional),
        "feature_dependencies": {
            feature: list(dependencies)
            for feature, dependencies in FEATURE_DEPENDENCIES.items()
        },
        "feature_constraints": FEATURE_CONSTRAINTS,
        "parameters": {
            "ve_rpm_count": {"default": 16, "values": [16, 32]},
            "airmass_blend_size": {"default": 8, "values": [8, 16]},
            "cylinder_trim_count": {
                "disabled_value": 0,
                "default_when_selected": 6,
                "values": list(range(1, 13)),
            },
            "engine_presets": {
                "default": "all",
                "default_when_expanded": "none",
                "expanded_when_any": ["ve_rpm_count", "airmass_blend_size"],
                "keywords": ["all", "none"],
                "exclusive_keywords": ["all", "none"],
                "mixed_selection_allowed": False,
                "explicit_selection_allowed_when_expanded": False,
                "minimal_symbol": "MINIMAL_PINS",
                "symbols": _engine_preset_symbols(root),
            },
            "features": {"max_length": 512},
            "request_id": {"pattern": REQUEST_ID_RE.pattern},
        },
    }


def parse_features(text, optional_modules):
    if not isinstance(text, str):
        raise CatalogError("features must be a string")
    if len(text) > 512:
        raise CatalogError("features exceeds 512 characters")
    if "\n" in text or "\r" in text:
        raise CatalogError("features must be a single line")
    text = text.strip()
    if not text:
        return []
    raw_tokens = text.split(",")
    if len(raw_tokens) > len(optional_modules):
        raise CatalogError(f"features contains more than {len(optional_modules)} tokens")

    optional = set(optional_modules)
    base = set(BASE_MODULES)
    selected = set()
    for raw in raw_tokens:
        token = raw.strip()
        if not token:
            raise CatalogError("features contains an empty token")
        if not re.fullmatch(r"[a-z_]+", token):
            raise CatalogError(f"invalid feature token {token!r}")
        if token in selected:
            raise CatalogError(f"duplicate module token {token!r}")
        if token in base:
            raise CatalogError(f"base module {token!r} is mandatory and must not be requested")
        if token not in optional:
            raise CatalogError(f"unknown or unsupported opt-in module {token!r}")
        selected.add(token)
    return [module for module in optional_modules if module in selected]


def validate_request_id(request_id):
    if request_id is not None and not REQUEST_ID_RE.fullmatch(request_id):
        raise CatalogError(
            "request_id must contain only letters, digits, underscores, and hyphens"
        )
    return request_id


def _normalize_engine_presets(value):
    if value is None:
        return None
    if "\n" in value or "\r" in value:
        raise CatalogError("engine-presets must be a single line")
    tokens = value.split()
    if not tokens:
        raise CatalogError("engine-presets must be 'all', 'none', or one or more symbols")
    if any(not TOKEN_RE.fullmatch(token) for token in tokens):
        raise CatalogError("engine-presets contains an invalid symbol")
    return " ".join(tokens)


def _enforce_feature_rules(board, selected_optional):
    for feature in selected_optional:
        constraint = FEATURE_CONSTRAINTS.get(feature)
        if not constraint:
            continue
        for capability, required in constraint["capabilities"].items():
            if board["capabilities"].get(capability) is not required:
                raise CatalogError(constraint["message"].format(board_id=board["id"]))


def _make_environment(temp_dir):
    path = os.environ.get("PATH", "/usr/bin:/bin")
    return {
        "PATH": path,
        "LC_ALL": "C",
        "LANG": "C",
        "TMPDIR": temp_dir,
        "MAKEFLAGS": "",
        "MFLAGS": "",
    }


def _make_error(output):
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    useful = [line for line in lines if "error" in line.lower() or line.startswith("ERROR:")]
    message = " | ".join(useful[-4:] or lines[-4:])
    return message[-2000:] or "no diagnostic output"


def _query_board_unsupported_modules(root, board, catalog):
    make = shutil.which("make")
    if not make:
        raise CatalogError("make is required to resolve board module support")
    project_dir = root / "firmware"
    board_dir = (root / board["board_dir"]).resolve()
    if not (board_dir / "board.mk").is_file():
        raise CatalogError(f"board.mk not found for {board['id']} at {board_dir}")
    stdin_makefile = (
        "BOARDS_DIR ?= $(PROJECT_DIR)/config/boards\n"
        "include $(BOARD_DIR)/board.mk\n"
        ".PHONY: print-unsupported\n"
        "print-unsupported:\n"
        "\t@printf 'FOME_MODULES_BOARD_UNSUPPORTED=%s\\n' "
        "'$(FOME_MODULES_BOARD_UNSUPPORTED)'\n"
    )
    with tempfile.TemporaryDirectory(prefix="fome-web-board-support-") as temp:
        try:
            result = subprocess.run(
                [
                    make,
                    "--silent",
                    "--no-print-directory",
                    "-f",
                    "-",
                    "print-unsupported",
                    f"PROJECT_DIR={project_dir}",
                    f"BOARD_DIR={board_dir}",
                    f"PROJECT_CPU={board['cpu']}",
                ],
                cwd=temp,
                env=_make_environment(temp),
                input=stdin_makefile,
                check=False,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=30,
            )
        except subprocess.TimeoutExpired as error:
            raise CatalogError(
                f"board support query timed out for {board['id']}"
            ) from error
    if result.returncode:
        raise CatalogError(
            f"board support query failed for {board['id']}: "
            + _make_error(result.stderr + "\n" + result.stdout)
        )
    marker = "FOME_MODULES_BOARD_UNSUPPORTED="
    values = [
        line[len(marker) :].strip()
        for line in result.stdout.splitlines()
        if line.startswith(marker)
    ]
    if len(values) != 1:
        raise CatalogError(
            f"board support query for {board['id']} returned no unique result"
        )
    unsupported = values[0].split()
    if len(unsupported) != len(set(unsupported)):
        raise CatalogError(
            f"board support query for {board['id']} returned duplicate modules"
        )
    unknown = sorted(set(unsupported) - set(catalog))
    if unknown:
        raise CatalogError(
            f"board {board['id']} declares unsupported unknown modules: "
            + ", ".join(unknown)
        )
    selected = set(unsupported)
    return tuple(module for module in catalog if module in selected)


def _run_make_profile(root, board, modules, ve_rpm_count, airmass_blend_size,
                      cylinder_trim_count, engine_presets):
    make = shutil.which("make")
    if not make:
        raise CatalogError("make is required to resolve the canonical module profile")
    host_makefile = root / "firmware/controllers/modules/module_profile_host.mk"
    project_dir = root / "firmware"
    board_dir = (root / board["board_dir"]).resolve()
    if not (board_dir / "board.mk").is_file():
        raise CatalogError(f"board.mk not found for {board['id']} at {board_dir}")
    if not (board_dir / board["compile_script"]).is_file():
        raise CatalogError(
            f"compile wrapper {board['compile_script']} not found for {board['id']}"
        )

    with tempfile.TemporaryDirectory(prefix="fome-web-catalog-") as temp:
        temp_dir = Path(temp)
        profile_file = temp_dir / "module_profile.txt"
        args = [
            make,
            "--no-print-directory",
            "-f",
            str(host_makefile),
            "module-profile",
            f"PROJECT_DIR={project_dir}",
            f"BOARD_DIR={board_dir}",
            f"PROJECT_CPU={board['cpu']}",
            f"FOME_MODULES={' '.join(modules)}",
            f"FOME_VE_RPM_COUNT={ve_rpm_count}",
            f"FOME_AIRMASS_BLEND_SIZE={airmass_blend_size}",
            f"MODULE_PROFILE_FILE={profile_file}",
        ]
        if cylinder_trim_count is not None:
            args.append(f"FOME_CYLINDER_TRIM_COUNT={cylinder_trim_count}")
        if engine_presets is not None:
            args.append(f"FOME_ENGINE_PRESETS={engine_presets}")
        try:
            result = subprocess.run(
                args,
                cwd=temp_dir,
                env=_make_environment(str(temp_dir)),
                check=False,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=60,
            )
        except subprocess.TimeoutExpired as error:
            raise CatalogError("canonical Make profile resolution timed out") from error
        if result.returncode:
            raise CatalogError(
                "canonical module profile rejected configuration: "
                + _make_error(result.stderr + "\n" + result.stdout)
            )
        if not profile_file.is_file():
            raise CatalogError("canonical Make resolver did not produce a module profile")
        return _parse_profile(profile_file)


def _profile_integer(values, key, allowed):
    raw = values[key]
    if not re.fullmatch(r"[0-9]+", raw):
        raise CatalogError(f"profile {key} is not an integer: {raw!r}")
    try:
        value = int(raw)
    except ValueError as error:
        raise CatalogError(f"profile {key} is not an integer: {raw!r}") from error
    if value not in allowed:
        raise CatalogError(f"profile {key} is invalid: {value}")
    return value


def _parse_profile(path):
    values = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise CatalogError(f"cannot read generated module profile: {error}") from error
    for number, line in enumerate(lines, 1):
        if not line or " " not in line:
            raise CatalogError(f"malformed module profile line {number}: {line!r}")
        key, value = line.split(" ", 1)
        if key in FRAGMENT_KEYS:
            continue
        if key not in PROFILE_KEYS:
            raise CatalogError(f"unknown module profile field {key!r}")
        if key in values:
            raise CatalogError(f"duplicate module profile field {key!r}")
        values[key] = value
    missing = sorted(PROFILE_KEYS - set(values))
    if missing:
        raise CatalogError(f"incomplete module profile; missing: {', '.join(missing)}")

    modules = values["modules"].split()
    if len(set(modules)) != len(modules) or any(
        not TOKEN_RE.fullmatch(module) for module in modules
    ):
        raise CatalogError("generated profile has an invalid module list")
    presets = values["engine_presets"].split()
    if not presets or len(set(presets)) != len(presets) or any(
        not TOKEN_RE.fullmatch(preset) for preset in presets
    ):
        raise CatalogError("generated profile has an invalid engine preset list")
    mode = values["engine_preset_mode"]
    if mode not in ENGINE_PRESET_MODES:
        raise CatalogError(f"generated profile has invalid engine preset mode {mode!r}")
    default = values["engine_preset_default"]
    if not TOKEN_RE.fullmatch(default):
        raise CatalogError("generated profile has an invalid engine preset default")
    if not values["board"] or not values["cpu"]:
        raise CatalogError("generated profile has an empty board or CPU")

    profile = {
        "modules": modules,
        "ve_rpm_count": _profile_integer(values, "ve_rpm_count", {16, 32}),
        "airmass_blend_size": _profile_integer(values, "airmass_blend_size", {8, 16}),
        "cylinder_trim_count": _profile_integer(
            values, "cylinder_fuel_trim_count", set(range(13))
        ),
        "engine_preset_mode": mode,
        "engine_presets": presets,
        "engine_preset_default": default,
    }
    return profile, values["board"], values["cpu"]


def resolve(root, board_id, features, ve_rpm_count=16, airmass_blend_size=8,
            cylinder_trim_count=None, engine_presets=None, request_id=None):
    root = _resolve_root(root)
    validate_request_id(request_id)
    catalog = _read_module_catalog(root)
    optional_catalog = tuple(module for module in catalog if module not in BASE_MODULES)
    selected_optional = parse_features(features, optional_catalog)
    boards = _load_boards()
    board = boards.get(board_id)
    if board is None:
        raise CatalogError(f"unknown board id {board_id!r}")
    if (
        not isinstance(ve_rpm_count, int)
        or isinstance(ve_rpm_count, bool)
        or ve_rpm_count not in {16, 32}
    ):
        raise CatalogError("ve-rpm-count must be 16 or 32")
    if (
        not isinstance(airmass_blend_size, int)
        or isinstance(airmass_blend_size, bool)
        or airmass_blend_size not in {8, 16}
    ):
        raise CatalogError("airmass-blend-size must be 8 or 16")
    if cylinder_trim_count is not None and (
        not isinstance(cylinder_trim_count, int)
        or isinstance(cylinder_trim_count, bool)
        or cylinder_trim_count not in range(1, 13)
    ):
        raise CatalogError("cylinder-trim-count must be an integer from 1 to 12")
    engine_presets = _normalize_engine_presets(engine_presets)

    modules = list(BASE_MODULES) + selected_optional
    _enforce_feature_rules(board, selected_optional)
    profile, profile_board, profile_cpu = _run_make_profile(
        root,
        board,
        modules,
        ve_rpm_count,
        airmass_blend_size,
        cylinder_trim_count,
        engine_presets,
    )
    expected_board = (root / board["board_dir"]).resolve()
    if Path(profile_board).resolve() != expected_board:
        raise CatalogError("canonical profile resolved a different board directory")
    if profile_cpu != board["cpu"]:
        raise CatalogError(
            f"canonical profile CPU {profile_cpu!r} does not match board CPU {board['cpu']!r}"
        )
    unknown_modules = [module for module in profile["modules"] if module not in catalog]
    if unknown_modules:
        raise CatalogError(f"canonical profile returned unknown modules: {unknown_modules}")

    resolved_optional = [
        module for module in optional_catalog if module in profile["modules"]
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "board_id": board_id,
        "board_dir": board["board_dir"],
        "compile_script": board["compile_script"],
        "ini_file": board["ini_file"],
        "cpu": board["cpu"],
        "base_modules": list(BASE_MODULES),
        "optional_modules": resolved_optional,
        "profile": profile,
    }


def _env_value(value):
    text = str(value)
    if "\n" in text or "\r" in text or "\x00" in text:
        raise CatalogError("GitHub environment values must be a single safe line")
    return text


def _preset_environment_value(profile):
    mode = profile["engine_preset_mode"]
    selected = profile["engine_presets"]
    if mode in {"default_all", "explicit_all"}:
        return "all"
    if selected == ["MINIMAL_PINS"]:
        return "none"
    return " ".join(selected)


def github_environment(document, request_id=None):
    profile = document["profile"]
    fields = []
    if request_id is not None:
        fields.append(("REQUEST_ID", request_id))
    fields.extend(
        [
            ("BOARD_ID", document["board_id"]),
            ("BOARD_DIR", document["board_dir"]),
            ("COMPILE_SCRIPT", document["compile_script"]),
            ("INI_FILE", document["ini_file"]),
            ("REQUESTED_FEATURES", " ".join(document["optional_modules"])),
            ("BASE_MODULES", " ".join(document["base_modules"])),
            (
                "SELECTED_OPTIONAL_MODULES",
                " ".join(document["optional_modules"]),
            ),
            ("FOME_MODULES", " ".join(profile["modules"])),
            ("FOME_VE_RPM_COUNT", profile["ve_rpm_count"]),
            ("FOME_AIRMASS_BLEND_SIZE", profile["airmass_blend_size"]),
            ("FOME_CYLINDER_TRIM_COUNT", profile["cylinder_trim_count"]),
            ("FOME_ENGINE_PRESETS", _preset_environment_value(profile)),
        ]
    )
    return "".join(f"{name}={_env_value(value)}\n" for name, value in fields)


def _write_json(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
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


def _add_root(parser):
    parser.add_argument(
        "--root",
        default=None,
        help="firmware repository root (defaults to the directory containing misc/)",
    )


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    catalog = commands.add_parser("catalog", help="emit the canonical web build catalog")
    _add_root(catalog)
    resolve_parser = commands.add_parser("resolve", help="resolve one build profile")
    _add_root(resolve_parser)
    resolve_parser.add_argument("--board", required=True)
    resolve_parser.add_argument("--features", required=True)
    resolve_parser.add_argument("--ve-rpm-count", type=int, choices=(16, 32), default=16)
    resolve_parser.add_argument(
        "--airmass-blend-size", type=int, choices=(8, 16), default=8
    )
    resolve_parser.add_argument(
        "--cylinder-trim-count", type=int, choices=range(1, 13), default=None
    )
    resolve_parser.add_argument("--engine-presets", default=None)
    resolve_parser.add_argument("--request-id", default=None)
    resolve_parser.add_argument("--output", default=None)
    resolve_parser.add_argument("--github-env", default=None)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        root = _resolve_root(args.root)
        if args.command == "catalog":
            document = catalog_document(root)
        else:
            document = resolve(
                root,
                args.board,
                args.features,
                args.ve_rpm_count,
                args.airmass_blend_size,
                args.cylinder_trim_count,
                args.engine_presets,
                args.request_id,
            )
        if args.command == "catalog" or not args.output:
            print(json.dumps(document, indent=2, sort_keys=True))
        if args.command == "resolve":
            if args.output:
                _write_json(args.output, document)
            if args.github_env:
                Path(args.github_env).parent.mkdir(parents=True, exist_ok=True)
                with Path(args.github_env).open("a", encoding="utf-8", newline="\n") as handle:
                    handle.write(github_environment(document, args.request_id))
    except (CatalogError, OSError) as error:
        print(f"catalog: error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
