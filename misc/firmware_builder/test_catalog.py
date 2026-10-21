"""Tests for the bounded custom-firmware catalog resolver."""

import contextlib
import importlib.util
import io
import json
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SPEC = importlib.util.spec_from_file_location("firmware_catalog", HERE / "catalog.py")
catalog = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = catalog
SPEC.loader.exec_module(catalog)

BOARD_IDS = [
    "hellen72",
    "hellen-nb1",
    "hellenNA8_96",
    "hellenNA6",
    "hellen128",
    "mre_f4",
    "mre_f7",
    "proteus_f4",
    "proteus_f7",
    "proteus_h7",
    "stm32f429_nucleo",
    "f407-discovery",
    "atlas",
    "core8",
]
CAN2_BOARDS = {"core8", "atlas", "proteus_f4", "proteus_f7", "proteus_h7"}
EXPECTED_MISSING_INI = {"stm32f429_nucleo"}


@unittest.skipUnless(shutil.which("make"), "host make is required")
class CatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert catalog.DEFAULT_ROOT == ROOT
        cls.document = catalog.catalog_document(ROOT)
        cls.optional = cls.document["optional_modules"]

    def test_all_14_board_mappings_are_valid(self):
        boards = self.document["boards"]
        self.assertEqual([board["id"] for board in boards], BOARD_IDS)
        for board in boards:
            with self.subTest(board=board["id"]):
                board_dir = ROOT / board["board_dir"]
                self.assertTrue((board_dir / "board.mk").is_file())
                wrapper = board_dir / board["compile_script"]
                self.assertTrue(wrapper.is_file())
                self.assertIn(board["cpu"], wrapper.read_text(encoding="utf-8"))
                ini_exists = (
                    ROOT / "firmware/tunerstudio/generated" / board["ini_file"]
                ).is_file()
                if board["id"] in EXPECTED_MISSING_INI:
                    self.assertFalse(ini_exists)
                else:
                    self.assertTrue(ini_exists)

    def test_machine_readable_board_support_gaps(self):
        boards = {board["id"]: board for board in self.document["boards"]}
        self.assertEqual(["lua"], boards["hellen128"]["unsupported_modules"])
        self.assertNotIn(
            "lua", boards["hellen128"]["supported_optional_modules"]
        )
        for board_id, board in boards.items():
            with self.subTest(board=board_id):
                self.assertEqual(
                    board["supported_optional_modules"],
                    [
                        module
                        for module in self.optional
                        if module not in board["unsupported_modules"]
                        and not (
                            module == "shock_preload"
                            and not board["capabilities"].get("can2")
                        )
                    ],
                )
                if board_id in CAN2_BOARDS:
                    self.assertIn(
                        "shock_preload",
                        board["supported_optional_modules"],
                    )
                else:
                    self.assertNotIn(
                        "shock_preload",
                        board["supported_optional_modules"],
                    )

    def test_board_support_is_queried_for_every_mapping(self):
        with mock.patch.object(
            catalog, "_query_board_unsupported_modules", return_value=()
        ) as query:
            document = catalog.catalog_document(ROOT)
        self.assertEqual(query.call_count, len(BOARD_IDS))
        boards = {board["id"]: board for board in document["boards"]}
        self.assertEqual([], boards["hellen128"]["unsupported_modules"])

    def test_catalog_source_parity(self):
        source = (
            ROOT / "firmware/controllers/modules/module_profile.mk"
        ).read_text(encoding="utf-8")
        match = re.search(
            r"(?m)^FOME_MODULE_CATALOG\s*:=\s*([^#\r\n]+)", source
        )
        self.assertIsNotNone(match)
        source_modules = match.group(1).split()
        emitted = self.document["base_modules"] + self.document["optional_modules"]
        self.assertEqual(emitted, source_modules)
        self.assertEqual(self.optional, source_modules[len(catalog.BASE_MODULES) :])

    def test_feature_rules_and_supported_values(self):
        self.assertEqual(
            self.document["feature_dependencies"]["emp_pump"], ["fan"]
        )
        self.assertEqual(
            self.document["feature_constraints"]["shock_preload"]["capabilities"],
            {"can2": True},
        )
        self.assertEqual(
            self.document["parameters"]["ve_rpm_count"]["values"], [16, 32]
        )
        self.assertEqual(
            self.document["parameters"]["airmass_blend_size"]["values"], [8, 16]
        )
        self.assertEqual(
            self.document["parameters"]["cylinder_trim_count"]["values"],
            list(range(1, 13)),
        )
        presets = self.document["parameters"]["engine_presets"]
        self.assertEqual("all", presets["default"])
        self.assertEqual("none", presets["default_when_expanded"])
        self.assertEqual(
            ["ve_rpm_count", "airmass_blend_size"],
            presets["expanded_when_any"],
        )
        self.assertEqual(["all", "none"], presets["exclusive_keywords"])
        self.assertFalse(presets["mixed_selection_allowed"])
        self.assertFalse(presets["explicit_selection_allowed_when_expanded"])

    def test_feature_csv_validation_and_canonical_order(self):
        self.assertEqual(catalog.parse_features("", self.optional), [])
        self.assertEqual(
            catalog.parse_features(" boost, lua ", self.optional),
            ["boost", "lua"],
        )
        invalid = {
            ",": "empty token",
            "lua,": "empty token",
            "lua,,boost": "empty token",
            "lua,lua": "duplicate",
            "fan": "mandatory",
            "not_real": "unknown",
            "Lua": "invalid",
            "lua\nboost": "single line",
            "lua boost": "invalid",
        }
        for value, message in invalid.items():
            with self.subTest(value=value):
                with self.assertRaisesRegex(catalog.CatalogError, message):
                    catalog.parse_features(value, self.optional)
        with self.assertRaisesRegex(catalog.CatalogError, "512"):
            catalog.parse_features("x" * 513, self.optional)

    def test_request_id_validation(self):
        for valid in ("request-1", "ABC_def", "123"):
            self.assertEqual(catalog.validate_request_id(valid), valid)
        for invalid in ("", "bad/request", "two words", "ümlaut", "x\ny"):
            with self.subTest(value=invalid):
                with self.assertRaises(catalog.CatalogError):
                    catalog.validate_request_id(invalid)

    def test_invalid_numeric_options_are_rejected(self):
        for kwargs in (
            {"ve_rpm_count": 24},
            {"ve_rpm_count": True},
            {"ve_rpm_count": 16.0},
            {"airmass_blend_size": 12},
            {"airmass_blend_size": True},
            {"cylinder_trim_count": 0},
            {"cylinder_trim_count": True},
            {"cylinder_trim_count": 6.0},
            {"cylinder_trim_count": 13},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(catalog.CatalogError):
                    catalog.resolve(ROOT, "core8", "", **kwargs)

    def test_shock_preload_requires_declared_can2(self):
        boards = {board["id"]: board for board in self.document["boards"]}
        enabled = {
            board_id for board_id, board in boards.items()
            if board["capabilities"].get("can2")
        }
        self.assertEqual(enabled, CAN2_BOARDS)
        with self.assertRaisesRegex(catalog.CatalogError, "requires CAN2"):
            catalog.resolve(ROOT, "hellen72", "shock_preload")
        catalog._enforce_feature_rules(
            boards["atlas"],
            ["shock_preload"],
        )

    def test_malformed_generated_profile_is_rejected(self):
        valid = """\
modules fan fuel_pump
cylinder_fuel_trim_count 0
ve_rpm_count 16
airmass_blend_size 8
engine_preset_mode default_none
engine_presets MINIMAL_PINS
engine_preset_default MINIMAL_PINS
board /tmp/board
cpu ARCH_STM32F4
config /tmp/config.txt
"""
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "profile.txt"
            profile.write_text(valid, encoding="utf-8")
            parsed, board, cpu = catalog._parse_profile(profile)
            self.assertEqual(parsed["modules"], ["fan", "fuel_pump"])
            self.assertEqual(parsed["cylinder_trim_count"], 0)
            self.assertEqual(board, "/tmp/board")
            self.assertEqual(cpu, "ARCH_STM32F4")
            profile.write_text(valid + "ve_rpm_count 32\n", encoding="utf-8")
            with self.assertRaisesRegex(catalog.CatalogError, "duplicate"):
                catalog._parse_profile(profile)


@unittest.skipUnless(shutil.which("make"), "host make is required")
class CanonicalMakeTests(unittest.TestCase):
    def test_lua_unavailable_on_hellen128_comes_from_make(self):
        with self.assertRaises(catalog.CatalogError) as caught:
            catalog.resolve(ROOT, "hellen128", "lua")
        self.assertIn("does not support", str(caught.exception))
        self.assertIn("lua", str(caught.exception))

    def test_expanded_dimensions_reject_default_all_presets(self):
        with self.assertRaises(catalog.CatalogError) as caught:
            catalog.resolve(
                ROOT,
                "core8",
                "",
                ve_rpm_count=32,
                engine_presets="all",
            )
        self.assertIn("cannot include literal engine presets", str(caught.exception))

    def test_make_rejects_mixed_all_and_none_presets(self):
        with self.assertRaisesRegex(catalog.CatalogError, "cannot mix 'all'"):
            catalog.resolve(
                ROOT, "core8", "", engine_presets="all none"
            )

    def test_dimensions_and_cylinder_trim_propagate(self):
        resolved = catalog.resolve(
            ROOT,
            "core8",
            "cylinder_fuel_trim",
            ve_rpm_count=32,
            airmass_blend_size=16,
            cylinder_trim_count=9,
            engine_presets="none",
        )
        profile = resolved["profile"]
        self.assertEqual(profile["ve_rpm_count"], 32)
        self.assertEqual(profile["airmass_blend_size"], 16)
        self.assertEqual(profile["cylinder_trim_count"], 9)
        self.assertEqual(profile["engine_preset_mode"], "explicit_none")
        self.assertEqual(profile["engine_presets"], ["MINIMAL_PINS"])
        self.assertIn("cylinder_fuel_trim", profile["modules"])
        self.assertEqual(resolved["optional_modules"], ["cylinder_fuel_trim"])

    def test_disabled_cylinder_trim_resolves_to_zero(self):
        resolved = catalog.resolve(
            ROOT, "core8", "", cylinder_trim_count=9, engine_presets="none"
        )
        self.assertEqual(resolved["profile"]["cylinder_trim_count"], 0)
        self.assertNotIn("cylinder_fuel_trim", resolved["profile"]["modules"])

    def test_reordered_features_have_the_same_profile(self):
        first = catalog.resolve(
            ROOT, "core8", "cylinder_fuel_trim,boost,lua"
        )
        second = catalog.resolve(
            ROOT, "core8", "lua, cylinder_fuel_trim, boost"
        )
        self.assertEqual(first, second)
        self.assertEqual(
            first["optional_modules"], ["boost", "lua", "cylinder_fuel_trim"]
        )
        self.assertEqual(first["profile"]["engine_preset_mode"], "default_all")

    @unittest.skipUnless((ROOT / ".git").exists(), "git metadata is unavailable")
    def test_cli_outputs_stay_outside_the_checkout(self):
        def status():
            result = subprocess.run(
                ["git", "status", "--porcelain", "-uall"],
                cwd=ROOT,
                check=True,
                capture_output=True,
                text=True,
            )
            return result.stdout

        before = status()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "resolved.json"
            github_env = Path(temporary) / "github.env"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = catalog.main(
                    [
                        "resolve",
                        "--root",
                        str(ROOT),
                        "--board",
                        "core8",
                        "--features",
                        "boost",
                        "--request-id",
                        "request-42",
                        "--output",
                        str(output),
                        "--github-env",
                        str(github_env),
                    ]
                )
            self.assertEqual(code, 0)
            self.assertEqual(stdout.getvalue(), "")
            resolved = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(resolved["schema_version"], 1)
            values = {}
            for line in github_env.read_text(encoding="utf-8").splitlines():
                key, value = line.split("=", 1)
                self.assertNotIn(key, values)
                values[key] = value
            self.assertEqual(values["REQUEST_ID"], "request-42")
            self.assertEqual(values["BOARD_ID"], "core8")
            self.assertEqual(values["REQUESTED_FEATURES"], "boost")
            self.assertEqual(values["FOME_ENGINE_PRESETS"], "all")
            self.assertEqual(values["FOME_VE_RPM_COUNT"], "16")
            self.assertEqual(values["FOME_AIRMASS_BLEND_SIZE"], "8")
            self.assertEqual(values["FOME_CYLINDER_TRIM_COUNT"], "0")
        self.assertEqual(status(), before)


if __name__ == "__main__":
    unittest.main()
