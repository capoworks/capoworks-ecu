"""Standard-library tests for resource calibration planning and collection."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

try:
    from . import calibration, resources
except ImportError:  # Allow direct execution from this directory.
    import calibration  # type: ignore[no-redef]
    import resources  # type: ignore[no-redef]


REPO = Path(__file__).resolve().parents[2]
BOARD_DATA = json.loads(
    (REPO / "misc/firmware_builder/boards.json").read_text(encoding="utf-8")
)["boards"]
BASE_MODULES = [
    "fan",
    "fuel_pump",
    "gear_detector",
    "tachometer",
    "trip_odometer",
    "map_averaging",
]
OPTIONAL_MODULES = [
    "hella_opst",
    "boost",
    "engine_sniffer",
    "lua",
    "emp_pump",
    "shock_preload",
    "cylinder_fuel_trim",
    "sd_card",
]
SOURCE_COMMIT = "3060ccd5873a89e6499c9ade5756338ab7c529fd"


def _catalog_document() -> dict:
    boards = []
    for board in BOARD_DATA:
        supported = []
        for module in OPTIONAL_MODULES:
            if module == "lua" and board["id"] == "hellen128":
                continue
            if module == "shock_preload" and not board["capabilities"]["can2"]:
                continue
            supported.append(module)
        boards.append(
            {
                **board,
                "unsupported_modules": [
                    module
                    for module in OPTIONAL_MODULES
                    if module not in supported
                ],
                "supported_optional_modules": supported,
                "required_modules": (
                    ["sd_card"] if board["id"] == "atlas" else []
                ),
            }
        )
    return {
        "schema_version": 1,
        "boards": boards,
        "base_modules": BASE_MODULES,
        "optional_modules": OPTIONAL_MODULES,
        "feature_dependencies": {"emp_pump": ["fan"]},
        "feature_constraints": {
            "shock_preload": {
                "capabilities": {"can2": True},
                "message": "shock_preload requires CAN2",
            }
        },
        "parameters": {},
    }


def _resolved(
    root: Path,
    board_id: str,
    features: str,
    *,
    ve_rpm_count: int,
    airmass_blend_size: int,
    cylinder_trim_count: int | None,
    engine_presets: str | None,
    sd_card: bool,
) -> dict:
    selected = features.split(",") if features else []
    board = next(item for item in BOARD_DATA if item["id"] == board_id)
    effective_optional = [
        module
        for module in OPTIONAL_MODULES
        if module in selected or (module == "sd_card" and sd_card)
    ]
    preset_mode = "default_all" if engine_presets is None else "explicit_none"
    presets = (
        ["MINIMAL_PINS", "PROTEUS_BMW_M73"]
        if engine_presets is None
        else ["MINIMAL_PINS"]
    )
    return {
        "schema_version": 1,
        "board_id": board_id,
        "board_dir": board["board_dir"],
        "compile_script": board["compile_script"],
        "ini_file": board["ini_file"],
        "cpu": board["cpu"],
        "base_modules": BASE_MODULES,
        "required_modules": (
            ["sd_card"] if board_id == "atlas" else []
        ),
        "optional_modules": effective_optional,
        "profile": {
            "modules": BASE_MODULES + effective_optional,
            "ve_rpm_count": ve_rpm_count,
            "airmass_blend_size": airmass_blend_size,
            "cylinder_trim_count": cylinder_trim_count or 0,
            "engine_preset_mode": preset_mode,
            "engine_presets": presets,
            "engine_preset_default": "MINIMAL_PINS",
        },
    }


class CalibrationPlanTests(unittest.TestCase):
    def setUp(self) -> None:
        self.document = _catalog_document()
        self.catalog_patch = mock.patch.object(
            calibration.catalog,
            "catalog_document",
            return_value=self.document,
        )
        self.catalog_patch.start()
        self.addCleanup(self.catalog_patch.stop)
        self.resolve_patch = mock.patch.object(
            calibration.catalog,
            "resolve",
            side_effect=_resolved,
        )
        self.resolve = self.resolve_patch.start()
        self.addCleanup(self.resolve_patch.stop)

    def test_every_catalog_board_has_a_stable_baseline_mapping(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="all",
            coverage="baseline",
        )

        records = matrix["include"]
        self.assertEqual(
            [board["id"] for board in self.document["boards"]],
            [record["board_id"] for record in records],
        )
        self.assertEqual(14, len(records))
        sd_by_board = {
            record["board_id"]: record["sd_card"] for record in records
        }
        self.assertFalse(sd_by_board["core8"])
        self.assertTrue(sd_by_board["atlas"])
        for record in records:
            self.assertEqual("", record["features"])
            self.assertEqual(16, record["ve_rpm_count"])
            self.assertEqual(8, record["airmass_blend_size"])
            self.assertEqual(0, record["cylinder_trim_count"])
            self.assertEqual("all", record["engine_presets"])
            self.assertRegex(record["profile_id"], calibration.PROFILE_ID_RE)
        self.assertEqual(
            {board["id"] for board in self.document["boards"]},
            {call.args[1] for call in self.resolve.call_args_list},
        )

    def test_singles_omit_capability_blocked_modules(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="hellen128",
            coverage="singles",
        )

        features = {
            module
            for record in matrix["include"]
            for module in record["features"].split(",")
            if module
        }
        self.assertNotIn("lua", features)
        self.assertNotIn("shock_preload", features)
        self.assertIn("cylinder_fuel_trim", features)
        self.assertEqual(
            1 + len(
                next(
                    board
                    for board in self.document["boards"]
                    if board["id"] == "hellen128"
                )["supported_optional_modules"]
            ),
            len(matrix["include"]),
        )

    def test_baseline_profile_ids_are_unique_in_all_coverage_plans(self) -> None:
        for coverage in ("baseline", "singles", "full"):
            with self.subTest(coverage=coverage):
                matrix = calibration.plan(
                    root="/source",
                    board="core8",
                    coverage=coverage,
                )
                baselines = [
                    record
                    for record in matrix["include"]
                    if record["profile_id"] == "core8-baseline"
                ]
                self.assertEqual(1, len(baselines))
                self.assertEqual("core8-baseline", baselines[0]["profile_id"])
                profile_ids = [record["profile_id"] for record in matrix["include"]]
                self.assertEqual(len(profile_ids), len(set(profile_ids)))

    def test_supported_single_profiles_propagate_effective_options(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="core8",
            coverage="singles",
        )

        records = {
            record["features"]: record
            for record in matrix["include"]
        }
        trim = records["cylinder_fuel_trim"]
        self.assertEqual(6, trim["cylinder_trim_count"])
        self.assertEqual("all", trim["engine_presets"])
        self.assertEqual("core8-single-cylinder-fuel-trim", trim["profile_id"])
        trim_call = next(
            call
            for call in self.resolve.call_args_list
            if call.args[2] == "cylinder_fuel_trim"
        )
        self.assertEqual(6, trim_call.kwargs["cylinder_trim_count"])
        self.assertIsNone(trim_call.kwargs["engine_presets"])
        self.assertFalse(trim_call.kwargs["sd_card"])

        shock_call = next(
            call
            for call in self.resolve.call_args_list
            if call.args[2] == "shock_preload"
        )
        self.assertEqual(16, shock_call.kwargs["ve_rpm_count"])
        self.assertEqual(8, shock_call.kwargs["airmass_blend_size"])
        self.assertIsNone(shock_call.kwargs["cylinder_trim_count"])
        self.assertIsNone(shock_call.kwargs["engine_presets"])
        self.assertFalse(shock_call.kwargs["sd_card"])

    def test_full_selects_all_supported_modules_and_trim_six(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="core8",
            coverage="full",
        )

        baseline, full = matrix["include"]
        self.assertEqual("", baseline["features"])
        self.assertEqual(",".join(OPTIONAL_MODULES[:-1]), full["features"])
        self.assertFalse(baseline["sd_card"])
        self.assertTrue(full["sd_card"])
        self.assertEqual(6, full["cylinder_trim_count"])
        self.assertEqual("all", full["engine_presets"])

    def test_expanded_covers_all_dimensions_without_presets(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="proteus_h7",
            coverage="expanded",
        )

        base = [
            (
                record["ve_rpm_count"],
                record["airmass_blend_size"],
                record["engine_presets"],
                record["cylinder_trim_count"],
            )
            for record in matrix["include"][:4]
        ]
        self.assertEqual(
            [
                (16, 8, "none", 0),
                (32, 8, "none", 0),
                (16, 16, "none", 0),
                (32, 16, "none", 0),
            ],
            base,
        )
        self.assertEqual(5, len(matrix["include"]))
        expanded_full = matrix["include"][-1]
        self.assertEqual((32, 16, "none", 6), (
            expanded_full["ve_rpm_count"],
            expanded_full["airmass_blend_size"],
            expanded_full["engine_presets"],
            expanded_full["cylinder_trim_count"],
        ))

    def test_profile_counts_follow_catalog_support(self) -> None:
        expected = {
            "baseline": len(self.document["boards"]),
            "singles": sum(
                1
                + len(board["supported_optional_modules"])
                - len(board["required_modules"])
                for board in self.document["boards"]
            ),
            "full": 2 * len(self.document["boards"]),
            "expanded": 5 * len(self.document["boards"]),
        }
        for coverage, count in expected.items():
            with self.subTest(coverage=coverage):
                matrix = calibration.plan(
                    root="/source",
                    board="all",
                    coverage=coverage,
                )
                self.assertEqual(count, len(matrix["include"]))
                ids = [record["profile_id"] for record in matrix["include"]]
                self.assertEqual(len(ids), len(set(ids)))

    def test_unknown_board_fails_before_resolution(self) -> None:
        with self.assertRaisesRegex(
            calibration.CalibrationError, "unknown board"
        ):
            calibration.plan(
                root="/source",
                board="not-a-board",
                coverage="baseline",
            )
        self.resolve.assert_not_called()

    def test_calibration_requests_sd_card_explicitly(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="core8",
            coverage="singles",
        )
        by_profile_id = {
            record["profile_id"]: record for record in matrix["include"]
        }
        baseline = by_profile_id["core8-baseline"]
        self.assertEqual("", baseline["features"])
        self.assertFalse(baseline["sd_card"])

        sd_record = by_profile_id["core8-single-sd-card"]
        self.assertEqual("", sd_record["features"])
        self.assertTrue(sd_record["sd_card"])
        for record in matrix["include"]:
            if record["profile_id"] in {
                "core8-baseline",
                "core8-single-sd-card",
            }:
                continue
            with self.subTest(profile_id=record["profile_id"]):
                self.assertNotEqual("", record["features"])
                self.assertFalse(record["sd_card"])

        baseline_call = next(
            call
            for call in self.resolve.call_args_list
            if call.args[1] == "core8"
            and call.args[2] == ""
            and call.kwargs["sd_card"] is False
        )
        self.assertEqual("", baseline_call.args[2])
        boost_call = next(
            call
            for call in self.resolve.call_args_list
            if call.args[1] == "core8" and call.args[2] == "boost"
        )
        self.assertFalse(boost_call.kwargs["sd_card"])
        sd_call = next(
            call
            for call in self.resolve.call_args_list
            if call.args[1] == "core8" and call.kwargs["sd_card"]
        )
        self.assertEqual("", sd_call.args[2])

        full = calibration.plan(
            root="/source",
            board="core8",
            coverage="full",
        )["include"][1]
        self.assertTrue(full["sd_card"])

    def test_atlas_required_sd_card_is_in_mandatory_baseline(self) -> None:
        matrix = calibration.plan(
            root="/source",
            board="atlas",
            coverage="singles",
        )
        baseline = matrix["include"][0]
        self.assertEqual("atlas-baseline", baseline["profile_id"])
        self.assertEqual("", baseline["features"])
        self.assertTrue(baseline["sd_card"])
        self.assertNotIn(
            "atlas-single-sd-card",
            [record["profile_id"] for record in matrix["include"]],
        )
        atlas_calls = [
            call
            for call in self.resolve.call_args_list
            if call.args[1] == "atlas"
        ]
        self.assertTrue(all(call.kwargs["sd_card"] for call in atlas_calls))


class CalibrationCollectTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / "source"
        self.output = self.base / "observation"
        self.elf = self.root / "firmware/build/fome.elf"
        self.profile = self.root / "firmware/generated/module_profile.txt"
        for path in (self.elf, self.profile):
            path.parent.mkdir(parents=True, exist_ok=True)
        self.elf.write_bytes(b"ELF fixture\n")
        self.profile.write_text(
            "\n".join(
                [
                    "modules " + " ".join(BASE_MODULES),
                    "cylinder_fuel_trim_count 0",
                    "ve_rpm_count 16",
                    "airmass_blend_size 8",
                    "engine_preset_mode default_all",
                    "engine_presets MINIMAL_PINS PROTEUS_BMW_M73",
                    "engine_preset_default MINIMAL_PINS",
                    (
                        "board "
                        + str(
                            (
                                self.root
                                / "firmware/config/boards/core8"
                            ).resolve()
                        )
                    ),
                    "cpu ARCH_STM32F4",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        policy = self.root / "misc/firmware_builder/policies"
        policy.mkdir(parents=True)
        (policy / "default.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "required_free_bytes": {},
                    "minimum_heap_capacity_bytes": 0,
                }
            ),
            encoding="utf-8",
        )
        self.document = _catalog_document()
        self.catalog_patch = mock.patch.object(
            calibration.catalog,
            "catalog_document",
            return_value=self.document,
        )
        self.catalog_patch.start()
        self.addCleanup(self.catalog_patch.stop)
        self.git_patch = mock.patch.object(
            calibration.pipeline,
            "_git_head",
            return_value=SOURCE_COMMIT,
        )
        self.git_patch.start()
        self.addCleanup(self.git_patch.stop)
        self.tools_patch = mock.patch.object(
            calibration.resources,
            "resolve_tools",
            return_value={
                "nm": Path("/vendor/nm"),
                "objdump": Path("/vendor/objdump"),
                "compiler": Path("/vendor/compiler"),
            },
        )
        self.tools_patch.start()
        self.addCleanup(self.tools_patch.stop)

    def _write_profile(
        self,
        *,
        board_dir: str = "firmware/config/boards/core8",
        cpu: str = "ARCH_STM32F4",
        ve_rpm_count: int = 16,
        airmass_blend_size: int = 8,
    ) -> None:
        self.profile.write_text(
            "\n".join(
                [
                    "modules " + " ".join(BASE_MODULES),
                    "cylinder_fuel_trim_count 0",
                    f"ve_rpm_count {ve_rpm_count}",
                    f"airmass_blend_size {airmass_blend_size}",
                    "engine_preset_mode default_all",
                    "engine_presets MINIMAL_PINS PROTEUS_BMW_M73",
                    "engine_preset_default MINIMAL_PINS",
                    "board " + str((self.root / board_dir).resolve()),
                    f"cpu {cpu}",
                ]
            )
            + "\n",
            encoding="utf-8",
        )

    def test_collect_uses_current_build_and_writes_artifact_folder(self) -> None:
        index = {
            "schema_version": 1,
            "observations": [{"observation_id": "evidence"}],
            "diagnostics": [],
        }
        report = {
            "schema_version": 1,
            "board_id": "core8",
            "cpu": "ARCH_STM32F4",
            "status": "accepted",
        }
        with mock.patch.object(
            calibration.resources,
            "extract_report",
            return_value=report,
        ) as extract, mock.patch.object(
            calibration.matrix,
            "merge_reports",
            return_value=index,
        ) as merge:
            collected = calibration.collect(
                root=self.root,
                board="core8",
                output=self.output,
            )

        self.assertEqual(report, collected)
        extract.assert_called_once_with(
            elf=self.elf,
            profile_path=self.profile,
            board_id="core8",
            cpu="ARCH_STM32F4",
            source_commit=SOURCE_COMMIT,
            tools={
                "nm": Path("/vendor/nm"),
                "objdump": Path("/vendor/objdump"),
                "compiler": Path("/vendor/compiler"),
            },
            policy_path=(
                self.root / "misc/firmware_builder/policies/default.json"
            ),
        )
        merge.assert_called_once_with([report])
        self.assertEqual(
            {"resources.json", "resource-index.json", "catalog.json"},
            {path.name for path in self.output.iterdir()},
        )
        self.assertFalse((self.output / "fome.elf").exists())
        self.assertEqual(
            report,
            json.loads(
                (self.output / "resources.json").read_text(encoding="utf-8")
            ),
        )

    def test_board_policy_overrides_default(self) -> None:
        self._write_profile(board_dir="firmware/config/boards/proteus")
        board_policy = (
            self.root
            / "misc/firmware_builder/policies/proteus_f4.json"
        )
        board_policy.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "required_free_bytes": {"flash": 1024},
                    "minimum_heap_capacity_bytes": 0,
                }
            ),
            encoding="utf-8",
        )
        with mock.patch.object(
            calibration.resources,
            "extract_report",
            return_value={
                "schema_version": 1,
                "board_id": "proteus_f4",
                "cpu": "ARCH_STM32F4",
                "status": "accepted",
            },
        ) as extract, mock.patch.object(
            calibration.matrix,
            "merge_reports",
            return_value={
                "schema_version": 1,
                "observations": [],
                "diagnostics": [],
            },
        ):
            calibration.collect(
                root=self.root,
                board="proteus_f4",
                output=self.output,
            )

        self.assertEqual(
            board_policy,
            extract.call_args.kwargs["policy_path"],
        )

    def test_collect_rejects_unknown_board_before_extraction(self) -> None:
        with mock.patch.object(
            calibration.resources, "extract_report"
        ) as extract:
            with self.assertRaisesRegex(
                calibration.CalibrationError, "unknown board"
            ):
                calibration.collect(
                    root=self.root,
                    board="not-a-board",
                    output=self.output,
                )

            extract.assert_not_called()

    def test_collect_rejects_same_cpu_from_a_different_board(self) -> None:
        self._write_profile(board_dir="firmware/config/boards/proteus")

        with mock.patch.object(
            calibration.resources, "extract_report"
        ) as extract:
            with self.assertRaisesRegex(
                calibration.CalibrationError, "board .* does not match"
            ):
                calibration.collect(
                    root=self.root,
                    board="core8",
                    output=self.output,
                )

        extract.assert_not_called()

    def test_collect_rejects_a_generated_profile_with_a_different_cpu(self) -> None:
        self._write_profile(cpu="ARCH_STM32H7")

        with mock.patch.object(
            calibration.resources, "extract_report"
        ) as extract:
            with self.assertRaisesRegex(
                calibration.CalibrationError, "CPU .* does not match"
            ):
                calibration.collect(
                    root=self.root,
                    board="core8",
                    output=self.output,
                )

        extract.assert_not_called()

    def test_collect_rejects_missing_board_required_modules(self) -> None:
        self._write_profile(board_dir="firmware/config/boards/atlas")

        with mock.patch.object(
            calibration.resources, "extract_report"
        ) as extract:
            with self.assertRaisesRegex(
                calibration.CalibrationError, "missing required modules.*sd_card"
            ):
                calibration.collect(
                    root=self.root,
                    board="atlas",
                    output=self.output,
                )

        extract.assert_not_called()

    def test_collect_rejects_resource_profile_mismatch(self) -> None:
        profile = resources.parse_profile(
            self.profile.read_text(encoding="utf-8")
        )
        profile["ve_rpm_count"] = 32
        report = {
            "schema_version": 1,
            "board_id": "core8",
            "cpu": "ARCH_STM32F4",
            "profile": profile,
            "status": "accepted",
        }
        with mock.patch.object(
            calibration.resources,
            "extract_report",
            return_value=report,
        ):
            with self.assertRaisesRegex(
                calibration.CalibrationError, "profile does not match"
            ):
                calibration.collect(
                    root=self.root,
                    board="core8",
                    output=self.output,
                )

    def test_extraction_failure_writes_diagnostic_not_verified_evidence(self) -> None:
        with mock.patch.object(
            calibration.resources,
            "extract_report",
            side_effect=resources.ExtractionError("nm rejected the ELF"),
        ):
            report = calibration.collect(
                root=self.root,
                board="core8",
                output=self.output,
            )

        self.assertEqual("rejected", report["status"])
        self.assertEqual(
            "extraction_failed", report["violations"][0]["code"]
        )
        on_disk = json.loads(
            (self.output / "resources.json").read_text(encoding="utf-8")
        )
        self.assertEqual("rejected", on_disk["status"])
        index = json.loads(
            (self.output / "resource-index.json").read_text(encoding="utf-8")
        )
        self.assertEqual([], index["observations"])
        self.assertEqual(1, len(index["diagnostics"]))
        self.assertEqual(False, index["diagnostics"][0]["complete"])


if __name__ == "__main__":
    unittest.main()
