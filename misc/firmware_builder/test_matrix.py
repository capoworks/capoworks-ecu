"""Standard-library tests for the resource matrix and preflight estimator."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location("firmware_matrix", HERE / "matrix.py")
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


COMPILER_SHA = "a" * 64
ELF_SHA = "b" * 64
COMMIT = "3060ccd5873a89e6499c9ade5756338ab7c529fd"
DEFAULT_POLICY_REQUIRED = {
    "flash": 10,
    "ram0": 10,
    "ram1": 10,
    "persistent": 10,
}
DEFAULT_POLICY_ID = matrix._policy_content_id(DEFAULT_POLICY_REQUIRED, 100)
ALL_RAM_REGIONS = tuple(f"ram{index}" for index in range(8))


def make_profile(
    features=(),
    *,
    trim_count=0,
    ve_rpm_count=16,
    airmass_blend_size=8,
    preset_mode="explicit_none",
    presets=("MINIMAL_PINS",),
    preset_default="MINIMAL_PINS",
):
    modules = list(matrix.BASE_MODULES)
    for feature in features:
        if feature not in modules:
            modules.append(feature)
    if "cylinder_fuel_trim" in modules and trim_count == 0:
        trim_count = 6
    if "cylinder_fuel_trim" not in modules:
        trim_count = 0
    return {
        "modules": modules,
        "ve_rpm_count": ve_rpm_count,
        "airmass_blend_size": airmass_blend_size,
        "cylinder_trim_count": trim_count,
        "engine_preset_mode": preset_mode,
        "engine_presets": list(presets),
        "engine_preset_default": preset_default,
    }


def make_policy(
    *,
    policy_id=None,
    required=None,
    omitted=(),
    minimum_heap=100,
):
    requirements = dict(DEFAULT_POLICY_REQUIRED)
    if required is not None:
        requirements.update(required)
    for region in omitted:
        requirements.pop(region, None)
    if policy_id is None:
        policy_id = matrix._policy_content_id(requirements, minimum_heap)
    return {
        "id": policy_id,
        "schema_version": 1,
        "required_free_bytes": requirements,
        "minimum_heap_capacity_bytes": minimum_heap,
    }


def make_report(
    features=(),
    *,
    board_id="core8",
    cpu="ARCH_STM32F4",
    source_commit=COMMIT,
    toolchain_id="arm-none-eabi-g++",
    policy=None,
    profile=None,
    feature_usage=None,
    trim_count=0,
    ve_rpm_count=16,
    airmass_blend_size=8,
    preset_mode="explicit_none",
    presets=("MINIMAL_PINS",),
    preset_default="MINIMAL_PINS",
    heap_capacity=2048,
    force_status=None,
    elf_sha256=ELF_SHA,
):
    active_policy = policy or make_policy()
    active_profile = profile or make_profile(
        features,
        trim_count=trim_count,
        ve_rpm_count=ve_rpm_count,
        airmass_blend_size=airmass_blend_size,
        preset_mode=preset_mode,
        presets=presets,
        preset_default=preset_default,
    )
    usage = {
        "flash": 1000,
        "ram0": 100,
        "ram1": 200,
        "persistent": 64,
        "heap": 0,
    }
    for values in (feature_usage or {}).values():
        for key, amount in values.items():
            usage[key] += amount

    capacities = {
        "flash": 10000,
        "ram0": 4096,
        "ram1": 8192,
        "ram2": 0,
        "ram3": 0,
        "ram4": 0,
        "ram5": 0,
        "ram6": 0,
        "ram7": 0,
    }
    requirements = active_policy["required_free_bytes"]
    regions = {}
    for name, capacity in capacities.items():
        used = usage.get(name, 0)
        free = capacity - used
        required = requirements.get(name, 0)
        regions[name] = {
            "origin": 0,
            "limit": capacity,
            "capacity_bytes": capacity,
            "used_bytes": used,
            "free_bytes": free,
            "required_free_bytes": required,
            "status": "accepted" if free >= required else "rejected",
        }

    persistent_used = usage["persistent"]
    persistent_capacity = 512
    persistent_free = persistent_capacity - persistent_used
    persistent_required = requirements.get("persistent", 0)
    heap_required = active_policy["minimum_heap_capacity_bytes"]
    region_statuses = [region["status"] for region in regions.values()]
    persistent_status = (
        "accepted" if persistent_free >= persistent_required else "rejected"
    )
    heap_status = (
        "accepted" if heap_capacity >= heap_required else "rejected"
    )
    statuses = region_statuses + [persistent_status, heap_status]
    derived_status = (
        "accepted" if all(status == "accepted" for status in statuses) else "rejected"
    )
    violations = []
    if derived_status == "rejected":
        violations.append(
            {
                "code": "policy_free_below_minimum",
                "message": "a complete resource slot is below its minimum",
            }
        )
    return {
        "schema_version": 1,
        "board_id": board_id,
        "cpu": cpu,
        "source_commit": source_commit,
        "toolchain": {
            "id": toolchain_id,
            "version": "compiler version 1",
            "sha256": COMPILER_SHA,
        },
        "profile": active_profile,
        "elf_sha256": elf_sha256,
        "regions": regions,
        "heap": {
            "origin": 0,
            "limit": heap_capacity,
            "capacity_bytes": heap_capacity,
            "required_capacity_bytes": heap_required,
            "status": heap_status,
        },
        "persistent": {
            "ram_region": "ram1",
            "used_bytes": persistent_used,
            "capacity_bytes": persistent_capacity,
            "free_bytes": persistent_free,
            "required_free_bytes": persistent_required,
            "status": persistent_status,
        },
        "policy": active_policy,
        "status": force_status or derived_status,
        "violations": violations,
        "hardware_guarantee": False,
    }


def make_request(
    profile,
    *,
    board_id="core8",
    cpu="ARCH_STM32F4",
    required_modules=(),
):
    return {
        "schema_version": 1,
        "board_id": board_id,
        "cpu": cpu,
        "required_modules": list(required_modules),
        "profile": profile,
    }


def make_index(*reports):
    return matrix.merge_reports(reports)


class IndexTests(unittest.TestCase):
    def test_exact_measurement_is_verified(self):
        report = make_report()
        result = matrix.evaluate(
            make_index(report),
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("verified", result["status"])
        self.assertEqual("exact_measurement_verified", result["reason"])
        self.assertEqual(1, result["measurement"]["sample_count"])
        self.assertNotIn("estimate", result)

    def test_semantic_preset_modes_and_reordered_lists_match(self):
        report = make_report(
            features=("lua", "boost"),
            preset_mode="explicit_all",
            presets=("PROTEUS_BMW_M73", "MINIMAL_PINS"),
        )
        request_profile = make_profile(
            features=("boost", "lua"),
            preset_mode="default_all",
            presets=("MINIMAL_PINS", "PROTEUS_BMW_M73"),
        )
        index = make_index(report)
        self.assertEqual(
            matrix.profile_key(report["profile"]),
            matrix.profile_key(request_profile),
        )
        result = matrix.evaluate(
            index,
            make_request(request_profile),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("verified", result["status"])
        self.assertEqual("default_all", result["profile"]["engine_preset_mode"])
        self.assertEqual(
            report["profile"]["engine_preset_mode"],
            index["observations"][0]["profile"]["engine_preset_mode"],
        )

    def test_index_output_is_deterministic_and_keeps_duplicate_evidence(self):
        first = make_report()
        changed = make_report()
        changed["regions"]["ram0"]["used_bytes"] += 1
        changed["regions"]["ram0"]["free_bytes"] -= 1
        index = make_index(first, changed)
        once = json.dumps(index, sort_keys=True)
        twice = json.dumps(matrix.merge_reports([changed, first]), sort_keys=True)
        self.assertEqual(once, twice)
        self.assertEqual(2, len(index["observations"]))

    def test_missing_identity_is_rejected_and_not_cached(self):
        report = make_report()
        del report["policy"]["id"]
        with self.assertRaisesRegex(matrix.MatrixError, "policy.id"):
            matrix.merge_reports([report])

    def test_bool_is_not_accepted_as_an_integer(self):
        report = make_report()
        report["regions"]["flash"]["used_bytes"] = True
        with self.assertRaisesRegex(matrix.MatrixError, "integer"):
            matrix.merge_reports([report])

    def test_source_commit_requires_a_full_or_git_sha256_digest(self):
        report = make_report(source_commit=COMMIT[:7])
        with self.assertRaisesRegex(
            matrix.MatrixError, "40- or 64-character"
        ):
            matrix.merge_reports([report])

    def test_policy_id_cannot_be_reused_for_changed_thresholds(self):
        policy = make_policy(
            policy_id=DEFAULT_POLICY_ID,
            required={"ram0": 11},
        )
        with self.assertRaisesRegex(
            matrix.MatrixError, "policy.id must equal.*content hash"
        ):
            matrix.merge_reports([make_report(policy=policy)])

    def test_malformed_capacity_and_accounting_are_rejected(self):
        for mutate, message in (
            (
                lambda report: report["regions"]["flash"].update(
                    {"capacity_bytes": -1, "limit": -1}
                ),
                "at least 0",
            ),
            (
                lambda report: report["regions"]["ram0"].update(
                    {"free_bytes": 1}
                ),
                "capacity_bytes - used_bytes",
            ),
            (
                lambda report: report["regions"]["ram1"].update(
                    {"capacity_bytes": 1}
                ),
                "limit - origin",
            ),
        ):
            with self.subTest(message=message):
                report = make_report()
                mutate(report)
                with self.assertRaisesRegex(matrix.MatrixError, message):
                    matrix.merge_reports([report])

    def test_accepted_report_with_policy_overrun_is_rejected(self):
        policy = make_policy(required={"persistent": 500})
        report = make_report(policy=policy, force_status="accepted")
        report["persistent"]["status"] = "accepted"
        report["violations"] = []
        with self.assertRaisesRegex(
            matrix.MatrixError, "persistent storage.*policy minimum"
        ):
            matrix.merge_reports([report])

    def test_accepted_report_without_complete_metrics_is_rejected(self):
        report = make_report()
        report["heap"] = {}
        with self.assertRaisesRegex(
            matrix.MatrixError, "accepted report must contain complete metrics"
        ):
            matrix.merge_reports([report])

    def test_accepted_report_requires_every_region_even_when_policy_omits_bank(self):
        policy = make_policy(omitted=ALL_RAM_REGIONS)
        report = make_report(policy=policy)
        del report["regions"]["ram7"]
        with self.assertRaisesRegex(
            matrix.MatrixError, "accepted report must contain complete metrics"
        ):
            matrix.merge_reports([report])

    def test_unknown_region_cannot_replace_a_missing_bank(self):
        policy = make_policy(omitted=ALL_RAM_REGIONS)
        report = make_report(policy=policy)
        report["regions"]["ram_total"] = report["regions"].pop("ram7")
        with self.assertRaisesRegex(matrix.MatrixError, "unknown region name"):
            matrix.merge_reports([report])

        result = matrix.evaluate(
            {
                "schema_version": 1,
                "observations": [report],
                "diagnostics": [],
            },
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            policy["id"],
        )
        self.assertEqual("requires_build", result["status"])

    def test_unknown_schema_fails_closed(self):
        report = make_report()
        report["schema_version"] = 2
        with self.assertRaisesRegex(matrix.MatrixError, "schema_version"):
            matrix.merge_reports([report])


class EvaluationTests(unittest.TestCase):
    def test_stale_source_compiler_and_policy_do_not_reuse_measurements(self):
        index = make_index(make_report())
        request = make_request(make_profile())
        other_policy_id = make_policy(required={"flash": 11})["id"]
        for source, compiler, policy in (
            ("f" * 40, "arm-none-eabi-g++", DEFAULT_POLICY_ID),
            (COMMIT, "different-compiler", DEFAULT_POLICY_ID),
            (COMMIT, "arm-none-eabi-g++", other_policy_id),
        ):
            with self.subTest(source=source, compiler=compiler, policy=policy):
                result = matrix.evaluate(
                    index, request, source, compiler, policy
                )
                self.assertEqual("requires_build", result["status"])
                self.assertIn("estimate", result)
                self.assertFalse(result["estimate"]["known"])

    def test_board_and_cpu_context_collisions_do_not_reuse_measurements(self):
        index = make_index(make_report())
        for board_id, cpu in (
            ("hellen128", "ARCH_STM32F4"),
            ("core8", "ARCH_STM32F7"),
        ):
            with self.subTest(board_id=board_id, cpu=cpu):
                result = matrix.evaluate(
                    index,
                    make_request(make_profile(), board_id=board_id, cpu=cpu),
                    COMMIT,
                    "arm-none-eabi-g++",
                    DEFAULT_POLICY_ID,
                )
                self.assertEqual("requires_build", result["status"])
                self.assertFalse(result["estimate"]["known"])

    def test_complete_measured_persistent_overflow_is_incompatible(self):
        report = make_report(
            feature_usage={"overflow": {"persistent": 500}}
        )
        self.assertEqual("rejected", report["status"])
        self.assertLess(report["persistent"]["free_bytes"], 0)
        result = matrix.evaluate(
            make_index(report),
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("incompatible", result["status"])
        self.assertEqual(
            "exact_measurement_resource_failure", result["reason"]
        )
        self.assertEqual("rejected", result["measurement"]["status"])

    def test_estimated_policy_overrun_is_only_a_build_warning(self):
        baseline = make_report()
        boost = make_report(
            features=("boost",),
            feature_usage={"boost": {"persistent": 100}},
        )
        lua = make_report(
            features=("lua",),
            feature_usage={"lua": {"persistent": 100}},
        )
        policy = make_policy(required={"persistent": 250})
        baseline["policy"] = policy
        baseline["persistent"]["required_free_bytes"] = 250
        for report in (baseline, boost, lua):
            report["policy"] = copy.deepcopy(policy)
            report["persistent"]["required_free_bytes"] = 250
            report["persistent"]["free_bytes"] = (
                report["persistent"]["capacity_bytes"]
                - report["persistent"]["used_bytes"]
            )
            report["persistent"]["status"] = "accepted"
        result = matrix.evaluate(
            make_index(baseline, boost, lua),
            make_request(make_profile(("boost", "lua"))),
            COMMIT,
            "arm-none-eabi-g++",
            policy["id"],
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual("estimate_policy_overrun_warning", result["reason"])
        self.assertTrue(result["estimate"]["predicted"]["predicted_overbudget"])
        self.assertNotEqual("incompatible", result["status"])

    def test_missing_single_feature_coverage_returns_unknown_estimate(self):
        index = make_index(
            make_report(),
            make_report(features=("boost",)),
        )
        result = matrix.evaluate(
            index,
            make_request(make_profile(("boost", "lua"))),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertFalse(result["estimate"]["known"])
        self.assertEqual(
            ["lua"], result["estimate"]["unknown"]["missing_features"]
        )

    def test_trimless_and_trim_variants_are_covered_per_target_trim_count(self):
        baseline = make_report()
        lua = make_report(
            features=("lua",),
            feature_usage={
                "lua": {
                    "flash": 5,
                    "ram0": 10,
                    "ram1": 20,
                    "persistent": 30,
                },
            },
            heap_capacity=2050,
        )
        trim_six = make_report(
            features=("cylinder_fuel_trim",),
            trim_count=6,
            feature_usage={
                "cylinder_fuel_trim": {
                    "flash": 7,
                    "ram0": 11,
                    "ram1": 21,
                    "persistent": 50,
                },
            },
            heap_capacity=2054,
        )
        index = make_index(
            baseline,
            lua,
            trim_six,
        )
        result = matrix.evaluate(
            index,
            make_request(
                make_profile(("lua", "cylinder_fuel_trim"), trim_count=6)
            ),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertTrue(result["estimate"]["known"])
        self.assertEqual(
            {
                "regions_used_bytes": {
                    "flash": 12,
                    "ram0": 21,
                    "ram1": 41,
                    "ram2": 0,
                    "ram3": 0,
                    "ram4": 0,
                    "ram5": 0,
                    "ram6": 0,
                    "ram7": 0,
                },
                "persistent_used_bytes": 80,
                "heap_capacity_bytes": 8,
            },
            result["estimate"]["summed_deltas"],
        )
        result = matrix.evaluate(
            index,
            make_request(
                make_profile(
                    ("lua", "cylinder_fuel_trim"), trim_count=12
                )
            ),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertFalse(result["estimate"]["known"])
        self.assertEqual(
            ["cylinder_fuel_trim"],
            result["estimate"]["unknown"]["missing_features"],
        )

    def test_interaction_residuals_use_the_target_trim_model(self):
        index = make_index(
            make_report(),
            make_report(features=("lua",)),
            make_report(features=("boost",)),
            make_report(features=("cylinder_fuel_trim",), trim_count=6),
            make_report(features=("cylinder_fuel_trim",), trim_count=12),
            make_report(
                features=("lua", "cylinder_fuel_trim"), trim_count=6
            ),
            make_report(
                features=("lua", "cylinder_fuel_trim"), trim_count=12
            ),
            make_report(
                features=("lua", "cylinder_fuel_trim"),
                trim_count=6,
                ve_rpm_count=32,
            ),
            make_report(
                features=("lua", "cylinder_fuel_trim"),
                trim_count=6,
                airmass_blend_size=16,
            ),
            make_report(
                features=("lua", "cylinder_fuel_trim"),
                trim_count=6,
                presets=("MINIMAL_PINS", "PROTEUS_BMW_M73"),
                preset_default="PROTEUS_BMW_M73",
            ),
        )
        result = matrix.evaluate(
            index,
            make_request(
                make_profile(("lua", "cylinder_fuel_trim", "boost"), trim_count=6)
            ),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertTrue(result["estimate"]["known"])
        residuals = result["estimate"]["interaction_diagnostics"]
        self.assertEqual(1, len(residuals))
        self.assertEqual(
            ["cylinder_fuel_trim", "lua"], residuals[0]["features"]
        )

    def test_dimension_change_is_not_reused(self):
        index = make_index(
            make_report(),
            make_report(features=("boost",), ve_rpm_count=32),
        )
        result = matrix.evaluate(
            index,
            make_request(make_profile(("boost",), ve_rpm_count=16)),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual(
            ["boost"], result["estimate"]["unknown"]["missing_features"]
        )

    def test_ram_alias_banks_are_not_aggregated(self):
        index = make_index(
            make_report(),
            make_report(
                features=("boost",),
                feature_usage={"boost": {"ram0": 10, "ram1": 20}},
            ),
            make_report(features=("lua",)),
        )
        result = matrix.evaluate(
            index,
            make_request(make_profile(("boost", "lua"))),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertTrue(result["estimate"]["known"])
        predicted = result["estimate"]["predicted"]["regions"]
        self.assertEqual(110, predicted["ram0"]["estimated_used_bytes"])
        self.assertEqual(220, predicted["ram1"]["estimated_used_bytes"])
        self.assertNotIn("ram_total", predicted)
        self.assertNotIn("ram_total", result["estimate"]["summed_deltas"])

    def test_board_required_module_is_part_of_the_estimate_baseline(self):
        index = make_index(
            make_report(features=("sd_card",), board_id="atlas"),
            make_report(
                features=("sd_card", "boost"),
                board_id="atlas",
                feature_usage={"boost": {"flash": 10, "ram0": 20}},
            ),
        )
        result = matrix.evaluate(
            index,
            make_request(
                make_profile(("sd_card", "boost")),
                board_id="atlas",
                required_modules=("sd_card",),
            ),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )

        self.assertEqual("requires_build", result["status"])
        self.assertTrue(result["estimate"]["known"])
        self.assertEqual(
            ["boost"], result["estimate"]["target_optional_modules"]
        )
        self.assertIn(
            "sd_card", result["estimate"]["baseline_profile"]["modules"]
        )
        self.assertEqual(
            ["boost"], list(result["estimate"]["deltas_by_feature"])
        )

    def test_conflicting_exact_observations_are_not_verified(self):
        policy = make_policy()
        accepted = make_report(policy=copy.deepcopy(policy))
        rejected = make_report(
            policy=copy.deepcopy(policy),
            feature_usage={"overflow": {"persistent": 500}},
        )
        self.assertEqual(accepted["policy"], rejected["policy"])
        self.assertNotEqual(
            accepted["persistent"]["used_bytes"],
            rejected["persistent"]["used_bytes"],
        )
        result = matrix.evaluate(
            make_index(accepted, rejected),
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual("conflicting_observations", result["reason"])

    def test_violation_only_rejection_is_an_incomplete_diagnostic(self):
        report = make_report(force_status="rejected")
        self.assertTrue(
            all(
                component["status"] == "accepted"
                for component in report["regions"].values()
            )
        )
        self.assertEqual("accepted", report["persistent"]["status"])
        self.assertEqual("accepted", report["heap"]["status"])
        report["violations"] = [
            {
                "code": "unrelated_warning",
                "message": "this text does not prove a resource failure",
            }
        ]
        index = make_index(report)
        self.assertEqual([], index["observations"])
        self.assertEqual(1, len(index["diagnostics"]))
        self.assertFalse(index["diagnostics"][0]["complete"])
        self.assertEqual(
            "diagnostic", index["diagnostics"][0]["observation_kind"]
        )
        result = matrix.evaluate(
            index,
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual(
            "incomplete_measurement_diagnostic", result["reason"]
        )
        self.assertNotEqual("incompatible", result["status"])

    def test_incomplete_extraction_diagnostic_is_not_an_overflow(self):
        report = make_report(force_status="rejected")
        report["regions"] = {}
        report["heap"] = {}
        report["persistent"] = {}
        report["elf_sha256"] = None
        report["violations"] = [
            {"code": "extraction_failed", "message": "ELF is absent"}
        ]
        result = matrix.evaluate(
            make_index(report),
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual(
            "incomplete_measurement_diagnostic", result["reason"]
        )
        self.assertNotEqual("incompatible", result["status"])

    def test_rejected_report_missing_bank_remains_a_requires_build_diagnostic(self):
        policy = make_policy(omitted=ALL_RAM_REGIONS)
        report = make_report(
            policy=policy,
            feature_usage={"overflow": {"ram0": 4096}},
        )
        del report["regions"]["ram7"]
        index = make_index(report)
        self.assertEqual([], index["observations"])
        self.assertEqual(1, len(index["diagnostics"]))
        self.assertFalse(index["diagnostics"][0]["complete"])
        result = matrix.evaluate(
            index,
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            policy["id"],
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual(
            "incomplete_measurement_diagnostic", result["reason"]
        )

    def test_malformed_cache_returns_requires_build(self):
        index = make_index(make_report())
        index["observations"][0]["status"] = "accepted"
        index["observations"][0]["persistent"]["free_bytes"] = -1
        result = matrix.evaluate(
            index,
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual("resource_index_unavailable", result["reason"])

    def test_evaluate_cannot_verify_an_index_with_a_reused_policy_id(self):
        index = make_index(make_report())
        index["observations"][0]["policy"]["required_free_bytes"]["ram0"] += 1
        result = matrix.evaluate(
            index,
            make_request(make_profile()),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertEqual("resource_index_unavailable", result["reason"])

    def test_invalid_canonical_request_is_structured_incompatible(self):
        result = matrix.evaluate(
            make_index(),
            {"board_id": "core8", "cpu": "ARCH_STM32F4", "profile": {}},
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("incompatible", result["status"])
        self.assertEqual("invalid_canonical_request", result["reason"])

    def test_multi_feature_residual_is_diagnostic_only(self):
        index = make_index(
            make_report(),
            make_report(
                features=("boost",),
                feature_usage={"boost": {"ram0": 10}},
            ),
            make_report(
                features=("lua",),
                feature_usage={"lua": {"ram0": 10}},
            ),
            make_report(
                features=("engine_sniffer",),
                feature_usage={"engine_sniffer": {"ram0": 10}},
            ),
            make_report(
                features=("boost", "lua"),
                feature_usage={
                    "boost": {"ram0": 10},
                    "lua": {"ram0": 10},
                },
            ),
        )
        result = matrix.evaluate(
            index,
            make_request(
                make_profile(("boost", "lua", "engine_sniffer"))
            ),
            COMMIT,
            "arm-none-eabi-g++",
            DEFAULT_POLICY_ID,
        )
        self.assertEqual("requires_build", result["status"])
        self.assertTrue(result["estimate"]["known"])
        residuals = result["estimate"]["interaction_diagnostics"]
        self.assertEqual(1, len(residuals))
        self.assertEqual(0, residuals[0]["residual_observed_minus_estimate"][
            "regions.ram0.used_bytes"
        ])
        self.assertNotIn('"confidence":', json.dumps(result["estimate"]))


class CliTests(unittest.TestCase):
    def test_invalid_merge_is_nonzero_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            output = directory / "index.json"
            output.write_text('{"sentinel": true}\n', encoding="utf-8")
            observation = directory / "bad.json"
            bad = make_report()
            del bad["board_id"]
            observation.write_text(json.dumps(bad), encoding="utf-8")
            code = matrix.main(
                [
                    "merge",
                    "--observations",
                    str(observation),
                    "--output",
                    str(output),
                ]
            )
            self.assertEqual(2, code)
            self.assertEqual(
                '{"sentinel": true}\n',
                output.read_text(encoding="utf-8"),
            )

    def test_invalid_catalog_constraint_uses_incompatible_status(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            index_path = directory / "index.json"
            result_path = directory / "result.json"
            index_path.write_text(
                json.dumps(make_index()), encoding="utf-8"
            )
            with mock.patch.object(
                matrix.catalog,
                "resolve",
                side_effect=matrix.catalog.CatalogError(
                    "shock_preload requires CAN2"
                ),
            ):
                code = matrix.main(
                    [
                        "evaluate",
                        "--root",
                        str(directory),
                        "--index",
                        str(index_path),
                        "--board",
                        "hellen72",
                        "--features",
                        "shock_preload",
                        "--source-commit",
                        COMMIT,
                        "--toolchain-id",
                        "arm-none-eabi-g++",
                        "--policy-id",
                        DEFAULT_POLICY_ID,
                        "--output",
                        str(result_path),
                    ]
                )
            self.assertEqual(0, code)
            result = json.loads(result_path.read_text(encoding="utf-8"))
            self.assertEqual("incompatible", result["status"])
            self.assertIn("CAN2", result["message"])

    def test_missing_index_is_structured_requires_build(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            result_path = directory / "result.json"
            resolved = make_request(make_profile())
            with mock.patch.object(
                matrix.catalog, "resolve", return_value=resolved
            ) as resolve:
                code = matrix.main(
                    [
                        "evaluate",
                        "--root",
                        str(directory),
                        "--index",
                        str(directory / "missing.json"),
                        "--board",
                        "core8",
                        "--features",
                        "",
                        "--sd-card",
                        "false",
                        "--source-commit",
                        COMMIT,
                        "--toolchain-id",
                        "arm-none-eabi-g++",
                        "--policy-id",
                        DEFAULT_POLICY_ID,
                        "--output",
                        str(result_path),
                    ]
                )
            self.assertFalse(resolve.call_args.kwargs["sd_card"])
            self.assertEqual(0, code)
            result = json.loads(result_path.read_text(encoding="utf-8"))
            self.assertEqual("requires_build", result["status"])
            self.assertEqual("resource_index_unavailable", result["reason"])

    def test_failed_merge_preserves_existing_index(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            existing_path = directory / "existing.json"
            output_path = directory / "output.json"
            existing = make_index(make_report())
            existing_path.write_text(json.dumps(existing), encoding="utf-8")
            output_path.write_text(json.dumps(existing), encoding="utf-8")
            bad = make_report()
            bad["heap"]["capacity_bytes"] = True
            observation_path = directory / "bad.json"
            observation_path.write_text(json.dumps(bad), encoding="utf-8")
            before = output_path.read_text(encoding="utf-8")
            code = matrix.main(
                [
                    "merge",
                    "--index",
                    str(existing_path),
                    "--observations",
                    str(observation_path),
                    "--output",
                    str(output_path),
                ]
            )
            self.assertEqual(2, code)
            self.assertEqual(before, output_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
