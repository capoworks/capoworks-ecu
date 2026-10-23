"""Standard-library tests for packaging and the custom firmware workflow."""

from __future__ import annotations

import hashlib
import io
import json
import re
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

try:
    from . import pipeline, resources
except ImportError:  # Allow direct execution from this directory.
    import pipeline  # type: ignore[no-redef]
    import resources  # type: ignore[no-redef]


SOURCE_COMMIT = "3060ccd5873a89e6499c9ade5756338ab7c529fd"
PERSISTENT_SIZE = 4096
PROFILE = {
    "modules": ["boost", "fan", "map_averaging", "sd_card"],
    "cylinder_trim_count": 0,
    "ve_rpm_count": 16,
    "airmass_blend_size": 8,
    "engine_preset_mode": "explicit_all",
    "engine_presets": ["MINIMAL_PINS", "PROTEUS_BMW_M73"],
    "engine_preset_default": "MINIMAL_PINS",
}
RESOLVED = {
    "schema_version": 1,
    "board_id": "proteus_f4",
    "board_dir": "firmware/config/boards/proteus",
    "compile_script": "compile_proteus_f4.sh",
    "ini_file": "fome_proteus_f4.ini",
    "cpu": "ARCH_STM32F4",
    "base_modules": [
        "fan",
        "fuel_pump",
        "gear_detector",
        "tachometer",
        "trip_odometer",
        "map_averaging",
    ],
    "optional_modules": ["boost", "sd_card"],
    "profile": {**PROFILE, "engine_preset_mode": "default_all"},
}


def _ini_bytes(
    *,
    page_size: int = PERSISTENT_SIZE - 20,
    ve_rpm_count: int = 16,
    airmass_blend_size: int = 8,
    suffix: str = "",
) -> bytes:
    return (
        "[Constants]\n"
        f"pageSize = {page_size}\n"
        f"veTable = array, U16, 1000, "
        f"[{ve_rpm_count}x16], \"%\", 0.1, 0, 0, 999, 1\n"
        f"airmassBlendTable = array, U08, 2000, "
        f"[{airmass_blend_size}x{airmass_blend_size}], "
        f"\"% Alpha-N\", 1, 0, 0, 100, 0\n"
        f"{suffix}"
    ).encode("utf-8")


def _zip_bytes(name: str, contents: bytes) -> bytes:
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(name, contents)
    return stream.getvalue()


def _nm(symbols: dict[str, int | tuple[int, int | None]]) -> str:
    lines = []
    for name, value in symbols.items():
        if isinstance(value, tuple):
            address, size = value
        else:
            address, size = value, None
        if size is None:
            lines.append(f"{address:016x} A {name}")
        else:
            lines.append(f"{address:016x} {size:08x} D {name}")
    return "\n".join(lines) + "\n"


def _objdump() -> str:
    return (
        "\nSections:\n"
        "Idx Name Size VMA LMA File off Algn\n"
        "  0 .text 00010000 08008000 08008000 00010000 2**2\n"
        "                  CONTENTS, ALLOC, LOAD, READONLY, CODE\n"
    )


def _realistic_report(elf_sha256: str, *, profile: dict = PROFILE) -> dict:
    symbols: dict[str, int | tuple[int, int | None]] = {
        "__flash0_base__": 0x08008000,
        "__flash0_end__": 0x080C0000,
        "__flash0_free__": 0x08090000,
        "__heap_base__": 0x20018000,
        "__heap_end__": 0x20020000,
        "persistentState": (0x10000000, PERSISTENT_SIZE),
    }
    for index in range(8):
        symbols[f"__ram{index}_base__"] = 0
        symbols[f"__ram{index}_end__"] = 0
        symbols[f"__ram{index}_free__"] = 0
    symbols["__ram0_base__"] = 0x20000020
    symbols["__ram0_end__"] = 0x20020000
    symbols["__ram0_free__"] = 0x20018000
    symbols["__ram4_base__"] = 0x10000000
    symbols["__ram4_end__"] = 0x10010000
    symbols["__ram4_free__"] = 0x10000000

    report = resources.assemble_report(
        board_id="proteus_f4",
        cpu="ARCH_STM32F4",
        source_commit=SOURCE_COMMIT,
        profile=profile,
        symbols=resources.parse_nm(_nm(symbols)),
        sections=resources.parse_objdump_sections(_objdump()),
        toolchain={
            "id": "c" * 64,
            "version": "arm-none-eabi-g++ 11.3.1",
            "sha256": "d" * 64,
        },
        elf_sha256=elf_sha256,
        policy_data={
            "schema_version": 1,
            "required_free_bytes": {},
            "minimum_heap_capacity_bytes": 0,
        },
    )
    return report


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


class PackageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / "root"
        self.elf = self.root / "firmware/build/fome.elf"
        self.binary = self.root / "firmware/deliver/fome.bin"
        self.update = self.root / "firmware/deliver/fome_update.srec"
        self.bootloader = self.root / "firmware/deliver/fome_bl.srec"
        self.ini = (
            self.root
            / "firmware/tunerstudio/generated"
            / RESOLVED["ini_file"]
        )
        self.module_profile = (
            self.root / "firmware/generated/module_profile.txt"
        )
        for path in (
            self.elf,
            self.binary,
            self.update,
            self.bootloader,
            self.ini,
            self.module_profile,
        ):
            path.parent.mkdir(parents=True, exist_ok=True)
        self.elf.write_bytes(b"realistic ELF fixture\n")
        self.binary.write_bytes(b"firmware binary\n")
        self.update.write_bytes(b"S0 update\n")
        self.bootloader.write_bytes(b"S0 bootloader\n")
        self.ini.write_bytes(_ini_bytes())
        self._write_generated_profile()

        self.resolved_path = self.base / "resolved-profile.json"
        self.resources_path = self.base / "resources.json"
        self.resolved_path.write_text(json.dumps(RESOLVED), encoding="utf-8")
        self.report = _realistic_report(_sha256(self.elf))
        self.resources_path.write_text(json.dumps(self.report), encoding="utf-8")
        self.output = self.base / "published"

        git_patch = mock.patch.object(
            pipeline, "_git_head", return_value=SOURCE_COMMIT
        )
        self.git_patch = git_patch.start()
        self.addCleanup(git_patch.stop)
        ref_patch = mock.patch.object(
            pipeline, "_git_ref", return_value="refs/heads/feature/testing"
        )
        self.ref_patch = ref_patch.start()
        self.addCleanup(ref_patch.stop)
        catalog_patch = mock.patch.object(
            pipeline.catalog,
            "catalog_document",
            return_value={
                "schema_version": 1,
                "boards": [
                    {
                        "id": "proteus_f4",
                        "board_dir": RESOLVED["board_dir"],
                        "compile_script": RESOLVED["compile_script"],
                        "ini_file": RESOLVED["ini_file"],
                        "cpu": RESOLVED["cpu"],
                        "capabilities": {"can2": True},
                    }
                ],
                "base_modules": RESOLVED["base_modules"],
                "optional_modules": ["boost", "sd_card"],
            },
        )
        self.catalog_patch = catalog_patch.start()
        self.addCleanup(catalog_patch.stop)

    def publish(self) -> dict:
        return pipeline.package(
            root=self.root,
            resolved_path=self.resolved_path,
            resources_path=self.resources_path,
            output=self.output,
            request_id="request-123",
            run_id="991",
            run_attempt="2",
        )

    def _rewrite_report(self, report: dict) -> None:
        self.resources_path.write_text(json.dumps(report), encoding="utf-8")

    def _rewrite_resolved(self, resolved: dict) -> None:
        self.resolved_path.write_text(json.dumps(resolved), encoding="utf-8")

    def _write_generated_profile(
        self,
        *,
        board_dir: str = RESOLVED["board_dir"],
        cpu: str = RESOLVED["cpu"],
        ve_rpm_count: int = 16,
        airmass_blend_size: int = 8,
        modules: list[str] = PROFILE["modules"],
    ) -> None:
        self.module_profile.write_text(
            "\n".join(
                [
                    "modules " + " ".join(modules),
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

    def _select_modules(
        self,
        *,
        sd_card: bool,
        required_modules: list[str] | None = None,
    ) -> None:
        modules = [module for module in PROFILE["modules"] if module != "sd_card"]
        optional_modules = [
            module for module in RESOLVED["optional_modules"] if module != "sd_card"
        ]
        if sd_card:
            modules.append("sd_card")
            optional_modules.append("sd_card")

        profile = {**PROFILE, "modules": modules}
        resolved = {
            **RESOLVED,
            "optional_modules": optional_modules,
            "profile": {**RESOLVED["profile"], "modules": modules},
        }
        if required_modules is not None:
            resolved["required_modules"] = required_modules
        self._rewrite_resolved(resolved)
        self._write_generated_profile(modules=modules)
        self.report = _realistic_report(_sha256(self.elf), profile=profile)
        self._rewrite_report(self.report)

    def _embed_ini(
        self,
        contents: bytes | None = None,
        *,
        name: str | None = None,
    ) -> None:
        archive = _zip_bytes(
            name or RESOLVED["ini_file"],
            self.ini.read_bytes() if contents is None else contents,
        )
        self.binary.write_bytes(b"firmware-prefix" + archive + b"trailing-data")

    def test_accepts_complete_semantically_matching_package(self) -> None:
        manifest = self.publish()

        self.assertTrue(self.output.is_dir())
        self.assertEqual(
            {
                self.ini.name,
                "fome.bin",
                "fome_update.srec",
                "fome_bl.srec",
                "resources.json",
                "resolved-profile.json",
                "resource-index.json",
                "catalog.json",
                "manifest.json",
            },
            {path.name for path in self.output.iterdir()},
        )
        self.assertEqual(2, manifest["schema_version"])
        self.assertEqual("accepted", manifest["resource_check"]["status"])
        self.assertTrue(manifest["resource_check"]["complete"])
        self.assertEqual(16, manifest["resolved"]["profile"]["ve_rpm_count"])
        self.assertEqual(8, manifest["resolved"]["profile"]["airmass_blend_size"])
        self.assertIn("engine_presets", manifest["resolved"]["profile"])
        self.assertEqual("boost", manifest["requested"]["features"])
        self.assertTrue(manifest["requested"]["sd_card"])
        self.assertEqual([], manifest["resolved"]["required_modules"])
        self.assertIn("regions", manifest["measurements"])
        self.assertEqual(SOURCE_COMMIT, manifest["source"]["commit"])
        self.assertEqual(
            "refs/heads/feature/testing", manifest["source"]["ref"]
        )
        self.assertEqual("991", manifest["run"]["id"])
        self.assertEqual("2", manifest["run"]["attempt"])

        index = json.loads(
            (self.output / "resource-index.json").read_text(encoding="utf-8")
        )
        self.assertEqual(1, len(index["observations"]))
        self.assertEqual([], index["diagnostics"])
        self.assertEqual(
            self.resources_path.read_bytes(),
            (self.output / "resources.json").read_bytes(),
        )

        artifacts = {item["file"]: item for item in manifest["artifacts"]}
        self.assertNotIn("manifest.json", artifacts)
        for name, item in artifacts.items():
            path = self.output / name
            self.assertEqual(_sha256(path), item["sha256"])
            self.assertEqual(path.stat().st_size, item["size_bytes"])
        manifest_text = (self.output / "manifest.json").read_text(encoding="utf-8")
        self.assertNotIn(str(self.root), manifest_text)
        self.assertNotIn(str(self.base), manifest_text)

    def test_manifest_keeps_sd_out_of_normalized_feature_selection(self) -> None:
        self._select_modules(sd_card=False)

        manifest = self.publish()

        self.assertEqual("boost", manifest["requested"]["features"])
        self.assertFalse(manifest["requested"]["sd_card"])
        self.assertEqual(["boost"], manifest["resolved"]["optional_modules"])
        self.assertEqual([], manifest["resolved"]["required_modules"])
        self.assertNotIn("sd_card", manifest["resolved"]["modules"])

    def test_manifest_preserves_required_module_metadata(self) -> None:
        self._select_modules(sd_card=True, required_modules=["sd_card"])

        manifest = self.publish()

        self.assertEqual("boost", manifest["requested"]["features"])
        self.assertTrue(manifest["requested"]["sd_card"])
        self.assertEqual(["sd_card"], manifest["resolved"]["required_modules"])
        self.assertIn("sd_card", manifest["resolved"]["modules"])

    def test_accepts_matching_embedded_ini(self) -> None:
        self._embed_ini()

        manifest = self.publish()

        self.assertEqual("accepted", manifest["resource_check"]["status"])
        self.assertTrue(self.output.is_dir())

    def test_rejected_resource_report_creates_no_output(self) -> None:
        report = dict(self.report)
        report["status"] = "rejected"
        report["violations"] = [
            {
                "code": "policy_free_below_minimum",
                "message": "flash is below the configured minimum",
            }
        ]
        self._rewrite_report(report)

        with self.assertRaisesRegex(pipeline.PackageError, "resource gate rejected"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_missing_elf_metadata_creates_no_output(self) -> None:
        report = dict(self.report)
        report["status"] = "rejected"
        report["elf_sha256"] = None
        report["violations"] = [
            {"code": "extraction_failed", "message": "ELF could not be inspected"}
        ]
        self._rewrite_report(report)

        with self.assertRaises(pipeline.PackageError):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_wrong_counts_or_presets_create_no_output(self) -> None:
        cases = {
            "count": {
                **RESOLVED,
                "profile": {**RESOLVED["profile"], "ve_rpm_count": 32},
            },
            "preset": {
                **RESOLVED,
                "profile": {
                    **RESOLVED["profile"],
                    "engine_presets": ["MINIMAL_PINS"],
                },
            },
        }
        for label, resolved in cases.items():
            with self.subTest(label=label):
                if self.output.exists():
                    self.output.rmdir()
                self._rewrite_resolved(resolved)
                with self.assertRaisesRegex(pipeline.PackageError, "does not match"):
                    self.publish()
                self.assertFalse(self.output.exists())
                self._rewrite_resolved(RESOLVED)

    def test_stale_elf_creates_no_output(self) -> None:
        self.elf.write_bytes(b"different ELF\n")

        with self.assertRaisesRegex(pipeline.PackageError, "ELF SHA-256"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_missing_ini_creates_no_output(self) -> None:
        self.ini.unlink()

        with self.assertRaisesRegex(pipeline.PackageError, "generated INI"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_generated_profile_for_a_different_board_creates_no_output(self) -> None:
        self._write_generated_profile(board_dir="firmware/config/boards/core8")

        with self.assertRaisesRegex(pipeline.PackageError, "board .* does not match"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_generated_profile_with_a_different_cpu_creates_no_output(self) -> None:
        self._write_generated_profile(cpu="ARCH_STM32H7")

        with self.assertRaisesRegex(pipeline.PackageError, "CPU .* does not match"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_generated_profile_mismatch_creates_no_output(self) -> None:
        self._write_generated_profile(ve_rpm_count=32)

        with self.assertRaisesRegex(
            pipeline.PackageError, "generated module profile does not match"
        ):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_accepts_all_supported_ini_geometry_combinations(self) -> None:
        page_size = PERSISTENT_SIZE - pipeline.PERSISTENT_WRAPPER_BYTES
        persistent_size = page_size + pipeline.PERSISTENT_WRAPPER_BYTES
        for ve_rpm_count, airmass_blend_size in (
            (16, 8),
            (32, 8),
            (16, 16),
            (32, 16),
        ):
            with self.subTest(
                ve_rpm_count=ve_rpm_count,
                airmass_blend_size=airmass_blend_size,
            ):
                pipeline._validate_ini_layout(
                    ini_bytes=_ini_bytes(
                        page_size=page_size,
                        ve_rpm_count=ve_rpm_count,
                        airmass_blend_size=airmass_blend_size,
                    ),
                    persistent_size_bytes=persistent_size,
                    ve_rpm_count=ve_rpm_count,
                    airmass_blend_size=airmass_blend_size,
                )

    def test_accepts_same_key_in_setting_context_help(self) -> None:
        self.ini.write_bytes(
            _ini_bytes(
                suffix=(
                    "; airmassBlendTable = \"commented-out contribution\"\n"
                    "#if USE_FAHRENHEIT\n"
                    "#endif\n"
                    " [ SettingContextHelp ] \n"
                    'airmassBlendTable = "Alpha-N contribution to cylinder air mass"\n'
                )
            )
        )

        manifest = self.publish()

        self.assertTrue(self.output.is_dir())
        self.assertEqual(8, manifest["resolved"]["profile"]["airmass_blend_size"])

    def test_rejects_duplicate_constants_field(self) -> None:
        self.ini.write_bytes(
            _ini_bytes(
                suffix=(
                    "airmassBlendTable = array, U08, 2200, [8x8], "
                    "\"% Alpha-N\", 1, 0, 0, 100, 0\n"
                )
            )
        )

        with self.assertRaisesRegex(
            pipeline.PackageError, "exactly one airmassBlendTable field"
        ):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_stale_ini_geometry_creates_no_output(self) -> None:
        self.ini.write_bytes(_ini_bytes(airmass_blend_size=16))

        with self.assertRaisesRegex(pipeline.PackageError, "geometry"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_stale_ini_page_size_creates_no_output(self) -> None:
        self.ini.write_bytes(_ini_bytes(page_size=PERSISTENT_SIZE - 21))

        with self.assertRaisesRegex(pipeline.PackageError, "pageSize"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_embedded_ini_mismatch_creates_no_output(self) -> None:
        self._embed_ini(_ini_bytes(suffix="; stale embedded copy\n"))

        with self.assertRaisesRegex(pipeline.PackageError, "embedded INI"):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_missing_embedded_ini_entry_creates_no_output(self) -> None:
        self._embed_ini(name="different.ini")

        with self.assertRaisesRegex(
            pipeline.PackageError, "missing expected INI entry"
        ):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_malformed_embedded_archive_creates_no_output(self) -> None:
        self.binary.write_bytes(b"firmware-prefixPK\x05\x06trailing-data")

        with self.assertRaisesRegex(
            pipeline.PackageError, "malformed embedded ZIP"
        ):
            self.publish()
        self.assertFalse(self.output.exists())

    def test_existing_output_is_preserved(self) -> None:
        self.output.mkdir()
        marker = self.output / "old-success"
        marker.write_text("keep me\n", encoding="utf-8")

        with self.assertRaisesRegex(pipeline.PackageError, "refusing to replace"):
            self.publish()
        self.assertEqual("keep me\n", marker.read_text(encoding="utf-8"))
        self.assertEqual({"old-success"}, {path.name for path in self.output.iterdir()})

    def test_detached_head_packages_with_null_source_ref(self) -> None:
        self.ref_patch.return_value = None

        manifest = self.publish()

        self.assertIsNone(manifest["source"]["ref"])
        self.assertEqual(SOURCE_COMMIT, manifest["source"]["commit"])


class GitRefTests(unittest.TestCase):
    def test_symbolic_ref_uses_explicit_repository_argument(self) -> None:
        completed = mock.Mock(
            returncode=0,
            stdout="refs/heads/feature/testing\n",
            stderr="",
        )
        with mock.patch.object(
            pipeline.subprocess, "run", return_value=completed
        ) as run:
            ref = pipeline._git_ref(Path("/tmp/firmware"))

        self.assertEqual("refs/heads/feature/testing", ref)
        run.assert_called_once_with(
            [
                "git",
                "-C",
                "/tmp/firmware",
                "symbolic-ref",
                "--quiet",
                "HEAD",
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_detached_head_has_no_symbolic_ref(self) -> None:
        completed = mock.Mock(returncode=1, stdout="", stderr="")
        with mock.patch.object(
            pipeline.subprocess, "run", return_value=completed
        ):
            self.assertIsNone(pipeline._git_ref(Path("/tmp/firmware")))

    def test_unexpected_symbolic_ref_failure_is_rejected(self) -> None:
        completed = mock.Mock(
            returncode=128,
            stdout="",
            stderr="fatal: not a git repository",
        )
        with mock.patch.object(
            pipeline.subprocess, "run", return_value=completed
        ):
            with self.assertRaisesRegex(
                pipeline.PackageError, "checked-out Git ref"
            ):
                pipeline._git_ref(Path("/tmp/firmware"))


def _input_section(workflow: str, name: str) -> str:
    match = re.search(
        rf"(?ms)^      {re.escape(name)}:\n(?P<body>.*?)(?=^      [A-Za-z0-9_]+:\n|^permissions:)",
        workflow,
    )
    if not match:
        raise AssertionError(f"workflow input {name!r} is missing")
    return match.group("body")


def _workflow_step(workflow: str, name: str) -> str:
    match = re.search(
        rf"(?ms)^      - name: {re.escape(name)}\n(?P<body>.*?)"
        r"(?=^      - (?:name:|uses:)|^  [A-Za-z0-9_]+:|\Z)",
        workflow,
    )
    if not match:
        raise AssertionError(f"workflow step {name!r} is missing")
    return match.group("body")


class WorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.repo = Path(__file__).resolve().parents[2]
        cls.workflow = (
            cls.repo / ".github/workflows/custom-firmware-build.yaml"
        ).read_text(encoding="utf-8")
        cls.resource_workflow = (
            cls.repo / ".github/workflows/firmware-resource-matrix.yaml"
        ).read_text(encoding="utf-8")
        cls.boards = json.loads(
            (cls.repo / "misc/firmware_builder/boards.json").read_text(
                encoding="utf-8"
            )
        )["boards"]

    def test_board_choices_match_catalog_without_pyyaml(self) -> None:
        section = _input_section(self.workflow, "board")
        choices = re.findall(r"(?m)^          - ([A-Za-z0-9_-]+)$", section)
        self.assertEqual(14, len(choices))
        self.assertEqual([board["id"] for board in self.boards], choices)

    def test_new_profile_inputs_have_constrained_choices(self) -> None:
        expectations = {
            "ve_rpm_count": (["16", "32"], "16"),
            "airmass_blend_size": (["8", "16"], "8"),
            "cylinder_trim_count": (
                [str(value) for value in range(1, 13)],
                "6",
            ),
        }
        for name, (values, default) in expectations.items():
            with self.subTest(name=name):
                section = _input_section(self.workflow, name)
                self.assertIn("type: choice", section)
                choices = re.findall(r"(?m)^          - '?([0-9]+)'?$", section)
                self.assertEqual(values, choices)
                self.assertIn(f"default: '{default}'", section)
        presets = _input_section(self.workflow, "engine_presets")
        self.assertIn("type: string", presets)
        self.assertIn("default: ''", presets)
        sd_card = _input_section(self.workflow, "sd_card")
        self.assertIn("type: boolean", sd_card)
        self.assertIn("default: true", sd_card)

    def test_workflow_uses_catalog_and_gated_package_helper(self) -> None:
        self.assertIn("misc/firmware_builder/catalog.py", self.workflow)
        self.assertIn('resolve_args=(', self.workflow)
        self.assertIn('--board "$INPUT_BOARD"', self.workflow)
        self.assertIn('--features "$INPUT_FEATURES"', self.workflow)
        self.assertIn(
            "INPUT_SD_CARD: ${{ inputs.sd_card }}", self.workflow
        )
        self.assertIn('--sd-card "$INPUT_SD_CARD"', self.workflow)
        self.assertIn('--github-env "$GITHUB_ENV"', self.workflow)
        self.assertIn("misc/firmware_builder/resources.py", self.workflow)
        self.assertIn("misc/firmware_builder/pipeline.py", self.workflow)
        self.assertNotIn("base_modules=(", self.workflow)
        self.assertNotIn("declare -A selected", self.workflow)

    def test_resource_workflow_forwards_sd_card_boolean(self) -> None:
        self.assertIn("SD_CARD: ${{ matrix.sd_card }}", self.resource_workflow)
        self.assertIn('--sd-card "$SD_CARD"', self.resource_workflow)

    def test_gate_precedes_firmware_publication(self) -> None:
        extract = self.workflow.index("Extract and gate resources")
        package = self.workflow.index("Package validated firmware")
        upload = self.workflow.index("Upload accepted firmware package")
        diagnostics = self.workflow.index("Upload resource diagnostics")
        self.assertLess(extract, diagnostics)
        self.assertLess(diagnostics, package)
        self.assertLess(package, upload)
        diagnostic_step = self.workflow[diagnostics:package]
        self.assertIn("if: always()", diagnostic_step)
        self.assertIn("resources.json", diagnostic_step)
        self.assertNotIn("fome.bin", diagnostic_step)
        self.assertNotIn("manifest.json", diagnostic_step)

    def test_source_commit_and_build_controls_are_pinned(self) -> None:
        self.assertIn("ref: master", self.workflow)
        self.assertIn("permissions:\n  contents: read", self.workflow)
        self.assertIn("SOURCE_COMMIT", self.workflow)
        capture_step = _workflow_step(self.workflow, "Capture source revision")
        verify_step = _workflow_step(self.workflow, "Verify source revision")
        repository_git = re.compile(
            r'\bgit\s+-C\s+"\$GITHUB_WORKSPACE"\s+rev-parse\s+HEAD\b'
        )
        self.assertRegex(capture_step, repository_git)
        self.assertRegex(verify_step, repository_git)
        self.assertIn('source_commit="$(git -C', capture_step)
        self.assertIn('printf \'SOURCE_COMMIT=%s\\n\' "$source_commit"', capture_step)
        self.assertIn('actual_commit="$(git -C', verify_step)
        self.assertIn('"$actual_commit" != "$SOURCE_COMMIT"', verify_step)
        self.assertIn("JOBS: '4'", self.workflow)
        self.assertLess(
            self.workflow.index("Capture source revision"),
            self.workflow.index("Build firmware"),
        )
        self.assertLess(
            self.workflow.index("Build firmware"),
            self.workflow.index("Verify source revision"),
        )


if __name__ == "__main__":
    unittest.main()
