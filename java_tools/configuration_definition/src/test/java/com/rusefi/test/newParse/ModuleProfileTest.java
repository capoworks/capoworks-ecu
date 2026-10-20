package com.rusefi.test.newParse;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

public class ModuleProfileTest {
    private static final Path FIRMWARE = Paths.get("../../firmware").toAbsolutePath().normalize();
    private static final Path BOARD = FIRMWARE.resolve("config/boards/f407-discovery");
    private static final Path PROFILE_TOOL = FIRMWARE.resolve("controllers/modules/module_profile.sh");
    private static final Path PROFILE_MAKE = FIRMWARE.resolve("controllers/modules/module_profile_host.mk");

    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void cylinderFuelTrimDefaultsOnWithCapacitySixAndIniFragment() throws Exception {
        Path profileDirectory = temporaryFolder.newFolder("default").toPath();
        Path profile = profileDirectory.resolve("module_profile.txt");
        Path definitions = profileDirectory.resolve("module_profile_defines.h");

        ProcessResult defaultResult = runProfileMake(profile, null, null);
        assertEquals(defaultResult.output, 0, defaultResult.exitCode);

        List<String> modules = Arrays.asList(profileValue(profile, "modules").split("\\s+"));
        assertTrue(modules.contains("fan"));
        assertTrue(modules.contains("cylinder_fuel_trim"));
        assertEquals("6", profileValue(profile, "cylinder_fuel_trim_count"));
        assertTrue(read(definitions).contains("#define CYLINDER_FUEL_TRIM_COUNT 6"));

        Path generatedIni = profileDirectory.resolve("cylinder_fuel_trim.ini");
        assertTrue(read(profile).contains("ini " + generatedIni + "\n"));
        assertTrue(Files.isRegularFile(generatedIni));
        assertTrue(read(generatedIni).contains("cylinderFuelTrimTbl6"));
        assertFalse(read(generatedIni).contains("cylinderFuelTrimTbl7"));
    }

    @Test
    public void explicitProfileWithoutCylinderFuelTrimCanonicalizesToZero() throws Exception {
        // FOME_MODULES replaces the default profile, so stale values are ignored.
        Path offDirectory = temporaryFolder.newFolder("off-with-value").toPath();
        Path offProfile = offDirectory.resolve("module_profile.txt");
        Path offDefinitions = offDirectory.resolve("module_profile_defines.h");
        ProcessResult offResult = runProfileMake(offProfile, "fan", "13");
        assertEquals(offResult.output, 0, offResult.exitCode);
        assertFalse(profileValue(offProfile, "modules").contains("cylinder_fuel_trim"));
        assertEquals("0", profileValue(offProfile, "cylinder_fuel_trim_count"));
        assertTrue(read(offDefinitions).contains("#define CYLINDER_FUEL_TRIM_COUNT 0"));
        assertFalse(read(offProfile).contains("cylinder_fuel_trim.ini"));
    }

    @Test
    public void selectedCylinderFuelTrimDefaultsToCapacitySix() throws Exception {
        Path profileDirectory = temporaryFolder.newFolder("selected-default").toPath();
        Path profile = profileDirectory.resolve("module_profile.txt");
        Path definitions = profileDirectory.resolve("module_profile_defines.h");

        ProcessResult result = runProfileMake(profile, "fan cylinder_fuel_trim", null);
        assertEquals(result.output, 0, result.exitCode);
        assertTrue(profileValue(profile, "modules").contains("cylinder_fuel_trim"));
        assertEquals("6", profileValue(profile, "cylinder_fuel_trim_count"));
        assertTrue(read(definitions).contains("#define CYLINDER_FUEL_TRIM_COUNT 6"));
    }

    @Test
    public void selectedCylinderFuelTrimAcceptsExplicitSupportedCounts() throws Exception {
        for (String count : Arrays.asList("1", "4", "6", "12")) {
            Path profileDirectory = temporaryFolder.newFolder("selected-" + count).toPath();
            Path profile = profileDirectory.resolve("module_profile.txt");
            Path definitions = profileDirectory.resolve("module_profile_defines.h");

            ProcessResult result = runProfileMake(profile, "fan cylinder_fuel_trim", count);
            assertEquals(result.output, 0, result.exitCode);
            assertTrue(profileValue(profile, "modules").contains("cylinder_fuel_trim"));
            assertEquals(count, profileValue(profile, "cylinder_fuel_trim_count"));
            assertTrue(read(definitions).contains("#define CYLINDER_FUEL_TRIM_COUNT " + count));
        }
    }

    @Test
    public void selectedCylinderFuelTrimRejectsInvalidCount() throws Exception {
        assertRejected("", "integer from 1 to 12");
        assertRejected("0", "integer from 1 to 12");
        assertRejected("13", "integer from 1 to 12");
        assertRejected("4.5", "integer from 1 to 12");
        assertRejected("4 5", "integer from 1 to 12");
        assertRejected("%", "integer from 1 to 12");
    }

    @Test
    public void defaultFuelTableDimensionsRenderLegacyProfileAndMacros() throws Exception {
        Path directory = temporaryFolder.newFolder("default-dimensions").toPath();
        Path profile = directory.resolve("module_profile.txt");
        Path definitions = directory.resolve("module_profile_defines.h");

        ProcessResult result = runProfileMake(profile, "fan", null);
        assertEquals(result.output, 0, result.exitCode);
        assertTableDimensions(profile, definitions, "16", "8");
        assertEquals("default_all", profileValue(profile, "engine_preset_mode"));
        assertEquals("MINIMAL_PINS", profileValue(profile, "engine_preset_default"));
        assertTrue(profileValue(profile, "engine_presets").contains("MINIMAL_PINS"));
    }

    @Test
    public void supportedFuelTableDimensionPairsRenderMatchingProfileAndMacros() throws Exception {
        String[][] choices = {
                {"16", "8"},
                {"16", "16"},
                {"32", "8"},
                {"32", "16"},
        };

        for (String[] choice : choices) {
            String veRpmCount = choice[0];
            String airmassBlendSize = choice[1];
            Path directory = temporaryFolder.newFolder("dimensions-" + veRpmCount + "-" + airmassBlendSize).toPath();
            Path profile = directory.resolve("module_profile.txt");
            Path definitions = directory.resolve("module_profile_defines.h");

            ProcessResult result = runProfileMake(
                    profile, "fan", null, veRpmCount, airmassBlendSize);
            assertEquals(result.output, 0, result.exitCode);
            assertTableDimensions(profile, definitions, veRpmCount, airmassBlendSize);
            if (veRpmCount.equals("16") && airmassBlendSize.equals("8")) {
                assertEquals("default_all", profileValue(profile, "engine_preset_mode"));
            } else {
                assertEquals("default_none", profileValue(profile, "engine_preset_mode"));
                assertEquals("MINIMAL_PINS", profileValue(profile, "engine_presets"));
            }
        }
    }

    @Test
    public void enginePresetSelectionIsCanonicalAndExpandedBuildsRejectSymbols() throws Exception {
        Path directory = temporaryFolder.newFolder("engine-presets").toPath();
        Path profile = directory.resolve("module_profile.txt");

        ProcessResult first = runProfileMake(
                profile, "fan", null, "16", "8", "HELLEN_NA6");
        assertEquals(first.output, 0, first.exitCode);
        assertEquals("explicit", profileValue(profile, "engine_preset_mode"));
        assertTrue(profileValue(profile, "engine_presets").contains("HELLEN_NA6"));
        assertTrue(profileValue(profile, "engine_presets").contains("MINIMAL_PINS"));

        Files.setLastModifiedTime(profile, FileTime.fromMillis(1000));
        ProcessResult second = runProfileMake(
                profile, "fan", null, "16", "8", "HELLEN_NB2_36");
        assertEquals(second.output, 0, second.exitCode);
        assertTrue(profileValue(profile, "engine_presets").contains("HELLEN_NB2_36"));
        assertFalse(profileValue(profile, "engine_presets").contains("HELLEN_NA6"));
        assertTrue(Files.getLastModifiedTime(profile).toMillis() > 1000);

        ProcessResult none = runProfileMake(
                profile, "fan", null, "32", "16", "none");
        assertEquals(none.output, 0, none.exitCode);
        assertEquals("explicit_none", profileValue(profile, "engine_preset_mode"));
        assertEquals("MINIMAL_PINS", profileValue(profile, "engine_presets"));

        ProcessResult rejected = runProfileMake(
                profile, "fan", null, "32", "16", "HELLEN_NA6");
        assertNotEquals(rejected.output, 0, rejected.exitCode);
        assertTrue(rejected.output, rejected.output.contains("cannot include literal engine presets"));
    }

    @Test
    public void fuelTableDimensionValidationRejectsUnsupportedValues() throws Exception {
        for (String value : Arrays.asList("", "8", "24", "16 32", "not-a-count")) {
            assertDimensionRejected(value, null, "FOME_VE_RPM_COUNT");
        }
        for (String value : Arrays.asList("", "4", "12", "8 16", "not-a-count")) {
            assertDimensionRejected(null, value, "FOME_AIRMASS_BLEND_SIZE");
        }
    }

    @Test
    public void fuelTableDimensionRoundTripRefreshesWithoutClean() throws Exception {
        Path directory = temporaryFolder.newFolder("dimension-round-trip").toPath();
        Path profile = directory.resolve("module_profile.txt");
        Path definitions = directory.resolve("module_profile_defines.h");
        Path object = directory.resolve("object.o");
        Path configStamp = directory.resolve("config.stamp");
        Path makefile = directory.resolve("profile-deps.mk");
        Path buildDirectory = directory.resolve("build");
        Path generatedDirectory = directory.resolve("generated");

        Files.createDirectories(buildDirectory);
        Files.createDirectories(generatedDirectory);
        Files.write(makefile, Arrays.asList(
                "PROJECT_DIR := " + FIRMWARE,
                "BOARD_DIR := " + BOARD,
                "include $(PROJECT_DIR)/controllers/modules/module_profile.mk",
                "include $(PROJECT_DIR)/fome_generated.mk",
                "all: $(OBJS)",
                "$(OBJS):",
                "\t@touch $@"
        ), StandardCharsets.UTF_8);

        ProcessResult first = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan", null, "32", "16", true);
        assertEquals(first.output, 0, first.exitCode);
        assertTableDimensions(profile, definitions, "32", "16");

        Files.setLastModifiedTime(object, FileTime.fromMillis(1000));
        ProcessResult middle = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan", null, "16", "8", true);
        assertEquals(middle.output, 0, middle.exitCode);
        assertTableDimensions(profile, definitions, "16", "8");
        assertTrue("dimension A -> B did not stale the object",
                Files.getLastModifiedTime(object).toMillis() > 1000);

        Files.setLastModifiedTime(object, FileTime.fromMillis(1000));
        ProcessResult returned = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan", null, "32", "16", true);
        assertEquals(returned.output, 0, returned.exitCode);
        assertTableDimensions(profile, definitions, "32", "16");
        assertTrue("dimension B -> A did not stale the object",
                Files.getLastModifiedTime(object).toMillis() > 1000);
    }

    @Test
    public void legacyProfileInvocationsRemainReadable() throws Exception {
        Path directory = temporaryFolder.newFolder("legacy-profile").toPath();
        Path legacyProfile = directory.resolve("legacy.txt");
        Files.write(legacyProfile, Arrays.asList(
                "modules fan",
                "board " + BOARD,
                "cpu ARCH_STM32F4"
        ), StandardCharsets.UTF_8);

        ProcessResult check = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "check", legacyProfile.toString()
        ));
        assertEquals(check.output, 0, check.exitCode);

        ProcessResult count = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "read", legacyProfile.toString(), "cylinder_fuel_trim_count"
        ));
        assertEquals(count.output, 0, count.exitCode);
        assertEquals("0\n", count.output);

        ProcessResult veRpmCount = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "read", legacyProfile.toString(), "ve_rpm_count"
        ));
        assertEquals(veRpmCount.output, 0, veRpmCount.exitCode);
        assertEquals("16\n", veRpmCount.output);

        ProcessResult airmassBlendSize = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "read", legacyProfile.toString(), "airmass_blend_size"
        ));
        assertEquals(airmassBlendSize.output, 0, airmassBlendSize.exitCode);
        assertEquals("8\n", airmassBlendSize.output);

        for (String[] expected : new String[][] {
                {"engine_preset_mode", "default_all\n"},
                {"engine_presets", "all\n"},
                {"engine_preset_default", "MINIMAL_PINS\n"},
        }) {
            ProcessResult presetField = runCommand(Arrays.asList(
                    "bash", PROFILE_TOOL.toString(), "read", legacyProfile.toString(), expected[0]
            ));
            assertEquals(presetField.output, 0, presetField.exitCode);
            assertEquals(expected[1], presetField.output);
        }

        ProcessResult definitions = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "definitions", legacyProfile.toString()
        ));
        assertEquals(definitions.output, 0, definitions.exitCode);
        Path legacyDefinitions = directory.resolve("legacy_defines.h");
        assertEquals(legacyDefinitions.toString() + "\n", definitions.output);
        assertTableMacros(legacyDefinitions, "16", "8");
        assertTrue(read(legacyDefinitions).contains("#define CYLINDER_FUEL_TRIM_COUNT 0"));

        Path writtenProfile = directory.resolve("written.txt");
        Path fragment = directory.resolve("legacy-config.txt");
        Files.write(fragment, Arrays.asList("#define LEGACY_VALUE 1"), StandardCharsets.UTF_8);
        ProcessResult write = runCommand(Arrays.asList(
                "bash", PROFILE_TOOL.toString(), "write",
                writtenProfile.toString(), BOARD.toString(), "ARCH_STM32F4", "fan",
                "config", fragment.toString()
        ));
        assertEquals(write.output, 0, write.exitCode);
        assertEquals("0", profileValue(writtenProfile, "cylinder_fuel_trim_count"));
        assertEquals(fragment.toString(), profileValue(writtenProfile, "config"));
    }

    @Test
    public void countChangeInvalidatesObjectsAndConfigurationGeneration() throws Exception {
        Path directory = temporaryFolder.newFolder("invalidation").toPath();
        Path profile = directory.resolve("module_profile.txt");
        Path object = directory.resolve("object.o");
        Path configStamp = directory.resolve("config.stamp");
        Path makefile = directory.resolve("profile-deps.mk");
        Path buildDirectory = directory.resolve("build");
        Path generatedDirectory = directory.resolve("generated");

        Files.createDirectories(buildDirectory);
        Files.createDirectories(generatedDirectory);
        Files.write(makefile, Arrays.asList(
                "PROJECT_DIR := " + FIRMWARE,
                "BOARD_DIR := " + BOARD,
                "include $(PROJECT_DIR)/controllers/modules/module_profile.mk",
                "include $(PROJECT_DIR)/fome_generated.mk",
                "all: $(OBJS)",
                "$(OBJS):",
                "\t@touch $@"
        ), StandardCharsets.UTF_8);

        ProcessResult first = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan cylinder_fuel_trim", "4", true);
        assertEquals(first.output, 0, first.exitCode);
        assertTrue(Files.isRegularFile(object));
        assertTrue(Files.isRegularFile(configStamp));
        Path generatedIni = directory.resolve("cylinder_fuel_trim.ini");
        assertTrue(Files.isRegularFile(generatedIni));
        assertTrue(read(generatedIni).contains("cylinderFuelTrimTbl4"));
        assertFalse(read(generatedIni).contains("cylinderFuelTrimTbl5"));

        Files.setLastModifiedTime(object, FileTime.fromMillis(1000));
        Files.setLastModifiedTime(configStamp, FileTime.fromMillis(1000));

        ProcessResult second = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan cylinder_fuel_trim", "6", true);
        assertEquals(second.output, 0, second.exitCode);
        assertEquals("6", profileValue(profile, "cylinder_fuel_trim_count"));
        assertTrue(read(generatedIni).contains("cylinderFuelTrimTbl6"));
        assertFalse(read(generatedIni).contains("cylinderFuelTrimTbl7"));
        assertTrue(read(profile).contains("ini " + generatedIni + "\n"));
        assertTrue("profile count change did not invalidate stale object", Files.getLastModifiedTime(object).toMillis() > 1000);

        ProcessResult dryRun = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan cylinder_fuel_trim", "6", false);
        assertEquals(dryRun.output, 0, dryRun.exitCode);
        assertTrue("profile count change did not invalidate configuration generation: " + dryRun.output,
                dryRun.output.contains("gen_config_board.sh"));

        ProcessResult off = runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                "fan", "6", true);
        assertEquals(off.output, 0, off.exitCode);
        assertEquals("0", profileValue(profile, "cylinder_fuel_trim_count"));
        assertFalse(read(profile).contains("cylinder_fuel_trim.ini"));
        assertTrue(Files.isRegularFile(generatedIni));
        assertEquals("", read(generatedIni));
    }

    @Test
    public void moduleValidationRejectsUnknownAndPatternValues() throws Exception {
        Path directory = temporaryFolder.newFolder("invalid-modules").toPath();
        ProcessResult validMixed = runProfileMake(
                directory.resolve("valid-mixed.txt"), "fan fuel_pump", null);
        assertEquals(validMixed.output, 0, validMixed.exitCode);

        ProcessResult multiword = runProfileMake(
                directory.resolve("multi.txt"), "fan bogus fuel_pump", null);
        assertNotEquals(multiword.output, 0, multiword.exitCode);
        assertTrue(multiword.output, multiword.output.contains("bogus"));

        ProcessResult pattern = runProfileMake(
                directory.resolve("pattern.txt"), "%", null);
        assertNotEquals(pattern.output, 0, pattern.exitCode);
        assertTrue(pattern.output, pattern.output.contains("Expected modules from"));
    }

    private void assertRejected(String count, String expectedError) throws Exception {
        Path directory = temporaryFolder.newFolder("reject-" + String.valueOf(count).replace('.', '-')).toPath();
        ProcessResult result = runProfileMake(
                directory.resolve("module_profile.txt"),
                "cylinder_fuel_trim",
                count);

        assertNotEquals(result.output, 0, result.exitCode);
        assertTrue(result.output, result.output.contains("FOME_CYLINDER_TRIM_COUNT"));
        assertTrue(result.output, result.output.contains(expectedError));
    }

    private void assertDimensionRejected(String veRpmCount, String airmassBlendSize, String expectedOption)
            throws Exception {
        Path directory = temporaryFolder.newFolder().toPath();
        ProcessResult result = runProfileMake(
                directory.resolve("module_profile.txt"),
                "fan",
                null,
                veRpmCount,
                airmassBlendSize);

        assertNotEquals(result.output, 0, result.exitCode);
        assertTrue(result.output, result.output.contains(expectedOption));
    }

    private static void assertTableDimensions(
            Path profile, Path definitions, String veRpmCount, String airmassBlendSize) throws Exception {
        assertEquals(veRpmCount, profileValue(profile, "ve_rpm_count"));
        assertEquals(airmassBlendSize, profileValue(profile, "airmass_blend_size"));
        assertTableMacros(definitions, veRpmCount, airmassBlendSize);
    }

    private static void assertTableMacros(Path definitions, String veRpmCount, String airmassBlendSize)
            throws Exception {
        String header = read(definitions);
        assertTrue(header.contains("#define FOME_VE_RPM_COUNT " + veRpmCount + "\n"));
        assertTrue(header.contains("#define VE_RPM_COUNT " + veRpmCount + "\n"));
        assertTrue(header.contains("#define ALPHA_N_RPM_COUNT " + veRpmCount + "\n"));
        assertTrue(header.contains("#define FOME_AIRMASS_BLEND_SIZE " + airmassBlendSize + "\n"));
        assertTrue(header.contains("#define AIRMASS_BLEND_LOAD_COUNT " + airmassBlendSize + "\n"));
        assertTrue(header.contains("#define AIRMASS_BLEND_RPM_COUNT " + airmassBlendSize + "\n"));
    }

    private ProcessResult runProfileMake(Path profile, String modules, String count) throws Exception {
        return runProfileMake(profile, modules, count, null, null);
    }

    private ProcessResult runProfileMake(
            Path profile, String modules, String count, String veRpmCount, String airmassBlendSize)
            throws Exception {
        return runProfileMake(
                profile, modules, count, veRpmCount, airmassBlendSize, null);
    }

    private ProcessResult runProfileMake(
            Path profile,
            String modules,
            String count,
            String veRpmCount,
            String airmassBlendSize,
            String enginePresets) throws Exception {
        List<String> command = new ArrayList<>(Arrays.asList(
                "make", "-s", "--no-print-directory",
                "-f", PROFILE_MAKE.toString(),
                "module-profile",
                "PROJECT_DIR=" + FIRMWARE,
                "BOARD_DIR=" + BOARD,
                "MODULE_PROFILE_FILE=" + profile
        ));
        if (modules != null) {
            command.add("FOME_MODULES=" + modules);
        }
        if (count != null) {
            command.add("FOME_CYLINDER_TRIM_COUNT=" + count);
        }
        if (veRpmCount != null) {
            command.add("FOME_VE_RPM_COUNT=" + veRpmCount);
        }
        if (airmassBlendSize != null) {
            command.add("FOME_AIRMASS_BLEND_SIZE=" + airmassBlendSize);
        }
        if (enginePresets != null) {
            command.add("FOME_ENGINE_PRESETS=" + enginePresets);
        }
        return runCommand(command);
    }

    private ProcessResult runDependencyMake(
            Path makefile,
            Path profile,
            Path object,
            Path configStamp,
            Path buildDirectory,
            Path generatedDirectory,
            String modules,
            String count,
            boolean assumeConfigCurrent
    ) throws Exception {
        return runDependencyMake(
                makefile, profile, object, configStamp, buildDirectory, generatedDirectory,
                modules, count, null, null, assumeConfigCurrent);
    }

    private ProcessResult runDependencyMake(
            Path makefile,
            Path profile,
            Path object,
            Path configStamp,
            Path buildDirectory,
            Path generatedDirectory,
            String modules,
            String count,
            String veRpmCount,
            String airmassBlendSize,
            boolean assumeConfigCurrent
    ) throws Exception {
        Files.createDirectories(profile.getParent());
        Files.createDirectories(object.getParent());
        Files.createDirectories(configStamp.getParent());
        Files.createDirectories(buildDirectory);
        Files.createDirectories(generatedDirectory);
        if (assumeConfigCurrent && !Files.exists(configStamp)) {
            Files.write(configStamp, new byte[0]);
        }

        List<String> command = new ArrayList<>(Arrays.asList(
                "make", "-s", "--no-print-directory",
                "-f", makefile.toString()
        ));
        if (assumeConfigCurrent) {
            command.add("-o");
            command.add(configStamp.toString());
        } else {
            command.add("-n");
        }
        command.addAll(Arrays.asList(
                "all",
                "module-profile",
                "PROJECT_DIR=" + FIRMWARE,
                "BOARD_DIR=" + BOARD,
                "PROJECT_CPU=ARCH_STM32F4",
                "BUILDDIR=" + buildDirectory,
                "GENERATED_DIR=" + generatedDirectory,
                "SHORT_BOARD_NAME=profile-test",
                "OBJS=" + object,
                "PCHOBJ=" + buildDirectory.resolve("pch.stamp"),
                "CONFIG_STAMP=" + configStamp,
                "LIVE_DATA_STAMP=" + buildDirectory.resolve("live.stamp"),
                "JAVA_TOOLS=" + buildDirectory.resolve("java.cookie"),
                "MODULE_PROFILE_FILE=" + profile,
                "FOME_MODULES=" + modules,
                "FOME_CYLINDER_TRIM_COUNT=" + count
        ));
        if (veRpmCount != null) {
            command.add("FOME_VE_RPM_COUNT=" + veRpmCount);
        }
        if (airmassBlendSize != null) {
            command.add("FOME_AIRMASS_BLEND_SIZE=" + airmassBlendSize);
        }
        return runCommand(command);
    }

    private ProcessResult runCommand(List<String> command) throws Exception {
        ProcessBuilder processBuilder = new ProcessBuilder(command);
        processBuilder.directory(FIRMWARE.toFile());
        processBuilder.redirectErrorStream(true);
        processBuilder.environment().remove("FOME_MODULES");
        processBuilder.environment().remove("FOME_CYLINDER_TRIM_COUNT");
        processBuilder.environment().remove("FOME_VE_RPM_COUNT");
        processBuilder.environment().remove("FOME_AIRMASS_BLEND_SIZE");
        processBuilder.environment().remove("FOME_ENGINE_PRESETS");

        Process process = processBuilder.start();
        StringBuilder output = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                output.append(line).append('\n');
            }
        }
        return new ProcessResult(process.waitFor(), output.toString());
    }

    private static String profileValue(Path profile, String field) throws IOException {
        for (String line : Files.readAllLines(profile, StandardCharsets.UTF_8)) {
            if (line.startsWith(field + " ")) {
                return line.substring(field.length() + 1);
            }
        }
        throw new AssertionError("Missing profile field " + field + " in " + profile);
    }

    private static String read(Path path) throws IOException {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
    }

    private static class ProcessResult {
        private final int exitCode;
        private final String output;

        private ProcessResult(int exitCode, String output) {
            this.exitCode = exitCode;
            this.output = output;
        }
    }
}
