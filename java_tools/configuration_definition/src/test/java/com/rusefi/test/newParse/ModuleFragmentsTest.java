package com.rusefi.test.newParse;

import com.rusefi.newparse.outputs.ModuleFragments;
import com.rusefi.newparse.outputs.PrintStreamAlwaysUnix;
import com.rusefi.newparse.outputs.TsWriter;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class ModuleFragmentsTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void noFragmentsKeepLegacyTextButRemoveOptionalMarker() throws Exception {
        String legacy = "struct_no_prefix config\nint32_t value\nend_struct\n";
        File legacyFile = write("legacy.txt", legacy);
        assertEquals(legacy, ModuleFragments.composeConfig(legacyFile.getPath(), Collections.emptyList()));

        File markedFile = write("marked.txt",
                "struct_no_prefix config\nint32_t value\n  ! MODULE_CONFIG  \nend_struct\n");
        assertEquals(legacy, ModuleFragments.composeConfig(markedFile.getPath(), Collections.emptyList()));
    }

    @Test
    public void configFragmentsProduceSelectedCompactLayout() throws Exception {
        File core = write("config.txt",
                "struct_no_prefix config\n" +
                        "int32_t before\n" +
                        "! MODULE_CONFIG\n" +
                        "int16_t after\n" +
                        "end_struct\n");
        File first = write("first-config.txt", "int32_t selectedA\n");
        File second = write("second-config.txt", "int16_t selectedB\n");
        write("unselected-config.txt", "int32_t unselectedModuleField\n");

        String composed = ModuleFragments.composeConfig(core.getPath(), Arrays.asList(first.getPath(), second.getPath()));
        String layout = NewParseHelper.parseToTs(composed);

        assertTrue(layout, layout.contains("before = scalar, S32, 0,"));
        assertTrue(layout, layout.contains("selectedA = scalar, S32, 4,"));
        assertTrue(layout, layout.contains("selectedB = scalar, S16, 8,"));
        assertTrue(layout, layout.contains("after = scalar, S16, 10,"));
        assertTrue(layout, layout.contains("pageSize            = 12"));
        assertFalse(layout, layout.contains("unselectedModuleField"));
    }

    @Test
    public void configFragmentOrderIsTheSuppliedOrder() throws Exception {
        File core = write("ordered-config.txt",
                "struct_no_prefix config\n" +
                        "! MODULE_CONFIG\n" +
                        "end_struct\n");
        File first = write("a-config.txt", "int8_t firstModule\n");
        File second = write("b-config.txt", "int8_t secondModule\n");

        String forward = NewParseHelper.parseToTs(
                ModuleFragments.composeConfig(core.getPath(), Arrays.asList(first.getPath(), second.getPath())));
        String reverse = NewParseHelper.parseToTs(
                ModuleFragments.composeConfig(core.getPath(), Arrays.asList(second.getPath(), first.getPath())));

        assertTrue(forward.indexOf("firstModule") < forward.indexOf("secondModule"));
        assertTrue(reverse.indexOf("secondModule") < reverse.indexOf("firstModule"));
    }

    @Test
    public void persistentConfigFragmentAppendsToRootWithoutDuplicatingActiveConfig() throws Exception {
        File core = write("persistent-config.txt",
                "struct_no_prefix persistent_config_s\n" +
                        "struct_no_prefix engine_configuration_s\n" +
                        "int32_t activeBefore\n" +
                        "! MODULE_CONFIG\n" +
                        "int32_t activeAfter\n" +
                        "end_struct\n" +
                        "engine_configuration_s engineConfiguration;\n" +
                        "int32_t rootAfter\n" +
                        "! MODULE_PERSISTENT_CONFIG\n" +
                        "end_struct\n");
        File active = write("active-fragment.txt", "int16_t activeModule\n");
        File persistent = write("persistent-fragment.txt", "int32_t persistentModuleTable\n");

        String layout = NewParseHelper.parseToTs(ModuleFragments.composeConfig(
                core.getPath(), Collections.singletonList(active.getPath()),
                Collections.singletonList(persistent.getPath())));

        assertTrue(layout, layout.indexOf("activeBefore") < layout.indexOf("activeModule"));
        assertTrue(layout, layout.indexOf("activeModule") < layout.indexOf("activeAfter"));
        assertTrue(layout, layout.indexOf("activeAfter") < layout.indexOf("rootAfter"));
        assertTrue(layout, layout.indexOf("rootAfter") < layout.indexOf("persistentModuleTable"));
        assertTrue(layout, layout.contains("activeBefore = scalar, S32, 0,"));
        assertTrue(layout, layout.contains("activeModule = scalar, S16, 4,"));
        assertTrue(layout, layout.contains("activeAfter = scalar, S32, 8,"));
        assertTrue(layout, layout.contains("rootAfter = scalar, S32, 12,"));
        assertTrue(layout, layout.contains("persistentModuleTable = scalar, S32, 16,"));
        assertTrue(layout, layout.contains("pageSize            = 20"));
        assertEquals(1, occurrences(layout, "persistentModuleTable"));

        String activeOnlyLayout = NewParseHelper.parseToTs(ModuleFragments.composeConfig(
                core.getPath(), Collections.singletonList(active.getPath()), Collections.emptyList()));

        assertFalse(activeOnlyLayout, activeOnlyLayout.contains("persistentModuleTable"));
        assertTrue(activeOnlyLayout, activeOnlyLayout.contains("activeBefore = scalar, S32, 0,"));
        assertTrue(activeOnlyLayout, activeOnlyLayout.contains("activeModule = scalar, S16, 4,"));
        assertTrue(activeOnlyLayout, activeOnlyLayout.contains("activeAfter = scalar, S32, 8,"));
        assertTrue(activeOnlyLayout, activeOnlyLayout.contains("pageSize            = 16"));
    }

    @Test
    public void twoArgumentConfigCompositionKeepsLegacyActiveFragmentApi() throws Exception {
        File core = write("two-argument-config.txt",
                "struct_no_prefix persistent_config_s\n" +
                        "struct_no_prefix engine_configuration_s\n" +
                        "! MODULE_CONFIG\n" +
                        "end_struct\n" +
                        "! MODULE_PERSISTENT_CONFIG\n" +
                        "end_struct\n");
        File active = write("two-argument-fragment.txt", "int32_t activeModule\n");

        String composed = ModuleFragments.composeConfig(core.getPath(),
                Collections.singletonList(active.getPath()));

        assertTrue(composed, composed.contains("int32_t activeModule\nend_struct\n"));
        assertFalse(composed, composed.contains(ModuleFragments.PERSISTENT_CONFIG_ANCHOR));
    }

    @Test
    public void outputFragmentsUseTheOutputAnchorOnly() throws Exception {
        File core = write("output_channels.txt",
                "struct_no_prefix total\n" +
                        "int32_t before\n" +
                        "! MODULE_OUTPUTS\n" +
                        "int16_t after\n" +
                        "end_struct\n");
        File first = write("first-output.txt", "int16_t selectedOutputA\n");
        File second = write("second-output.txt", "int32_t selectedOutputB\n");
        write("unselected-output.txt", "int32_t unselectedOutput\n");

        String layout = NewParseHelper.parseToOutputChannels(
                ModuleFragments.composeOutput(core.getPath(), Arrays.asList(first.getPath(), second.getPath())));

        assertTrue(layout, layout.contains("before = scalar, S32, 0,"));
        assertTrue(layout, layout.contains("selectedOutputA = scalar, S16, 4,"));
        assertTrue(layout, layout.contains("selectedOutputB = scalar, S32, 8,"));
        assertTrue(layout, layout.contains("after = scalar, S16, 12,"));
        assertTrue(layout, layout.contains("; total TS size = 16"));
        assertFalse(layout, layout.contains("unselectedOutput"));
    }

    @Test
    public void missingAndDuplicateAnchorsFail() throws Exception {
        File fragment = write("fragment.txt", "int32_t moduleField\n");
        File missing = write("missing-anchor.txt", "struct_no_prefix config\nend_struct\n");
        File duplicate = write("duplicate-anchor.txt",
                "struct_no_prefix config\n! MODULE_CONFIG\n! MODULE_CONFIG\nend_struct\n");

        assertComposeFails(
                () -> ModuleFragments.composeConfig(missing.getPath(), Collections.singletonList(fragment.getPath())),
                "Missing '! MODULE_CONFIG' anchor",
                missing.getPath());
        assertComposeFails(
                () -> ModuleFragments.composeConfig(duplicate.getPath(), Collections.singletonList(fragment.getPath())),
                "Duplicate '! MODULE_CONFIG' anchor",
                duplicate.getPath());

        File missingOutput = write("missing-output-anchor.txt", "struct_no_prefix total\nend_struct\n");
        assertComposeFails(
                () -> ModuleFragments.composeOutput(missingOutput.getPath(),
                        Collections.singletonList(fragment.getPath())),
                "Missing '! MODULE_OUTPUTS' anchor",
                missingOutput.getPath());

        File persistentFragment = write("persistent-fragment.txt", "int32_t persistentField\n");
        File missingPersistent = write("missing-persistent-anchor.txt",
                "struct_no_prefix persistent_config_s\n! MODULE_CONFIG\nend_struct\n");
        assertComposeFails(
                () -> ModuleFragments.composeConfig(missingPersistent.getPath(), Collections.emptyList(),
                        Collections.singletonList(persistentFragment.getPath())),
                "Missing '! MODULE_PERSISTENT_CONFIG' anchor",
                missingPersistent.getPath());

        File duplicatePersistent = write("duplicate-persistent-anchor.txt",
                "struct_no_prefix persistent_config_s\n" +
                        "! MODULE_PERSISTENT_CONFIG\n" +
                        "! MODULE_PERSISTENT_CONFIG\n" +
                        "end_struct\n");
        assertComposeFails(
                () -> ModuleFragments.composeConfig(duplicatePersistent.getPath(), Collections.emptyList(),
                        Collections.singletonList(persistentFragment.getPath())),
                "Duplicate '! MODULE_PERSISTENT_CONFIG' anchor",
                duplicatePersistent.getPath());
    }

    @Test
    public void iniSectionsLandAtTheirContextsAndKeepWriterProcessing() throws Exception {
        File template = write("tunerstudio.template.ini",
                "[CurveEditor]\n" +
                        "; MODULE_INI CurveEditor\n" +
                        "[GaugeConfigurations]\n" +
                        "; MODULE_INI GaugeConfigurations\n" +
                        "[FrontPage]\n" +
                        "; MODULE_INI FrontPage\n" +
                        "[Menu]\n" +
                        "; MODULE_INI Menu\n" +
                        "[ControllerCommands]\n" +
                        "; MODULE_INI ControllerCommands\n" +
                        "[UserDefined]\n" +
                        "; MODULE_INI UserDefined\n");
        File first = write("first.ini",
                "[CurveEditor]\n" +
                        "curve = @@module_name@@\n" +
                        "#if USE_FAHRENHEIT\n" +
                        "curveTemperature = \"F\"\n" +
                        "#else\n" +
                        "curveTemperature = \"C\"\n" +
                        "#endif\n" +
                        "moduleVisible = 1 @@if_show_module\n" +
                        "moduleHidden = 1 @@if_show_missing\n" +
                        "[GaugeConfigurations]\n" +
                        "gauge = moduleGauge\n" +
                        "[FrontPage]\n" +
                        "indicator = moduleIndicator\n" +
                        "[Menu]\n" +
                        "menu = firstModuleMenu\n" +
                        "[ControllerCommands]\n" +
                        "command = moduleCommand\n" +
                        "[UserDefined]\n" +
                        "dialog = moduleDialog\n");
        File second = write("second.ini",
                "[Menu]\n" +
                        "menu = secondModuleMenu\n");

        ByteArrayOutputStream output = new ByteArrayOutputStream();
        PrintStream printStream = new PrintStreamAlwaysUnix(output, true, StandardCharsets.UTF_8.name());
        new TsWriter().writeTunerstudio(
                NewParseHelper.parse("#define module_name 42\n#define show_module true\n"),
                template.getPath(), printStream, Arrays.asList(first.getPath(), second.getPath()));
        String result = output.toString(StandardCharsets.UTF_8.name());

        assertFalse(result, result.contains("; MODULE_INI"));
        assertTrue(result, result.contains("[CurveEditor]\ncurve = 42\n"));
        assertTrue(result, result.contains("#if USE_FAHRENHEIT\ncurveTemperature = \"F\"\n#else\n"
                + "curveTemperature = \"C\"\n#endif\n"));
        assertTrue(result, result.contains("moduleVisible = 1 \n"));
        assertFalse(result, result.contains("moduleHidden"));
        assertTrue(result, result.contains("[GaugeConfigurations]\ngauge = moduleGauge\n"));
        assertTrue(result, result.contains("[FrontPage]\nindicator = moduleIndicator\n"));
        assertTrue(result, result.contains("[Menu]\nmenu = firstModuleMenu\nmenu = secondModuleMenu\n"));
        assertTrue(result, result.contains("[ControllerCommands]\ncommand = moduleCommand\n"));
        assertTrue(result, result.contains("[UserDefined]\ndialog = moduleDialog\n"));
    }

    @Test
    public void optionalLinesRequireEveryIndependentCondition() throws Exception {
        File template = write("multiple-conditions.ini",
                "[UserDefined]\n" +
                        "both = 1@@if_show_family@@if_show_preset\n" +
                        "familyOnly = 1@@if_show_family\n" +
                        "missingPreset = 1@@if_show_family@@if_show_missing\n" +
                        "falsePreset = 1@@if_show_family@@if_show_disabled\n" +
                        "presetOnly = 1@@if_show_preset\n");

        ByteArrayOutputStream output = new ByteArrayOutputStream();
        PrintStream printStream = new PrintStreamAlwaysUnix(output, true, StandardCharsets.UTF_8.name());
        new TsWriter().writeTunerstudio(
                NewParseHelper.parse(
                        "#define show_family true\n" +
                                "#define show_preset true\n" +
                                "#define show_disabled false\n"),
                template.getPath(), printStream);
        String result = output.toString(StandardCharsets.UTF_8.name());

        assertTrue(result, result.contains("both = 1\n"));
        assertTrue(result, result.contains("familyOnly = 1\n"));
        assertTrue(result, result.contains("presetOnly = 1\n"));
        assertFalse(result, result.contains("missingPreset"));
        assertFalse(result, result.contains("falsePreset"));
        assertFalse(result, result.contains("@@if_"));
    }

    @Test
    public void namedIniContextsAreFixedKeysAndStayInsideTheirTemplateSections() throws Exception {
        File template = write("named-contexts.ini",
                "[PcVariables]\n" +
                        "; MODULE_INI PcVariables\n" +
                        "[ConstantsExtensions]\n" +
                        "; MODULE_INI ConstantsExtensions\n" +
                        "[TableEditor]\n" +
                        "; MODULE_INI TableEditor\n" +
                        "[Menu]\n" +
                        "menu = existingMenu\n" +
                        "\t; MODULE_INI Menu.Advanced\n" +
                        "\tsubMenu = existingBoost\n" +
                        "[UserDefined]\n" +
                        "dialog = monitoringSettings\n" +
                        "\t; MODULE_INI UserDefined.SnifferThreshold\n" +
                        "\tfield = existingThreshold\n");
        File fragment = write("named-contexts-fragment.ini",
                "[PcVariables]\n" +
                        "pcVariable = moduleValue\n" +
                        "[ConstantsExtensions]\n" +
                        "maintainConstantValue = moduleValue, { 1 }\n" +
                        "[TableEditor]\n" +
                        "table = moduleTable\n" +
                        "[Menu.Advanced]\n" +
                        "\tsubMenu = moduleAdvanced\n" +
                        "[UserDefined.SnifferThreshold]\n" +
                        "\tfield = moduleThreshold\n");

        String result = String.join("\n", ModuleFragments.composeIniTemplate(
                template.getPath(), Collections.singletonList(fragment.getPath())));

        assertFalse(result, result.contains("; MODULE_INI"));
        assertFalse(result, result.contains("[Menu.Advanced]"));
        assertFalse(result, result.contains("[UserDefined.SnifferThreshold]"));
        assertTrue(result, result.contains("[PcVariables]\npcVariable = moduleValue\n"));
        assertTrue(result, result.contains("[ConstantsExtensions]\n"
                + "maintainConstantValue = moduleValue, { 1 }\n"));
        assertTrue(result, result.indexOf("[TableEditor]\ntable = moduleTable\n") >= 0);
        assertTrue(result, result.indexOf("menu = existingMenu\n\tsubMenu = moduleAdvanced\n")
                < result.indexOf("\tsubMenu = existingBoost"));
        assertTrue(result, result.contains("dialog = monitoringSettings\n\tfield = moduleThreshold\n"));
        assertTrue(result, result.indexOf("field = moduleThreshold") < result.indexOf("field = existingThreshold"));
    }

    @Test
    public void namedIniContextValidationRejectsMissingWrongDuplicateAndMalformedKeys() throws Exception {
        File menuFragment = write("menu-fragment.ini", "[Menu.Advanced]\nsubMenu = moduleAdvanced\n");
        File missing = write("missing-named-marker.ini", "[Menu]\nmenu = existingMenu\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(missing.getPath(),
                        Collections.singletonList(menuFragment.getPath())),
                "Missing '; MODULE_INI Menu.Advanced' marker",
                missing.getPath());

        File wrongScope = write("wrong-named-scope.ini",
                "[UserDefined]\n" +
                        "; MODULE_INI Menu.Advanced\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(wrongScope.getPath(),
                        Collections.singletonList(menuFragment.getPath())),
                "must appear inside [Menu]",
                wrongScope.getPath());

        File duplicate = write("duplicate-named-marker.ini",
                "[Menu]\n" +
                        "; MODULE_INI Menu.Advanced\n" +
                        "; MODULE_INI Menu.Advanced\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(menuFragment.getPath())),
                "Duplicate '; MODULE_INI Menu.Advanced' marker",
                duplicate.getPath());

        File malformedKey = write("malformed-key.ini",
                "[Menu.Advanced.More]\n" +
                        "subMenu = moduleAdvanced\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(malformedKey.getPath())),
                "Invalid TunerStudio composition key [Menu.Advanced.More]",
                malformedKey.getPath());

        File duplicateKey = write("duplicate-key.ini",
                "[Menu.Advanced]\n" +
                        "subMenu = first\n" +
                        "[Menu.Advanced]\n" +
                        "subMenu = second\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(duplicateKey.getPath())),
                "Duplicate [Menu.Advanced] section",
                duplicateKey.getPath());
    }

    @Test
    public void iniFragmentValidationNamesTheBadFile() throws Exception {
        File fragment = write("bad.ini",
                "[GaugeConfigurations]\n" +
                        "gauge = moduleGauge\n");
        File missing = write("missing-marker.ini",
                "[CurveEditor]\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(missing.getPath(),
                        Collections.singletonList(fragment.getPath())),
                "Missing '; MODULE_INI GaugeConfigurations' marker",
                missing.getPath());

        File duplicate = write("duplicate-marker.ini",
                "[GaugeConfigurations]\n" +
                        "; MODULE_INI GaugeConfigurations\n" +
                        "; MODULE_INI GaugeConfigurations\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(fragment.getPath())),
                "Duplicate '; MODULE_INI GaugeConfigurations' marker",
                duplicate.getPath());

        File unknown = write("unknown.ini", "[NotAContext]\nvalue = 1\n");
        File knownMarker = write("known-marker.ini",
                "[CurveEditor]\n" +
                        "; MODULE_INI CurveEditor\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(knownMarker.getPath(),
                        Collections.singletonList(unknown.getPath())),
                "Missing '; MODULE_INI NotAContext' marker",
                unknown.getPath());

        File preface = write("preface.ini", "value = beforeSection\n[Menu]\nmenu = item\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(preface.getPath())),
                "Text before first section",
                preface.getPath());
    }

    private File write(String name, String content) throws Exception {
        File file = temporaryFolder.newFile(name);
        Files.write(file.toPath(), content.getBytes(StandardCharsets.UTF_8));
        return file;
    }

    private static void assertComposeFails(ThrowingRunnable action, String expectedMessage, String expectedPath) {
        try {
            action.run();
            fail("Expected composition to fail");
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(expectedMessage));
            assertTrue(e.getMessage(), e.getMessage().contains(expectedPath));
        } catch (Exception e) {
            throw new AssertionError("Unexpected exception", e);
        }
    }

    private static int occurrences(String text, String value) {
        int count = 0;
        int offset = 0;
        while ((offset = text.indexOf(value, offset)) >= 0) {
            count++;
            offset += value.length();
        }
        return count;
    }

    private interface ThrowingRunnable {
        void run() throws Exception;
    }
}
