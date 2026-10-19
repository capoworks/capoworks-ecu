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
                "; MODULE_INI GaugeConfigurations\n" +
                        "; MODULE_INI GaugeConfigurations\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(fragment.getPath())),
                "Duplicate '; MODULE_INI GaugeConfigurations' marker",
                duplicate.getPath());

        File unknown = write("unknown.ini", "[NotAContext]\nvalue = 1\n");
        assertComposeFails(
                () -> ModuleFragments.composeIniTemplate(duplicate.getPath(),
                        Collections.singletonList(unknown.getPath())),
                "Unknown TunerStudio section [NotAContext]",
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

    private interface ThrowingRunnable {
        void run() throws Exception;
    }
}
