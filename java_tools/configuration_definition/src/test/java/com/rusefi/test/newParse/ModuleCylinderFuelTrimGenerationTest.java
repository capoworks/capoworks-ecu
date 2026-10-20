package com.rusefi.test.newParse;

import com.rusefi.RusefiParseErrorStrategy;
import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.outputs.CStructWriter;
import com.rusefi.newparse.outputs.ModuleFragments;
import com.rusefi.newparse.outputs.OutputChannelWriter;
import com.rusefi.newparse.outputs.PersistentLayoutIdentity;
import com.rusefi.newparse.outputs.PrintStreamAlwaysUnix;
import com.rusefi.newparse.parsing.Definition;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

public class ModuleCylinderFuelTrimGenerationTest {
    private static final Path FIRMWARE = Paths.get("../../firmware").toAbsolutePath().normalize();
    private static final Path MODULE = FIRMWARE.resolve("controllers/modules/cylinder_fuel_trim");
    private static final Path RENDERER = MODULE.resolve("render_tunerstudio.sh");
    private static final Path UI_TEMPLATE = MODULE.resolve("tunerstudio.template.ini");
    private static final Path PERSISTENT_FRAGMENT = MODULE.resolve("persistent_config.txt");
    private static final Path OUTPUT_FRAGMENT = MODULE.resolve("output_channels.txt");

    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void persistentLayoutOwnsExactCountAndDisappearsWhenOff() throws Exception {
        ParseState count4 = parsePersistent(4);
        ParseState count4Again = parsePersistent(4);
        ParseState count6 = parsePersistent(6);
        ParseState off = parsePersistent(0);

        String count4Id = PersistentLayoutIdentity.layoutId(count4);
        assertEquals(count4Id, PersistentLayoutIdentity.layoutId(count4Again));
        assertNotEquals(count4Id, PersistentLayoutIdentity.layoutId(count6));

        String count4Descriptor = PersistentLayoutIdentity.canonicalDescriptor(count4);
        assertTrue(count4Descriptor, count4Descriptor.contains("persistent_config_s.cylinderFuelTrims.table|"));
        assertTrue(count4Descriptor, count4Descriptor.contains("persistent_config_s.cylinderFuelTrimLoadSource|"));

        String count4C = writeC(count4);
        assertTrue(count4C, count4C.contains("cylinder_fuel_trim_s cylinderFuelTrims[4];"));
        assertTrue(count4C, count4C.contains("load_override_e cylinderFuelTrimLoadSource[4];"));
        assertFalse(count4C, count4C.contains("cylinderFuelTrims[5]"));

        String offDescriptor = PersistentLayoutIdentity.canonicalDescriptor(off);
        assertTrue(offDescriptor, offDescriptor.contains("persistent_config_s.coreValue|"));
        assertFalse(offDescriptor, offDescriptor.contains("cylinderFuelTrim"));
        String offC = writeC(off);
        assertTrue(offC, offC.contains("coreValue ="));
        assertFalse(offC, offC.contains("cylinderFuelTrim"));
    }

    @Test
    public void outputChannelsUseOnlyExactConfiguredCylinderCount() throws Exception {
        ParseState count4 = parseOutput(4);
        String channels = writeOutput(count4, false);
        String datalogs = writeOutput(parseOutput(4), true);

        assertTrue(channels, channels.contains("cylinderFuelTrim1"));
        assertTrue(channels, channels.contains("cylinderFuelTrim4"));
        assertTrue(channels, channels.contains("cylinderFuelTrimLoad4"));
        assertFalse(channels, channels.contains("cylinderFuelTrim5"));
        assertTrue(datalogs, datalogs.contains("cylinderFuelTrim4"));
        assertFalse(datalogs, datalogs.contains("cylinderFuelTrim5"));
        assertFalse(channels, channels.contains("sdCylinderFuelTrim"));
        assertFalse(channels, channels.contains("alphaNCylinderFuelTrim"));
    }

    @Test
    public void rendererMaterializesExactRowsForOneFourTwelveAndOff() throws Exception {
        for (int count : Arrays.asList(0, 1, 4, 12)) {
            Path output = temporaryFolder.newFile("cylinder-trim-" + count + ".ini").toPath();
            ProcessResult result = run(Arrays.asList(
                    "bash", RENDERER.toString(), Integer.toString(count),
                    UI_TEMPLATE.toString(), output.toString()));
            assertEquals(result.output, 0, result.exitCode);
            String ini = read(output);

            assertEquals(ini, count, occurrences(ini, "table = cylinderFuelTrimTbl"));
            assertEquals(ini, count, occurrences(ini, "groupChildMenu = cylinderFuelTrimTbl"));
            assertEquals(ini, count, occurrences(ini, "field = \"Cylinder "));
            assertEquals(ini, count, occurrences(ini, "cylinderFuelTrimGauge"));
            assertFalse(ini, ini.contains("@CYLINDER_FUEL_TRIM_"));
            assertFalse(ini, ini.contains("sdCylinder"));
            assertFalse(ini, ini.contains("alphaNCylinder"));
            assertFalse(ini, ini.contains("cylinderModelFuelTrim"));

            if (count == 0) {
                assertEquals("", ini);
            } else {
                assertTrue(ini, ini.contains("cylinderFuelTrimTbl" + count));
                assertFalse(ini, ini.contains("cylinderFuelTrimTbl" + (count + 1)));
                assertTrue(ini, ini.contains("cylinderFuelTrimRpmBins, RPMValue"));
                assertTrue(ini, ini.contains("cylinderFuelTrimLoadBins, cylinderFuelTrimLoad" + count));
                assertTrue(ini, ini.contains(
                        "bitStringValue(loadAxisLabels, cylinderFuelTrimLoadSource" + count));
            }
        }
    }

    private ParseState parsePersistent(int count) throws Exception {
        Path workspace = Files.createTempDirectory(temporaryFolder.getRoot().toPath(), "persistent-");
        Path core = workspace.resolve("persistent.txt");
        Files.write(core, Arrays.asList(
                "struct_no_prefix persistent_config_s",
                "int32_t coreValue",
                "! MODULE_PERSISTENT_CONFIG",
                "end_struct"
        ), StandardCharsets.UTF_8);
        List<String> fragments = count == 0
                ? Collections.emptyList()
                : Collections.singletonList(PERSISTENT_FRAGMENT.toString());
        return parseWithDefinitions(workspace, count,
                ModuleFragments.composeConfig(core.toString(), Collections.emptyList(), fragments));
    }

    private ParseState parseOutput(int count) throws Exception {
        Path workspace = Files.createTempDirectory(temporaryFolder.getRoot().toPath(), "output-");
        Path core = workspace.resolve("output.txt");
        Files.write(core, Arrays.asList(
                "struct_no_prefix total",
                "! MODULE_OUTPUTS",
                "end_struct"
        ), StandardCharsets.UTF_8);
        String composed = ModuleFragments.composeOutput(core.toString(),
                Collections.singletonList(OUTPUT_FRAGMENT.toString()));
        return parseWithDefinitions(workspace, count, composed);
    }

    private ParseState parseWithDefinitions(Path workspace, int count, String input) throws Exception {
        Path definitions = workspace.resolve("definitions.h");
        Files.write(definitions, Arrays.asList(
                "#define CYLINDER_FUEL_TRIM_COUNT " + count,
                "#define load_override_e_enum \"Model default\", \"Measured MAP\", \"TPS\", \"Acc Pedal\", \"Cyl Filling %\", \"Effective MAP\", \"INVALID\", \"INVALID\"",
                "custom load_override_e 1 bits, U08, @OFFSET@, [0:2], @@load_override_e_enum@@"
        ), StandardCharsets.UTF_8);
        ParseState parseState = new ParseState();
        parseState.setDefinitionPolicy(Definition.OverwritePolicy.NotAllowed);
        RusefiParseErrorStrategy.parseDefinitionFile(parseState.getListener(), definitions.toString());
        RusefiParseErrorStrategy.parseDefinitionString(parseState.getListener(), input);
        return parseState;
    }

    private static String writeC(ParseState state) throws Exception {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        new CStructWriter().writeCStructs(state,
                new PrintStreamAlwaysUnix(output, true, StandardCharsets.UTF_8.name()));
        return output.toString(StandardCharsets.UTF_8);
    }

    private static String writeOutput(ParseState state, boolean datalog) throws Exception {
        ByteArrayOutputStream channels = new ByteArrayOutputStream();
        ByteArrayOutputStream logs = new ByteArrayOutputStream();
        new OutputChannelWriter(
                new PrintStreamAlwaysUnix(channels, true, StandardCharsets.UTF_8.name()),
                new PrintStreamAlwaysUnix(logs, true, StandardCharsets.UTF_8.name())
        ).writeOutputChannels(state, null);
        return (datalog ? logs : channels).toString(StandardCharsets.UTF_8);
    }

    private ProcessResult run(List<String> command) throws Exception {
        ProcessBuilder processBuilder = new ProcessBuilder(command);
        processBuilder.directory(FIRMWARE.toFile());
        processBuilder.redirectErrorStream(true);
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

    private static String read(Path path) throws Exception {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
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

    private static class ProcessResult {
        private final int exitCode;
        private final String output;

        private ProcessResult(int exitCode, String output) {
            this.exitCode = exitCode;
            this.output = output;
        }
    }
}
