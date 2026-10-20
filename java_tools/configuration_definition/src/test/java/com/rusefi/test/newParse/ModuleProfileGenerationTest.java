package com.rusefi.test.newParse;

import com.rusefi.RusefiParseErrorStrategy;
import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.outputs.CStructWriter;
import com.rusefi.newparse.outputs.OutputChannelWriter;
import com.rusefi.newparse.outputs.PrintStreamAlwaysUnix;
import com.rusefi.newparse.outputs.TsWriter;
import com.rusefi.newparse.parsing.Definition;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ModuleProfileGenerationTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void generatedCountDrivesConfigLiveDataAndIniDefinitions() throws Exception {
        Path definitions = temporaryFolder.newFile("module_profile_defines.h").toPath();
        Files.write(definitions, Arrays.asList(
                "! generated test definitions",
                "#define CYLINDER_FUEL_TRIM_COUNT 4"
        ), StandardCharsets.UTF_8);

        String schemaDefinition =
                "struct_no_prefix total\n" +
                        "float[CYLINDER_FUEL_TRIM_COUNT iterate] fuelTrim;;\"percent\", 1, 0, 0, 0, 2\n" +
                        "end_struct\n";
        ParseState configState = parseWithDefinitions(definitions, schemaDefinition);

        ByteArrayOutputStream configOutput = new ByteArrayOutputStream();
        PrintStream configPrintStream = new PrintStreamAlwaysUnix(
                configOutput, true, StandardCharsets.UTF_8.name());
        new CStructWriter().writeCStructs(configState, configPrintStream);
        String generatedConfig = configOutput.toString(StandardCharsets.UTF_8.name());
        assertTrue(generatedConfig, generatedConfig.contains("float fuelTrim[4];"));

        ParseState liveState = parseWithDefinitions(definitions, schemaDefinition);
        ByteArrayOutputStream liveOutput = new ByteArrayOutputStream();
        ByteArrayOutputStream dataLogOutput = new ByteArrayOutputStream();
        new OutputChannelWriter(
                new PrintStreamAlwaysUnix(liveOutput, true, StandardCharsets.UTF_8.name()),
                new PrintStreamAlwaysUnix(dataLogOutput, true, StandardCharsets.UTF_8.name())
        ).writeOutputChannels(liveState, "Cylinder trim");
        String generatedLiveData = liveOutput.toString(StandardCharsets.UTF_8.name());
        assertTrue(generatedLiveData, generatedLiveData.contains("fuelTrim1"));
        assertTrue(generatedLiveData, generatedLiveData.contains("fuelTrim4"));
        assertFalse(generatedLiveData, generatedLiveData.contains("fuelTrim5"));

        Path template = temporaryFolder.newFile("template.ini").toPath();
        Path ini = temporaryFolder.newFile("generated.ini").toPath();
        Files.write(template, Arrays.asList(
                "[Constants]",
                "cylinderFuelTrimCount = @@CYLINDER_FUEL_TRIM_COUNT@@"
        ), StandardCharsets.UTF_8);
        new TsWriter().writeTunerstudio(
                parseWithDefinitions(definitions, schemaDefinition), template.toString(), ini.toString());
        String generatedIni = new String(Files.readAllBytes(ini), StandardCharsets.UTF_8);
        assertTrue(generatedIni, generatedIni.contains("cylinderFuelTrimCount = 4"));
    }

    private static ParseState parseWithDefinitions(Path definitions, String input) throws Exception {
        ParseState parseState = new ParseState();
        parseState.setDefinitionPolicy(Definition.OverwritePolicy.NotAllowed);
        RusefiParseErrorStrategy.parseDefinitionFile(parseState.getListener(), definitions.toString());
        RusefiParseErrorStrategy.parseDefinitionString(parseState.getListener(), input);
        return parseState;
    }
}
