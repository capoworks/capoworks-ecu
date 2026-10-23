package com.rusefi.test.newParse;

import com.rusefi.RusefiParseErrorStrategy;
import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.outputs.CStructWriter;
import com.rusefi.newparse.outputs.JavaFieldsWriter;
import com.rusefi.newparse.outputs.OutputChannelWriter;
import com.rusefi.newparse.outputs.PrintStreamAlwaysUnix;
import com.rusefi.newparse.outputs.TsWriter;
import com.rusefi.newparse.parsing.Definition;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import javax.tools.JavaCompiler;
import javax.tools.ToolProvider;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
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

    @Test
    public void javaDefinitionsPreserveTypesForGeneratedModuleProfileValues() throws Exception {
        Path enabledJavaFile = writeJavaDefinitions("enabled_defines.h", "EnabledModule.java",
                "#define FOME_SD_CARD_ENABLED true",
                "#define MODULE_COUNT 4",
                "#define MODULE_RATIO 1.5",
                "#define MODULE_NAME \"true\"");
        Path disabledJavaFile = writeJavaDefinitions("disabled_defines.h", "DisabledModule.java",
                "#define FOME_SD_CARD_ENABLED false");

        String enabledJava = new String(Files.readAllBytes(enabledJavaFile), StandardCharsets.UTF_8);
        assertTrue(enabledJava,
                enabledJava.contains("\tpublic static final boolean FOME_SD_CARD_ENABLED = true;"));
        assertTrue(enabledJava,
                enabledJava.contains("\tpublic static final int MODULE_COUNT = 4;"));
        assertTrue(enabledJava,
                enabledJava.contains("\tpublic static final double MODULE_RATIO = 1.5;"));
        assertTrue(enabledJava,
                enabledJava.contains("\tpublic static final String MODULE_NAME = \"true\";"));

        String disabledJava = new String(Files.readAllBytes(disabledJavaFile), StandardCharsets.UTF_8);
        assertTrue(disabledJava,
                disabledJava.contains("\tpublic static final boolean FOME_SD_CARD_ENABLED = false;"));

        assertGeneratedJavaCompiles(enabledJavaFile, disabledJavaFile);
    }

    private Path writeJavaDefinitions(String headerName, String javaName, String... definitionLines) throws Exception {
        Path definitions = temporaryFolder.newFile(headerName).toPath();
        Files.write(definitions, Arrays.asList(definitionLines), StandardCharsets.UTF_8);

        ParseState state = new ParseState();
        state.setDefinitionPolicy(Definition.OverwritePolicy.NotAllowed);
        RusefiParseErrorStrategy.parseDefinitionFile(state.getListener(), definitions.toString());

        Path javaFile = temporaryFolder.newFile(javaName).toPath();
        JavaFieldsWriter writer = new JavaFieldsWriter(javaFile.toString(), 0);
        writer.writeDefinitions(state.getDefinitions());
        writer.finish();
        return javaFile;
    }

    private void assertGeneratedJavaCompiles(Path... javaFiles) throws Exception {
        Path stub = temporaryFolder.newFolder("stubs", "com", "rusefi", "config")
                .toPath().resolve("Field.java");
        Files.write(stub, Arrays.asList(
                "package com.rusefi.config;",
                "",
                "public class Field {}"
        ), StandardCharsets.UTF_8);
        Path classes = temporaryFolder.newFolder("classes").toPath();

        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        assertNotNull("Tests require a JDK with the system Java compiler", compiler);
        String[] arguments = new String[4 + javaFiles.length];
        arguments[0] = "-d";
        arguments[1] = classes.toString();
        arguments[2] = "-proc:none";
        arguments[3] = stub.toString();
        for (int i = 0; i < javaFiles.length; i++) {
            arguments[4 + i] = javaFiles[i].toString();
        }
        assertEquals("Generated Java did not compile", 0,
                compiler.run(null, null, null, arguments));
    }

    private static ParseState parseWithDefinitions(Path definitions, String input) throws Exception {
        ParseState parseState = new ParseState();
        parseState.setDefinitionPolicy(Definition.OverwritePolicy.NotAllowed);
        RusefiParseErrorStrategy.parseDefinitionFile(parseState.getListener(), definitions.toString());
        RusefiParseErrorStrategy.parseDefinitionString(parseState.getListener(), input);
        return parseState;
    }
}
