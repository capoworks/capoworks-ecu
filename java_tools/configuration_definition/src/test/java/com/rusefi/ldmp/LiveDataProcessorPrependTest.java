package com.rusefi.ldmp;

import com.rusefi.RusefiParseErrorStrategy;
import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.parsing.Definition;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;

import static org.junit.Assert.assertEquals;

public class LiveDataProcessorPrependTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void commandPrependIsAvailableToEveryLiveDataSchema() throws Exception {
        Path definitions = temporaryFolder.newFile("module_profile_defines.h").toPath();
        Files.write(definitions, Arrays.asList(
                "! generated test definitions",
                "#define CYLINDER_FUEL_TRIM_COUNT 4"
        ), StandardCharsets.UTF_8);

        LiveDataProcessor processor = new LiveDataProcessor();
        processor.addPrependFile(definitions.toString());

        ParseState parseState = new ParseState();
        parseState.setDefinitionPolicy(Definition.OverwritePolicy.NotAllowed);
        processor.parsePrependDefinitions(parseState, null);
        RusefiParseErrorStrategy.parseDefinitionString(parseState.getListener(),
                "struct_no_prefix live_schema_test_s\n" +
                        "float[CYLINDER_FUEL_TRIM_COUNT iterate] fuelTrim\n" +
                        "end_struct\n");

        Definition count = parseState.findDefinition("CYLINDER_FUEL_TRIM_COUNT");
        assertEquals(4, count.asInt());
    }
}
