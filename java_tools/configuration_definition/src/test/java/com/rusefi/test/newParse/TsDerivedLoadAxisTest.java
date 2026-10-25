/*
 * @author Giacomo di Ciocco, (C) 2026
 */
package com.rusefi.test.newParse;

import org.junit.Test;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TsDerivedLoadAxisTest {
    private static final Path TEMPLATE = Paths.get(
            "../../firmware/tunerstudio/tunerstudio.template.ini").toAbsolutePath().normalize();
    private static final String LOAD_AXIS = "modelDefaultLoadAxis";
    private static final String DEFINITION =
            "modelDefaultLoadAxis = { fuelAlgorithm == " +
                    "@@engine_load_mode_e_LM_SPEED_DENSITY@@ ? 5 : (fuelAlgorithm == " +
                    "@@engine_load_mode_e_LM_ALPHA_N@@ ? 2 : 4) }";

    @Test
    public void derivedLoadAxisIsDefinedOnlyInOutputChannels() throws Exception {
        List<String> lines = readTemplate();
        List<String> outputChannels = section(lines, "OutputChannels");
        List<String> pcVariables = section(lines, "PcVariables");

        List<String> definitions = definitions(outputChannels);
        assertEquals("OutputChannels must define modelDefaultLoadAxis once", 1, definitions.size());
        assertEquals(DEFINITION, definitions.get(0).trim());
        assertEquals("modelDefaultLoadAxis must have one definition in the template",
                1, definitions(lines).size());
        assertTrue("modelDefaultLoadAxis must not be a PcVariables entry",
                definitions(pcVariables).isEmpty());
    }

    @Test
    public void tableLabelReferencesKeepTheirDefinitions() throws Exception {
        List<String> lines = readTemplate();
        assertFalse("OutputChannels must define modelDefaultLoadAxis", definitions(lines).isEmpty());

        List<String> pcVariables = section(lines, "PcVariables");
        assertTrue("loadAxisLabels enum must remain defined in PcVariables",
                pcVariables.stream().anyMatch(line -> line.trim().startsWith("loadAxisLabels = bits")));

        List<String> tableEditor = section(lines, "TableEditor");
        int references = 0;
        for (String line : lines) {
            if (!line.contains(LOAD_AXIS) || defines(line)) {
                continue;
            }

            references++;
            assertTrue("modelDefaultLoadAxis reference must be in TableEditor: " + line,
                    tableEditor.contains(line));
            assertTrue("modelDefaultLoadAxis reference must be an xyLabels entry: " + line,
                    line.trim().startsWith("xyLabels"));
            assertTrue("modelDefaultLoadAxis reference must bind through loadAxisLabels: " + line,
                    line.contains("bitStringValue(loadAxisLabels"));
        }
        assertTrue("Expected at least one bound table-label reference", references > 0);
    }

    private static List<String> readTemplate() throws Exception {
        return Files.readAllLines(TEMPLATE, StandardCharsets.UTF_8);
    }

    private static List<String> definitions(List<String> lines) {
        List<String> definitions = new ArrayList<>();
        for (String line : lines) {
            if (defines(line)) {
                definitions.add(line);
            }
        }
        return definitions;
    }

    private static boolean defines(String line) {
        String trimmed = line.trim();
        if (!trimmed.startsWith(LOAD_AXIS)) {
            return false;
        }
        String operator = trimmed.substring(LOAD_AXIS.length()).trim();
        return operator.startsWith("=") && !operator.startsWith("==");
    }

    private static List<String> section(List<String> lines, String name) {
        String header = "[" + name + "]";
        int start = -1;
        for (int i = 0; i < lines.size(); i++) {
            String trimmed = lines.get(i).trim();
            if (start < 0 && header.equals(trimmed)) {
                start = i + 1;
            } else if (start >= 0 && trimmed.startsWith("[") && trimmed.endsWith("]")) {
                return lines.subList(start, i);
            }
        }
        assertTrue("Missing INI section " + header + " in " + TEMPLATE, start >= 0);
        return lines.subList(start, lines.size());
    }
}
