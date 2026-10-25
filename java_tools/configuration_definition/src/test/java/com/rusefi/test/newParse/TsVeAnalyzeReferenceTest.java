/*
 * @author Giacomo di Ciocco, (C) 2026
 */
package com.rusefi.test.newParse;

import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TsVeAnalyzeReferenceTest {
    private static final Path TEMPLATE =
            Paths.get("../../firmware/tunerstudio/tunerstudio.template.ini").toAbsolutePath().normalize();
    private static final Pattern TABLE_DECLARATION =
            Pattern.compile("^\\s*table\\s*=\\s*([^,\\s]+)\\s*,");
    private static final Pattern VE_ANALYZE_MAP =
            Pattern.compile("^\\s*veAnalyzeMap\\s*=\\s*([^,\\s]+)\\s*,\\s*([^,\\s]+)\\s*,");

    @Test
    public void everyVeAnalyzeMapTableIsDeclaredInTableEditor() throws IOException {
        List<String> lines = Files.readAllLines(TEMPLATE, StandardCharsets.UTF_8);
        Set<String> tableDeclarations = new LinkedHashSet<>();
        List<String> checkedReferences = new ArrayList<>();
        List<String> referencedTables = new ArrayList<>();
        List<String> missingReferences = new ArrayList<>();
        boolean inTableEditor = false;

        for (int i = 0; i < lines.size(); i++) {
            String line = lines.get(i);
            String trimmed = line.trim();
            if (trimmed.startsWith("[") && trimmed.endsWith("]")) {
                inTableEditor = "[TableEditor]".equals(trimmed);
            } else if (inTableEditor) {
                Matcher table = TABLE_DECLARATION.matcher(line);
                if (table.find()) {
                    tableDeclarations.add(table.group(1));
                }
            }

            Matcher map = VE_ANALYZE_MAP.matcher(line);
            if (map.find()) {
                String primary = "line " + (i + 1) + " primary table '" + map.group(1) + "'";
                String target = "line " + (i + 1) + " target table '" + map.group(2) + "'";
                checkedReferences.add(primary);
                checkedReferences.add(target);
                referencedTables.add(map.group(1));
                referencedTables.add(map.group(2));
            }
        }

        for (int i = 0; i < checkedReferences.size(); i++) {
            if (!tableDeclarations.contains(referencedTables.get(i))) {
                missingReferences.add(checkedReferences.get(i));
            }
        }

        assertFalse("No veAnalyzeMap entries found in " + TEMPLATE, checkedReferences.isEmpty());
        assertTrue("veAnalyzeMap references are missing from [TableEditor] in " + TEMPLATE
                        + "\nMissing: " + missingReferences
                        + "\nChecked: " + checkedReferences
                        + "\nDeclared tables: " + tableDeclarations,
                missingReferences.isEmpty());
    }
}
