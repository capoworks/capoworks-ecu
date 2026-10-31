/*
 * @author Giacomo di Ciocco, (C) 2026
 */
package com.rusefi.test.newParse;

import com.opensr5.ini.RawIniFile;
import org.junit.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TsVeAnalyzeReferenceTest {
    private static final Path TEMPLATE =
            Paths.get("../../firmware/tunerstudio/tunerstudio.template.ini").toAbsolutePath().normalize();
    private static final Path WALL_FUEL_STATE =
            Paths.get("../../firmware/controllers/algo/wall_fuel_state.txt").toAbsolutePath().normalize();
    private static final Pattern TABLE_DECLARATION =
            Pattern.compile("^\\s*table\\s*=\\s*([^,\\s]+)\\s*,");
    private static final Pattern VE_ANALYZE_MAP =
            Pattern.compile("^\\s*veAnalyzeMap\\s*=\\s*([^,\\s]+)\\s*,\\s*([^,\\s]+)\\s*,");
    private static final String LAMBDA_SELECTOR = "veAnalyzeLambdaBank";
    private static final String SELECTED_LAMBDA = "veAnalyzeSelectedLambda";
    private static final String SELECTED_EGO = "veAnalyzeSelectedEgoCorrection";
    private static final String BANK_0_ASSIGNED = "veAnalyzeBank0Assigned";
    private static final String BANK_1_ASSIGNED = "veAnalyzeBank1Assigned";
    private static final String BANK_ASSIGNED = "veAnalyzeSelectedBankAssigned";
    private static final String INVALID_LAMBDA = "veAnalyzeLambdaInvalid";
    private static final String EVENT_READY = "veAnalyzeFuelEventReady";
    private static final String EVENT_AE_PERCENT = "veAnalyzeAePercent";
    private static final String EVENT_FILM_PERCENT = "veAnalyzeWallCorrectionPercent";
    private static final String EVENT_SUM_ABS_PERCENT = "veAnalyzeSumAbsAeFilmPercent";
    private static final String INJECTION_MULTIPLIER = "veAnalyzeInjectionMultiplier";
    private static final String[] EVENT_AE_BANKS =
            bankChannels("veAnalyzeFuelBankPeakAePercent");
    private static final String[] EVENT_FILM_BANKS =
            bankChannels("veAnalyzeFuelBankPeakFilmPercent");
    private static final String[] EVENT_SUM_ABS_BANKS =
            bankChannels("veAnalyzeFuelBankPeakSumAbsPercent");

    @Test
    public void everyVeAnalyzeMapTableIsDeclaredInTableEditor() throws IOException {
        List<RawIniFile.Line> lines = readIni().getLines();
        Set<String> tableDeclarations = new LinkedHashSet<>();
        List<String> checkedReferences = new ArrayList<>();
        List<String> referencedTables = new ArrayList<>();
        List<String> missingReferences = new ArrayList<>();

        for (RawIniFile.Line line : section(lines, "TableEditor")) {
            Matcher table = TABLE_DECLARATION.matcher(line.getRawText());
            if (table.find()) {
                tableDeclarations.add(table.group(1));
            }
        }

        for (int i = 0; i < lines.size(); i++) {
            RawIniFile.Line line = lines.get(i);
            Matcher map = VE_ANALYZE_MAP.matcher(line.getRawText());
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

    @Test
    public void veAnalyzeMapsAndFiltersUseOneSelectedBank() throws IOException {
        List<RawIniFile.Line> section = section(readIni().getLines(), "VeAnalyze");
        List<List<RawIniFile.Line>> blocks = veAnalyzeBlocks(section);
        assertEquals("Every VE Analyze model needs an independently qualified filter block", 3, blocks.size());

        for (List<RawIniFile.Line> block : blocks) {
            RawIniFile.Line map = entries(block, "veAnalyzeMap").get(0);
            String[] mapTokens = map.getTokens();
            assertTrue("Unexpected VE Analyze map token count: " + map, mapTokens.length >= 6);
            assertEquals(SELECTED_LAMBDA, mapTokens[3]);
            assertEquals(SELECTED_EGO, mapTokens[4]);
            String condition = joinTokens(mapTokens, 5);
            assertTrue("Model qualification must remain on the map: " + map,
                    condition.contains("blendedStatus"));
            assertFalse("Readiness gates must be filters, not activeCondition: " + map,
                    condition.contains("veAnalyzeSampleReady")
                            || condition.contains("veAnalyzeRateValid")
                            || condition.contains("veAnalyzeTpsRateAbs")
                            || condition.contains(EVENT_READY)
                            || condition.contains(EVENT_AE_PERCENT)
                            || condition.contains(EVENT_FILM_PERCENT)
                            || condition.contains(EVENT_SUM_ABS_PERCENT));

            assertNonAdjustableGate(block, "veAnalyzeSampleReady");
            assertNonAdjustableGate(block, "veAnalyzeRateValid");
            assertNonAdjustableGate(block, EVENT_READY);
            assertAdjustableFilter(block, "veAnalyzeTpsRateAbs", ">", "5");
            assertAdjustableFilter(block, EVENT_AE_PERCENT, ">", "1");
            assertAdjustableFilter(block, EVENT_FILM_PERCENT, ">", "2");
            assertAdjustableFilter(block, EVENT_SUM_ABS_PERCENT, ">", "2");
            assertAdjustableFilter(block, SELECTED_LAMBDA, "<", "0.70");
            assertAdjustableFilter(block, SELECTED_LAMBDA, ">", "1.30");
            assertFilterTitleContains(block, EVENT_AE_PERCENT, "event BASE fuel");
            assertFilterTitleContains(block, EVENT_FILM_PERCENT, "event BASE fuel");
            assertFilterTitleContains(block, EVENT_SUM_ABS_PERCENT, "event BASE fuel");

            List<RawIniFile.Line> invalid = entries(block, "filter").stream()
                    .filter(line -> "invalidLambdaFilter".equals(line.getTokens()[1]))
                    .collect(Collectors.toList());
            assertEquals("Each model block needs one invalid-lambda filter", 1, invalid.size());
            String[] invalidTokens = invalid.get(0).getTokens();
            assertEquals(INVALID_LAMBDA, invalidTokens[3]);
            assertEquals(">", invalidTokens[4]);
            assertEquals("0", invalidTokens[5]);
            assertEquals("Invalid-lambda rejection must declare userAdjustable=false", "false", invalidTokens[6]);
        }

        for (RawIniFile.Line line : section) {
            assertFalse("Legacy VE Analyze deltaTps filter remains: " + line,
                    Arrays.asList(line.getTokens()).contains("deltaTps"));
        }
    }

    @Test
    public void eventPeakAliasesUseAllFourBanksAndEventBaseFuel() throws IOException {
        List<RawIniFile.Line> outputs = section(readIni().getLines(), "OutputChannels");

        String ae = expression(outputs, EVENT_AE_PERCENT);
        assertBankMaximum(ae, EVENT_AE_BANKS);
        assertFalse("AE ratio must use existing ECU-side absolute peaks: " + ae, ae.contains("abs("));

        String film = expression(outputs, EVENT_FILM_PERCENT);
        assertBankMaximum(film, EVENT_FILM_BANKS);
        assertFalse("Film alias must not return to the legacy wall/running-fuel estimate: " + film,
                film.contains("wallFuelCorrectionValue") || film.contains("running_fuel"));

        String sumAbs = expression(outputs, EVENT_SUM_ABS_PERCENT);
        assertBankMaximum(sumAbs, EVENT_SUM_ABS_BANKS);
        assertFalse("The ECU already computes the sum of absolute AE and film ratios: " + sumAbs,
                sumAbs.contains("abs(") || sumAbs.contains("+"));

        for (String alias : Arrays.asList(EVENT_AE_PERCENT, EVENT_FILM_PERCENT, EVENT_SUM_ABS_PERCENT)) {
            String ratio = expression(outputs, alias);
            assertFalse("Event ratios must not reapply the legacy injection multiplier: " + ratio,
                    ratio.contains(INJECTION_MULTIPLIER));
        }

        String legacyMultiplier = expression(outputs, INJECTION_MULTIPLIER);
        assertTrue("Legacy multiplier alias must remain available for saved definitions: "
                        + legacyMultiplier,
                legacyMultiplier.contains("currentInjectionMode"));
    }

    @Test
    public void selectedAliasesChainToMatchingLambdaAndEgoBank() throws IOException {
        List<RawIniFile.Line> outputs = section(readIni().getLines(), "OutputChannels");

        String lambda = expression(outputs, SELECTED_LAMBDA);
        assertTrue(lambda, lambda.contains(LAMBDA_SELECTOR));
        assertTrue(lambda, lambda.contains("lambdaValues1"));
        assertTrue(lambda, lambda.contains("lambdaValues2"));
        assertTrue("Selection must be a choice, not an average: " + lambda,
                lambda.contains("?") && lambda.contains(":") && !lambda.contains("/"));

        String ego = expression(outputs, SELECTED_EGO);
        assertTrue(ego, ego.contains(LAMBDA_SELECTOR));
        assertTrue(ego, ego.contains("100"));
        assertTrue(ego, ego.contains("fuelPidCorrection1"));
        assertTrue(ego, ego.contains("fuelPidCorrection2"));
        assertTrue("EGO selection must not average banks: " + ego,
                ego.contains("?") && ego.contains(":") && !ego.contains("/"));

        String assigned = expression(outputs, BANK_ASSIGNED);
        assertTrue(assigned, assigned.contains(LAMBDA_SELECTOR));
        assertTrue(assigned, assigned.contains(BANK_0_ASSIGNED));
        assertTrue(assigned, assigned.contains(BANK_1_ASSIGNED));

        String bank0Assigned = expression(outputs, BANK_0_ASSIGNED);
        String bank1Assigned = expression(outputs, BANK_1_ASSIGNED);
        assertTrue(bank0Assigned, bank0Assigned.contains("cylindersCount"));
        assertTrue(bank1Assigned, bank1Assigned.contains("cylindersCount"));
        for (int cylinder = 1; cylinder <= 12; cylinder++) {
            assertTrue("Configured cylinder assignment must be checked for bank 0: " + bank0Assigned,
                    bank0Assigned.contains("cylinderBankSelect" + cylinder + " == 0"));
            assertTrue("Configured cylinder assignment must be checked for bank 1: " + bank1Assigned,
                    bank1Assigned.contains("cylinderBankSelect" + cylinder + " == 1"));
        }

        String invalid = expression(outputs, INVALID_LAMBDA);
        assertTrue(invalid, invalid.contains(LAMBDA_SELECTOR));
        assertTrue(invalid, invalid.contains(BANK_ASSIGNED));
        assertTrue(invalid, invalid.contains("isNaN(" + SELECTED_LAMBDA + ")"));
        assertTrue(invalid, invalid.contains(SELECTED_LAMBDA + " <= 0"));
    }

    @Test
    public void lambdaBankSelectorIsPcOnlyAndExposedInVeAnalyzeSettings() throws IOException {
        RawIniFile ini = readIni();
        List<RawIniFile.Line> pcVariables = section(ini.getLines(), "PcVariables");
        RawIniFile.Line selector = definition(pcVariables, LAMBDA_SELECTOR);
        assertArrayEquals(new String[]{
                        LAMBDA_SELECTOR, "bits", "U08", "[0:0]", "Lambda 1", "Lambda 2"},
                selector.getTokens());

        List<RawIniFile.Line> defaults = entries(section(ini.getLines(), "ConstantsExtensions"), "defaultValue");
        assertTrue("Lambda selector must default to Lambda 1",
                defaults.stream().anyMatch(line -> line.getTokens().length == 3
                        && LAMBDA_SELECTOR.equals(line.getTokens()[1])
                        && "0".equals(line.getTokens()[2])));
        assertFalse("Lambda selector must not be an ECU configuration constant",
                definitions(section(ini.getLines(), "Constants"), LAMBDA_SELECTOR).size() > 0);

        List<RawIniFile.Line> menu = section(ini.getLines(), "Menu");
        assertTrue("VE Analyze settings must be reachable from the Fuel menu",
                menu.stream().anyMatch(line -> {
                    String[] tokens = line.getTokens();
                    return tokens.length >= 2
                            && "subMenu".equals(tokens[0])
                            && "veAnalyzeSettings".equals(tokens[1]);
                }));
        List<RawIniFile.Line> userDefined = section(ini.getLines(), "UserDefined");
        assertTrue("VE Analyze dialog must expose the PC lambda selector",
                hasDialogField(userDefined, "veAnalyzeSettings", LAMBDA_SELECTOR));
    }

    @Test
    public void wueAnalyzeStaysCoherentWithLambdaBankOne() throws IOException {
        RawIniFile ini = readIni();
        List<RawIniFile.Line> wueMaps = entries(section(ini.getLines(), "WueAnalyze"), "wueAnalyzeMap");
        assertEquals(1, wueMaps.size());
        String[] tokens = wueMaps.get(0).getTokens();
        assertEquals("WUE must use its warmup curve", "wueAnalyzer_warmup_curve", tokens[1]);
        assertEquals("WUE lambda input must remain bank 1", "lambdaValues1", tokens[4]);
        assertEquals("WUE coolant input must remain coolant", "coolant", tokens[5]);
        assertEquals("WUE warmup channel must remain the coolant coefficient",
                "running_coolantTemperatureCoefficient", tokens[6]);
        assertEquals("WUE EGO input must remain the bank-1 alias", "egoCorrectionForVeAnalyze", tokens[7]);

        String wueEgo = expression(section(ini.getLines(), "OutputChannels"), "egoCorrectionForVeAnalyze");
        assertTrue(wueEgo, wueEgo.contains("100 + fuelPidCorrection1"));
        assertTrue(wueEgo, wueEgo.contains("fuelPidCorrection1"));
        assertFalse("WUE must not follow the VE Analyze selector: " + wueEgo,
                wueEgo.contains("fuelPidCorrection2") || wueEgo.contains(LAMBDA_SELECTOR));
    }

    @Test
    public void wallFuelTelemetryPreservesRawFloatLayoutAndDecodesClientMilligrams() throws IOException {
        String source = new String(Files.readAllBytes(WALL_FUEL_STATE), StandardCharsets.UTF_8);
        String outputChannels = NewParseHelper.parseToOutputChannels(source);

        assertRawGramsDecodeToMilligrams(outputChannels, "wallFuelCorrection", 0);
        assertRawGramsDecodeToMilligrams(outputChannels, "wallFuel", 4);

        String cStruct = NewParseHelper.parseToC(source);
        assertRawFloatFieldDeclaration(cStruct, "wallFuelCorrection");
        assertRawFloatFieldDeclaration(cStruct, "wallFuel");
        assertFalse("Float telemetry scaling must not introduce C++ autoscaling: " + cStruct,
                cStruct.contains("scaled_channel<"));
        assertTrue("Wall fuel correction must remain at offset 0: " + cStruct,
                cStruct.contains("offsetof(wall_fuel_state_s, wallFuelCorrection) == 0"));
        assertTrue("Wall fuel must remain at offset 4: " + cStruct,
                cStruct.contains("offsetof(wall_fuel_state_s, wallFuel) == 4"));
        assertTrue("Wall fuel state layout must remain 8 bytes: " + cStruct,
                cStruct.contains("sizeof(wall_fuel_state_s) == 8"));
    }

    private static void assertRawFloatFieldDeclaration(String cStruct, String field) {
        Pattern declaration = Pattern.compile("(?m)^[\\t ]*float[\\t ]+" + Pattern.quote(field)
                + "[\\t ]*(?:=[\\t ]*[^;\\r\\n]+)?;[\\t ]*\\r?$");
        assertTrue("Expected a raw float C declaration for " + field + ":\n" + cStruct,
                declaration.matcher(cStruct).find());
    }

    private static void assertRawGramsDecodeToMilligrams(
            String outputChannels, String channel, int offset) {
        Pattern linePattern = Pattern.compile("^" + Pattern.quote(channel)
                + " = scalar, F32, " + offset + ", \"mg\", ([^,]+), 0$", Pattern.MULTILINE);
        Matcher line = linePattern.matcher(outputChannels);
        assertTrue("Missing F32 milligram telemetry descriptor for " + channel + ":\n" + outputChannels,
                line.find());

        double decodedMilligrams = 0.001 * Double.parseDouble(line.group(1));
        assertEquals("A raw " + channel + " value of 0.001 g must decode to 1 mg",
                1.0, decodedMilligrams, 1e-12);
    }

    private static void assertNonAdjustableGate(List<RawIniFile.Line> block, String channel) {
        List<RawIniFile.Line> filters = filtersForChannel(block, channel);
        assertEquals("Each model block must require one " + channel + " gate", 1, filters.size());
        String[] tokens = filters.get(0).getTokens();
        assertEquals("Required gate operator must be <", "<", tokens[4]);
        assertEquals("Required gate threshold must be 1", "1", tokens[5]);
        assertEquals(channel + " must declare userAdjustable=false", "false", tokens[6]);

        double threshold = Double.parseDouble(tokens[5]);
        assertTrue("A not-ready value of 0 must be rejected: " + filters.get(0),
                filterRejects(tokens[4], threshold, 0));
        assertFalse("A ready value of 1 must pass: " + filters.get(0),
                filterRejects(tokens[4], threshold, 1));
    }

    private static boolean filterRejects(String operator, double threshold, double value) {
        switch (operator) {
            case "<":
                return value < threshold;
            case "<=":
                return value <= threshold;
            case ">":
                return value > threshold;
            case ">=":
                return value >= threshold;
            case "==":
                return value == threshold;
            case "!=":
                return value != threshold;
            default:
                throw new AssertionError("Unsupported TunerStudio filter operator: " + operator);
        }
    }

    private static void assertAdjustableFilter(
            List<RawIniFile.Line> block, String channel, String operator, String threshold) {
        List<RawIniFile.Line> filters = filtersForChannel(block, channel).stream()
                .filter(line -> operator.equals(line.getTokens()[4]))
                .collect(Collectors.toList());
        assertEquals("Expected one " + operator + " " + threshold + " filter for " + channel,
                1, filters.size());
        assertEquals(threshold, filters.get(0).getTokens()[5]);
        assertEquals("true", filters.get(0).getTokens()[6]);
    }

    private static void assertFilterTitleContains(
            List<RawIniFile.Line> block, String channel, String expected) {
        List<RawIniFile.Line> filters = filtersForChannel(block, channel);
        assertEquals("Expected one filter for " + channel, 1, filters.size());
        assertTrue("Expected filter title to contain '" + expected + "': " + filters.get(0),
                filters.get(0).getRawText().contains(expected));
    }

    private static void assertBankMaximum(String expression, String[] bankChannels) {
        String expected = "max(max(max(" + bankChannels[0] + ", " + bankChannels[1] + "), "
                + bankChannels[2] + "), " + bankChannels[3] + ")";
        assertTrue("Bank alias must take the maximum across the four event banks; expected "
                        + expected + ": " + expression,
                expression.contains(expected));
        for (String bankChannel : bankChannels) {
            assertTrue("Bank maximum must reference " + bankChannel + ": " + expression,
                    expression.contains(bankChannel));
        }
    }

    private static String[] bankChannels(String base) {
        String[] channels = new String[4];
        for (int bank = 1; bank <= channels.length; bank++) {
            channels[bank - 1] = base + bank;
        }
        return channels;
    }

    private static List<RawIniFile.Line> filtersForChannel(List<RawIniFile.Line> block, String channel) {
        return entries(block, "filter").stream()
                .filter(line -> line.getTokens().length >= 7 && channel.equals(line.getTokens()[3]))
                .collect(Collectors.toList());
    }

    private static List<List<RawIniFile.Line>> veAnalyzeBlocks(List<RawIniFile.Line> section) {
        List<List<RawIniFile.Line>> blocks = new ArrayList<>();
        List<RawIniFile.Line> current = null;
        for (RawIniFile.Line line : section) {
            if (line.getTokens().length > 0 && "veAnalyzeMap".equals(line.getTokens()[0])) {
                current = new ArrayList<>();
                blocks.add(current);
            }
            if (current != null) {
                current.add(line);
            }
        }
        return blocks;
    }

    private static boolean hasDialogField(List<RawIniFile.Line> menu, String dialog, String field) {
        boolean inDialog = false;
        for (RawIniFile.Line line : menu) {
            String[] tokens = line.getTokens();
            if (tokens.length >= 2 && "dialog".equals(tokens[0])) {
                inDialog = dialog.equals(tokens[1]);
            } else if (inDialog && tokens.length >= 3
                    && "field".equals(tokens[0])
                    && field.equals(tokens[2])) {
                return true;
            }
        }
        return false;
    }

    private static String expression(List<RawIniFile.Line> section, String name) {
        String rawText = definition(section, name).getRawText();
        int assignment = rawText.indexOf('=');
        assertTrue("Expected an assignment for " + name + ": " + rawText, assignment >= 0);
        return rawText.substring(assignment + 1).trim();
    }

    private static RawIniFile.Line definition(List<RawIniFile.Line> section, String name) {
        List<RawIniFile.Line> matches = definitions(section, name);
        assertEquals("Expected one definition for " + name, 1, matches.size());
        return matches.get(0);
    }

    private static List<RawIniFile.Line> definitions(List<RawIniFile.Line> lines, String name) {
        return entries(lines, name).stream()
                .filter(line -> line.getRawText().trim().startsWith(name))
                .collect(Collectors.toList());
    }

    private static List<RawIniFile.Line> entries(List<RawIniFile.Line> lines, String key) {
        return lines.stream()
                .filter(line -> line.getTokens().length > 0 && key.equals(line.getTokens()[0]))
                .collect(Collectors.toList());
    }

    private static String joinTokens(String[] tokens, int from) {
        return String.join(" ", Arrays.copyOfRange(tokens, from, tokens.length));
    }

    private static List<RawIniFile.Line> section(List<RawIniFile.Line> lines, String name) {
        String header = "[" + name + "]";
        int start = -1;
        for (int i = 0; i < lines.size(); i++) {
            String trimmed = lines.get(i).getRawText().trim();
            if (start < 0 && header.equals(trimmed)) {
                start = i + 1;
            } else if (start >= 0 && trimmed.startsWith("[") && trimmed.endsWith("]")) {
                return lines.subList(start, i);
            }
        }
        assertTrue("Missing INI section " + header + " in " + TEMPLATE, start >= 0);
        return lines.subList(start, lines.size());
    }

    private static RawIniFile readIni() throws IOException {
        return RawIniFile.read(TEMPLATE.toString());
    }
}
