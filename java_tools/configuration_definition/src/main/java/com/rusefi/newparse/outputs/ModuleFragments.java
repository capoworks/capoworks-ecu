package com.rusefi.newparse.outputs;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Pattern;

/** Static composition points for module-owned configuration, live data, and TunerStudio text. */
public final class ModuleFragments {
    public static final String CONFIG_ANCHOR = "! MODULE_CONFIG";
    public static final String PERSISTENT_CONFIG_ANCHOR = "! MODULE_PERSISTENT_CONFIG";
    public static final String OUTPUT_ANCHOR = "! MODULE_OUTPUTS";
    public static final String INI_ANCHOR_PREFIX = "; MODULE_INI ";

    private static final Pattern INI_CONTEXT = Pattern.compile("[A-Za-z0-9_]+(?:\\.[A-Za-z0-9_]+)?");

    private ModuleFragments() {
    }

    public static String composeConfig(String coreFile, List<String> fragmentFiles) throws IOException {
        return composeConfig(coreFile, fragmentFiles, Collections.emptyList());
    }

    public static String composeConfig(String coreFile, List<String> fragmentFiles,
                                       List<String> persistentFragmentFiles) throws IOException {
        Map<String, List<String>> fragmentsByAnchor = new LinkedHashMap<>();
        fragmentsByAnchor.put(CONFIG_ANCHOR, fragmentFiles);
        fragmentsByAnchor.put(PERSISTENT_CONFIG_ANCHOR, persistentFragmentFiles);
        return composeAnchors(coreFile, fragmentsByAnchor);
    }

    public static String composeOutput(String coreFile, List<String> fragmentFiles) throws IOException {
        return composeSingleAnchor(coreFile, fragmentFiles, OUTPUT_ANCHOR);
    }

    /**
     * Inserts each fragment section at its matching template marker. Markers with no supplied
     * section are removed so the same template can be used with an empty module selection.
     */
    public static List<String> composeIniTemplate(String templateFile, List<String> fragmentFiles) throws IOException {
        Map<String, List<String>> sectionBodies = readIniSections(fragmentFiles);
        List<String> templateLines = Files.readAllLines(Paths.get(templateFile), StandardCharsets.UTF_8);

        Map<String, Integer> markerCounts = new LinkedHashMap<>();
        String currentSection = null;
        for (int i = 0; i < templateLines.size(); i++) {
            String line = templateLines.get(i);
            String section = sectionHeader(line, templateFile, i + 1);
            if (section != null) {
                currentSection = validateIniContext(section, templateFile, i + 1);
            }

            String markerSection = iniMarkerSection(line, templateFile);
            if (markerSection != null) {
                markerSection = validateIniContext(markerSection, templateFile, i + 1);
                String baseSection = markerSection.split("\\.", 2)[0];
                if (!baseSection.equals(currentSection)) {
                    throw new IllegalArgumentException("Module INI marker for [" + markerSection
                            + "] must appear inside [" + baseSection + "] in " + templateFile
                            + ":" + (i + 1) + ", not [" + currentSection + "]");
                }
                markerCounts.put(markerSection, markerCounts.getOrDefault(markerSection, 0) + 1);
            }
        }

        for (Map.Entry<String, Integer> marker : markerCounts.entrySet()) {
            if (marker.getValue() > 1) {
                throw new IllegalArgumentException("Duplicate '" + INI_ANCHOR_PREFIX + marker.getKey()
                        + "' marker in " + templateFile);
            }
        }

        for (String section : sectionBodies.keySet()) {
            if (markerCounts.getOrDefault(section, 0) == 0) {
                throw new IllegalArgumentException("Missing '" + INI_ANCHOR_PREFIX + section
                        + "' marker in " + templateFile + " for supplied fragments " + fragmentFiles);
            }
        }

        List<String> result = new ArrayList<>();
        for (String line : templateLines) {
            String section = iniMarkerSection(line, templateFile);
            if (section == null) {
                result.add(line);
            } else {
                result.addAll(sectionBodies.getOrDefault(section, Collections.emptyList()));
            }
        }
        return result;
    }

    private static String composeSingleAnchor(String coreFile, List<String> fragmentFiles, String anchor)
            throws IOException {
        String coreText = new String(Files.readAllBytes(Paths.get(coreFile)), StandardCharsets.UTF_8);
        List<String> lines = splitLines(coreText);

        int anchorCount = 0;
        for (String line : lines) {
            if (line.trim().equals(anchor)) {
                anchorCount++;
            }
        }

        if (fragmentFiles.isEmpty()) {
            if (anchorCount == 0) {
                return coreText;
            }
            List<String> withoutMarkers = new ArrayList<>();
            for (String line : lines) {
                if (!line.trim().equals(anchor)) {
                    withoutMarkers.add(line);
                }
            }
            return String.join("\n", withoutMarkers);
        }

        if (anchorCount == 0) {
            throw new IllegalArgumentException("Missing '" + anchor + "' anchor in " + coreFile);
        }
        if (anchorCount > 1) {
            throw new IllegalArgumentException("Duplicate '" + anchor + "' anchor in " + coreFile);
        }

        List<String> fragmentLines = readFragmentLines(fragmentFiles);
        List<String> result = new ArrayList<>();
        for (String line : lines) {
            if (line.trim().equals(anchor)) {
                result.addAll(fragmentLines);
            } else {
                result.add(line);
            }
        }
        return String.join("\n", result);
    }

    private static Map<String, List<String>> readIniSections(List<String> fragmentFiles) throws IOException {
        Map<String, List<String>> bodies = new LinkedHashMap<>();

        for (String fragmentFile : fragmentFiles) {
            List<String> lines = Files.readAllLines(Paths.get(fragmentFile), StandardCharsets.UTF_8);
            Set<String> sectionsInFile = new HashSet<>();
            String currentSection = null;

            for (int i = 0; i < lines.size(); i++) {
                String line = lines.get(i);
                String trimmed = line.trim();
                String section = sectionHeader(line, fragmentFile, i + 1);

                if (section != null) {
                    section = validateIniContext(section, fragmentFile, i + 1);
                    if (!sectionsInFile.add(section)) {
                        throw new IllegalArgumentException("Duplicate [" + section + "] section in "
                                + fragmentFile + ":" + (i + 1));
                    }
                    currentSection = section;
                    bodies.computeIfAbsent(section, ignored -> new ArrayList<>());
                } else if (currentSection == null) {
                    if (!trimmed.isEmpty()) {
                        throw new IllegalArgumentException("Text before first section in " + fragmentFile
                                + ":" + (i + 1) + ": " + line);
                    }
                } else {
                    bodies.get(currentSection).add(line);
                }
            }
        }
        return bodies;
    }

    private static String sectionHeader(String line, String fragmentFile, int lineNumber) {
        String trimmed = line.trim();
        if (!trimmed.startsWith("[")) {
            return null;
        }

        int close = trimmed.indexOf(']');
        if (close < 0 || close != trimmed.length() - 1) {
            throw new IllegalArgumentException("Malformed section header in " + fragmentFile
                    + ":" + lineNumber + ": " + line);
        }

        String section = trimmed.substring(1, close);
        return section;
    }

    private static String iniMarkerSection(String line, String templateFile) {
        String trimmed = line.trim();
        if (!trimmed.startsWith("; MODULE_INI")) {
            return null;
        }
        if (!trimmed.startsWith(INI_ANCHOR_PREFIX)) {
            throw new IllegalArgumentException("Malformed module INI marker in " + templateFile + ": " + line);
        }

        return trimmed.substring(INI_ANCHOR_PREFIX.length()).trim();
    }

    private static String validateIniContext(String context, String fileName, int lineNumber) {
        if (!INI_CONTEXT.matcher(context).matches()) {
            throw new IllegalArgumentException("Invalid TunerStudio composition key [" + context + "] in "
                    + fileName + ":" + lineNumber + "; expected Section or Section.Context");
        }
        return context;
    }

    private static String composeAnchors(String coreFile, Map<String, List<String>> fragmentsByAnchor)
            throws IOException {
        String coreText = new String(Files.readAllBytes(Paths.get(coreFile)), StandardCharsets.UTF_8);
        List<String> lines = splitLines(coreText);

        Map<String, Integer> anchorCounts = new LinkedHashMap<>();
        for (String anchor : fragmentsByAnchor.keySet()) {
            anchorCounts.put(anchor, 0);
        }
        for (String line : lines) {
            String trimmed = line.trim();
            if (anchorCounts.containsKey(trimmed)) {
                anchorCounts.put(trimmed, anchorCounts.get(trimmed) + 1);
            }
        }

        Map<String, List<String>> fragmentLinesByAnchor = new LinkedHashMap<>();
        for (Map.Entry<String, List<String>> entry : fragmentsByAnchor.entrySet()) {
            String anchor = entry.getKey();
            List<String> fragmentFiles = entry.getValue();
            int anchorCount = anchorCounts.get(anchor);

            if (!fragmentFiles.isEmpty()) {
                if (anchorCount == 0) {
                    throw new IllegalArgumentException("Missing '" + anchor + "' anchor in " + coreFile);
                }
                if (anchorCount > 1) {
                    throw new IllegalArgumentException("Duplicate '" + anchor + "' anchor in " + coreFile);
                }
                fragmentLinesByAnchor.put(anchor, readFragmentLines(fragmentFiles));
            }
        }

        List<String> result = new ArrayList<>();
        for (String line : lines) {
            String anchor = line.trim();
            if (fragmentsByAnchor.containsKey(anchor)) {
                result.addAll(fragmentLinesByAnchor.getOrDefault(anchor, Collections.emptyList()));
            } else {
                result.add(line);
            }
        }
        return String.join("\n", result);
    }

    private static List<String> readFragmentLines(List<String> fragmentFiles) throws IOException {
        List<String> result = new ArrayList<>();
        for (String fragmentFile : fragmentFiles) {
            String text = new String(Files.readAllBytes(Paths.get(fragmentFile)), StandardCharsets.UTF_8);
            List<String> lines = splitLines(text);
            if (!lines.isEmpty() && lines.get(lines.size() - 1).isEmpty()) {
                lines.remove(lines.size() - 1);
            }
            result.addAll(lines);
        }
        return result;
    }

    private static List<String> splitLines(String text) {
        String normalized = text.replace("\r\n", "\n").replace('\r', '\n');
        return new ArrayList<>(Arrays.asList(normalized.split("\n", -1)));
    }
}
