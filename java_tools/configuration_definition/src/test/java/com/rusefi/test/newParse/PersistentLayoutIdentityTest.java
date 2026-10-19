package com.rusefi.test.newParse;

import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.outputs.PersistentLayoutIdentity;
import org.junit.Test;

import static com.rusefi.test.newParse.NewParseHelper.parse;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

public class PersistentLayoutIdentityTest {
    private static final String DEFINITIONS =
            "#define MODE_NAMES \"invalid\", \"off\", \"on\", \"auto\"\n" +
            "custom mode_t 1 bits, U08, @OFFSET@, [0:1], @@MODE_NAMES@@\n" +
            "custom label_t 8 string, ASCII, @OFFSET@, 8\n";

    private static final String NESTED =
            "struct nested\n" +
            "uint16_t[3] values;;\"kPa\", 0.1, 0, 0, 1000, 1\n" +
            "end_struct\n";

    private static final String COMPLEX_ROOT =
            "struct_no_prefix persistent_config_s\n" +
            "uint8_t first\n" +
            "uint32_t second\n" +
            "int32_t autotemp temperature;;\"C\", 1, 0, -40, 150, 0\n" +
            "uint16_t autoscale scaled;;\"%\", 0.1, 0, 0, 100, 1\n" +
            "bit enabled\n" +
            "bit active\n" +
            "mode_t mode\n" +
            "label_t label\n" +
            "uint8_t[2 x 3] matrix\n" +
            "nested nestedValue\n" +
            "nested[2 iterate] rows\n" +
            "union\n" +
            "uint16_t raw\n" +
            "uint8_t[2] bytes\n" +
            "end_union\n" +
            "end_struct\n";

    @Test
    public void identityIncludesOnlyPersistentStorageSemantics() {
        String complex = DEFINITIONS + NESTED + COMPLEX_ROOT;
        String complexId = layoutId(complex);
        String complexDescriptor = PersistentLayoutIdentity.canonicalDescriptor(parse(complex));

        assertTrue(complexId.matches("0x[0-9a-f]{16}ULL"));
        assertEquals(complexId, layoutId(complex));
        assertEquals(complexId, layoutId(complex));
        assertEquals(complexDescriptor, PersistentLayoutIdentity.canonicalDescriptor(parse(complex)));
        assertTrue(complexDescriptor.contains("struct_array|"));
        assertTrue(complexDescriptor.contains("union|"));
        assertTrue(complexDescriptor.contains("string|"));
        assertTrue(complexDescriptor.contains("enum_value|"));
        assertTrue(complexDescriptor.contains("bit|"));
        assertTrue(complexDescriptor.contains("persistent_config_s.nestedValue.values|"));
        assertTrue(complexDescriptor.contains("persistent_config_s.rows.values|"));
        assertTrue(complexDescriptor.contains("|1:C|"));

        String swapped =
                "struct_no_prefix persistent_config_s\n" +
                "uint32_t second\n" +
                "uint32_t first\n" +
                "end_struct\n";
        String swappedAgain =
                "struct_no_prefix persistent_config_s\n" +
                "uint32_t first\n" +
                "uint32_t second\n" +
                "end_struct\n";
        assertNotEquals(layoutId(swapped), layoutId(swappedAgain));

        assertNotEquals(
                layoutId("struct_no_prefix persistent_config_s\nint32_t value\nend_struct\n"),
                layoutId("struct_no_prefix persistent_config_s\nuint32_t value\nend_struct\n"));

        assertNotEquals(
                layoutId("struct_no_prefix persistent_config_s\nbit enabled\nbit active\nend_struct\n"),
                layoutId("struct_no_prefix persistent_config_s\nbit active\nbit enabled\nend_struct\n"));

        assertNotEquals(
                layoutId("struct_no_prefix persistent_config_s\nuint8_t[2 x 3] matrix\nend_struct\n"),
                layoutId("struct_no_prefix persistent_config_s\nuint8_t[3 x 2] matrix\nend_struct\n"));

        String scaled =
                "struct_no_prefix persistent_config_s\n" +
                "uint16_t scaled;;\"%\", 0.1, 2, 0, 100, 1\n" +
                "end_struct\n";
        String equivalentScale =
                "struct_no_prefix persistent_config_s\n" +
                "uint16_t scaled;;\"%\", {1 / 10}, 2, 0, 100, 1\n" +
                "end_struct\n";
        String changedOffset =
                "struct_no_prefix persistent_config_s\n" +
                "uint16_t scaled;;\"%\", 0.1, 3, 0, 100, 1\n" +
                "end_struct\n";
        assertEquals(layoutId(scaled), layoutId(equivalentScale));
        assertNotEquals(layoutId(scaled), layoutId(changedOffset));
        assertNotEquals(layoutId(scaled),
                layoutId("struct_no_prefix persistent_config_s\n" +
                        "uint16_t scaled;;\"%\", 0.25, 2, 0, 100, 1\nend_struct\n"));

        String enumBase =
                "#define E \"off\", \"on\"\n" +
                "custom e_t 1 bits, U08, @OFFSET@, [0:1], @@E@@\n" +
                "struct_no_prefix persistent_config_s\n" +
                "e_t mode\n" +
                "end_struct\n";
        String enumRenamed =
                "#define E \"disabled\", \"on\"\n" +
                "custom e_t 1 bits, U08, @OFFSET@, [0:1], @@E@@\n" +
                "struct_no_prefix persistent_config_s\n" +
                "e_t mode\n" +
                "end_struct\n";
        assertNotEquals(layoutId(enumBase), layoutId(enumRenamed));

        String metadataVariant =
                "#define UNUSED_DEFAULT 123\n" +
                "#define UNUSED_ENUM \"left\", \"right\"\n" +
                "custom unused_t 1 bits, U08, @OFFSET@, [0:1], @@UNUSED_ENUM@@\n" +
                "struct unused_struct\n" +
                "int32_t ignored\n" +
                "end_struct\n" +
                DEFINITIONS +
                NESTED +
                "struct_no_prefix persistent_config_s\n" +
                "uint8_t first;new comment\n" +
                "uint32_t second;;\"\", 1, 0, 0, 100, 0\n" +
                "int32_t autotemp temperature;;\"C\", 1, 0, -40, 150, 0\n" +
                "uint16_t autoscale scaled;;\"%\", 0.1, 0, 0, 100, 1\n" +
                "bit enabled,\"yes\",\"no\";new help\n" +
                "bit active;another comment\n" +
                "mode_t mode;changed help\n" +
                "label_t label;changed string help\n" +
                "uint8_t[2 x 3] matrix;changed help\n" +
                "nested nestedValue;changed help\n" +
                "nested[2 iterate] rows;changed help\n" +
                "union\n" +
                "uint16_t raw;changed help\n" +
                "uint8_t[2] bytes;changed help\n" +
                "end_union\n" +
                "end_struct\n";
        assertEquals(complexId, layoutId(metadataVariant));
    }

    private static String layoutId(String input) {
        ParseState state = parse(input);
        return PersistentLayoutIdentity.layoutId(state);
    }
}
