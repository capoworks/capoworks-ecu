package com.rusefi.test.newParse;

import org.junit.Assert;
import org.junit.Test;

import java.io.IOException;

import static com.rusefi.test.newParse.NewParseHelper.parseToC;
import static com.rusefi.test.newParse.NewParseHelper.parseToTs;

public class TableStorageOrderTest {
    @Test
    public void rectangularTableUsesRowsThenColsInCAndColsThenRowsInTs() throws IOException {
        String definitions =
                "struct_no_prefix myStruct\n" +
                        "begin_table\n" +
                        "table_rows num 4 uint8_t rowBins\n" +
                        "table_cols num 8 uint16_t colBins\n" +
                        "table_values float tableVals\n" +
                        "end_table\n" +
                        "end_struct\n";

        String c = parseToC(definitions);
        assertContains(c, "\tuint8_t rowBins[4];");
        assertContains(c, "\tuint16_t colBins[8];");
        assertContains(c, "\tfloat tableVals[4][8];");
        assertContains(c, "static_assert(offsetof(myStruct, rowBins) == 0);");
        assertContains(c, "static_assert(offsetof(myStruct, colBins) == 4);");
        assertContains(c, "static_assert(offsetof(myStruct, tableVals) == 20);");
        assertContains(c, "static_assert(sizeof(myStruct) == 148);");

        String ts = parseToTs(definitions);
        assertContains(ts, "rowBins = array, U08, 0, [4], \"\", 1, 0, 0, 0, 0\n");
        assertContains(ts, "colBins = array, U16, 4, [8], \"\", 1, 0, 0, 0, 0\n");
        assertContains(ts, "tableVals = array, F32, 20, [8x4], \"\", 1, 0, 0, 0, 0\n");
        assertContains(ts, "; total TS size = 148\n");
    }

    @Test
    public void explicitRectangularArrayKeepsDeclarationOrder() throws IOException {
        String definitions =
                "struct_no_prefix myStruct\n" +
                        "float[8 x 4] explicitValues\n" +
                        "end_struct\n";

        String c = parseToC(definitions);
        assertContains(c, "\tfloat explicitValues[8][4];");
        assertContains(c, "static_assert(offsetof(myStruct, explicitValues) == 0);");

        String ts = parseToTs(definitions);
        assertContains(ts, "explicitValues = array, F32, 0, [8x4], \"\", 1, 0, 0, 0, 0\n");
    }

    @Test
    public void squareTablesAndExplicitArraysKeepTheirShape() throws IOException {
        String definitions =
                "struct_no_prefix myStruct\n" +
                        "begin_table\n" +
                        "table_rows num 8 uint8_t rowBins\n" +
                        "table_cols num 8 uint8_t colBins\n" +
                        "table_values uint8_t tableVals\n" +
                        "end_table\n" +
                        "uint8_t[8 x 8] explicitValues\n" +
                        "end_struct\n";

        String c = parseToC(definitions);
        assertContains(c, "\tuint8_t tableVals[8][8];");
        assertContains(c, "\tuint8_t explicitValues[8][8];");
        assertContains(c, "static_assert(offsetof(myStruct, tableVals) == 16);");
        assertContains(c, "static_assert(offsetof(myStruct, explicitValues) == 80);");

        String ts = parseToTs(definitions);
        assertContains(ts, "tableVals = array, U08, 16, [8x8], \"\", 1, 0, 0, 0, 0\n");
        assertContains(ts, "explicitValues = array, U08, 80, [8x8], \"\", 1, 0, 0, 0, 0\n");
    }

    @Test
    public void multiValueTableUnionMembersUseTableStorageOrderInC() throws IOException {
        String definitions =
                "struct_no_prefix myStruct\n" +
                        "begin_table\n" +
                        "table_rows num 4 uint16_t rowBins\n" +
                        "table_cols num 8 uint16_t colBins\n" +
                        "table_values float tableFloat\n" +
                        "table_values uint32_t tableRaw\n" +
                        "end_table\n" +
                        "end_struct\n";

        String c = parseToC(definitions);
        assertContains(c, "\tfloat tableFloat[4][8];");
        assertContains(c, "\tuint32_t tableRaw[4][8];");

        String ts = parseToTs(definitions);
        assertContains(ts, "tableFloat = array, F32, 24, [8x4], \"\", 1, 0, 0, 0, 0\n");
        assertContains(ts, "tableRaw = array, U32, 24, [8x4], \"\", 1, 0, 0, 0, 0\n");
    }

    private static void assertContains(String actual, String expected) {
        Assert.assertTrue(
                "Expected to find:\n" + expected + "\nIn:\n" + actual,
                actual.contains(expected));
    }
}
