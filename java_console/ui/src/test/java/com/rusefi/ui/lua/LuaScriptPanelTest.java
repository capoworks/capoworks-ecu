package com.rusefi.ui.lua;

import com.opensr5.ini.IniFileReader;
import com.opensr5.ini.RawIniFile;
import com.opensr5.ini.field.IniField;
import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

public class LuaScriptPanelTest {
    private static final String ECU_SIGNATURE = "ecu-signature";
    private static final String LUA_SCRIPT_FIELD = "luaScript = string, ASCII, 137, 29";

    @Test
    public void matchingSignatureResolvesRawIniField() {
        IniField field = resolve(withSignature(ECU_SIGNATURE), ECU_SIGNATURE, 166);

        assertNotNull(field);
        assertEquals("luaScript", field.getName());
        assertEquals(137, field.getOffset());
        assertEquals(29, field.getSize());
    }

    @Test
    public void missingLuaScriptFieldIsUnavailable() {
        assertRejected(parse("signature = \"" + ECU_SIGNATURE + "\""), ECU_SIGNATURE, 1024);
    }

    @Test
    public void nullEcuSignatureIsRejected() {
        assertRejected(withSignature(ECU_SIGNATURE), null, 1024);
    }

    @Test
    public void emptyEcuSignatureIsRejected() {
        assertRejected(withSignature(ECU_SIGNATURE), "", 1024);
    }

    @Test
    public void nullFileSignatureIsRejected() {
        assertRejected(parse(LUA_SCRIPT_FIELD), ECU_SIGNATURE, 1024);
    }

    @Test
    public void emptyFileSignatureIsRejected() {
        assertRejected(withSignature(""), ECU_SIGNATURE, 1024);
    }

    @Test
    public void mismatchedCurrentRawFileSignatureIsRejected() {
        assertRejected(withSignature("other-signature"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void negativeOffsetIsRejected() {
        assertRejected(withField("luaScript = string, ASCII, -1, 10"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void offsetOutsideImageIsRejected() {
        assertRejected(withField("luaScript = string, ASCII, 101, 1"), ECU_SIGNATURE, 100);
    }

    @Test
    public void zeroSizeIsRejected() {
        assertRejected(withField("luaScript = string, ASCII, 10, 0"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void negativeSizeIsRejected() {
        assertRejected(withField("luaScript = string, ASCII, 10, -1"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void overflowSizeIsRejected() {
        assertRejected(withField("luaScript = string, ASCII, 10, 2147483647"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void nonAsciiStringFieldIsUnavailable() {
        assertRejected(withField("luaScript = string, UTF-8, 10, 10"), ECU_SIGNATURE, 1024);
    }

    @Test
    public void nonStringFieldIsUnavailable() {
        assertRejected(withField("luaScript = array, ASCII, 10, 10"), ECU_SIGNATURE, 1024);
    }

    private static RawIniFile withSignature(String signature) {
        return parse("signature = \"" + signature + "\"", LUA_SCRIPT_FIELD);
    }

    private static RawIniFile withField(String luaScriptField) {
        return parse("signature = \"" + ECU_SIGNATURE + "\"", luaScriptField);
    }

    private static RawIniFile parse(String... lines) {
        return IniFileReader.read(new ByteArrayInputStream(
                String.join("\n", lines).getBytes(StandardCharsets.US_ASCII)));
    }

    private static void assertRejected(RawIniFile ini, String connectedSignature, int imageSize) {
        assertNull(resolve(ini, connectedSignature, imageSize));
    }

    private static IniField resolve(RawIniFile ini, String connectedSignature, int imageSize) {
        return LuaScriptPanel.resolveLuaScriptField(ini, connectedSignature, imageSize);
    }
}
