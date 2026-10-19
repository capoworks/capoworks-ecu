package com.rusefi.newparse.outputs;

import com.rusefi.newparse.ParseState;
import com.rusefi.newparse.layout.ArrayLayout;
import com.rusefi.newparse.layout.BitGroupLayout;
import com.rusefi.newparse.layout.EnumLayout;
import com.rusefi.newparse.layout.Layout;
import com.rusefi.newparse.layout.ScalarLayout;
import com.rusefi.newparse.layout.StringLayout;
import com.rusefi.newparse.layout.StructNamePrefixer;
import com.rusefi.newparse.layout.StructLayout;
import com.rusefi.newparse.layout.UnionLayout;
import com.rusefi.newparse.layout.UnusedLayout;
import com.rusefi.newparse.parsing.EnumValues;
import com.rusefi.newparse.parsing.FieldOptions;
import com.rusefi.newparse.parsing.Struct;
import com.rusefi.newparse.parsing.Type;

import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;

/**
 * Builds the persistent-layout descriptor and its build-time identifier.
 *
 * <p>The descriptor is a versioned, canonical UTF-8 record stream. Each record is
 * {@code <record-byte-length>:<tag>|<field-byte-length>:<field>|...\\n}. The body length
 * excludes the record-terminating newline. Field values are length-prefixed so names and enum
 * labels containing delimiters remain unambiguous. This is a deliberate simplification over the
 * JSON design in {@code tune_policy.json}: it has the same field coverage without a JSON
 * dependency, object mapper, or reflection.
 *
 * <p>Version 1 records describe only the {@code persistent_config_s} storage graph: the fixed
 * little-endian storage ABI, root size/alignment, referenced structures, physical scalar types,
 * field order/offsets, bit positions, array shapes, numeric scale/offset in the internal units,
 * and numeric-to-name mappings for referenced enums. Comments, UI labels, defaults, autoscale,
 * autotemp, unrelated definitions/types, and outputs are intentionally absent.
 */
public final class PersistentLayoutIdentity {
    public static final int DESCRIPTOR_VERSION = 1;
    public static final String ROOT_STRUCT_NAME = "persistent_config_s";

    private static final int[] NO_ARRAY_DIMS = new int[0];
    private static final char[] HEX = "0123456789abcdef".toCharArray();

    private PersistentLayoutIdentity() {
    }

    /**
     * Returns the first eight SHA-256 digest bytes as a big-endian unsigned C constant, for example
     * {@code 0x0123456789abcdefULL}.
     */
    public static String layoutId(ParseState parseState) {
        byte[] descriptor = canonicalDescriptor(parseState).getBytes(StandardCharsets.UTF_8);

        final byte[] digest;
        try {
            digest = MessageDigest.getInstance("SHA-256").digest(descriptor);
        } catch (NoSuchAlgorithmException e) {
            // SHA-256 is required by the Java platform.
            throw new IllegalStateException("SHA-256 is unavailable", e);
        }

        char[] result = new char[2 + 16 + 3];
        result[0] = '0';
        result[1] = 'x';
        for (int i = 0; i < 8; i++) {
            int value = digest[i] & 0xff;
            result[2 + i * 2] = HEX[value >>> 4];
            result[3 + i * 2] = HEX[value & 0x0f];
        }
        result[18] = 'U';
        result[19] = 'L';
        result[20] = 'L';
        return new String(result);
    }

    /**
     * Returns the canonical descriptor encoded as version 1 UTF-8 records.
     *
     * <p>The last parsed structure named {@code persistent_config_s} is used as the root. The
     * caller should normally invoke this immediately after parsing the configuration definition.
     */
    public static String canonicalDescriptor(ParseState parseState) {
        Struct root = findRoot(parseState);
        StructLayout rootLayout = new StructLayout(0, "root", root);

        IdentityVisitor visitor = new IdentityVisitor();
        rootLayout.visit(visitor, null, null, 0, NO_ARRAY_DIMS);
        return visitor.finish();
    }

    /**
     * Returns whether the parsed definition contains the persistent configuration root.
     */
    public static boolean hasPersistentConfigRoot(ParseState parseState) {
        return findRootOrNull(parseState) != null;
    }

    private static Struct findRoot(ParseState parseState) {
        Struct root = findRootOrNull(parseState);
        if (root == null) {
            throw new IllegalStateException("No " + ROOT_STRUCT_NAME + " structure was parsed");
        }
        return root;
    }

    private static Struct findRootOrNull(ParseState parseState) {
        if (parseState == null) {
            throw new IllegalArgumentException("parseState must not be null");
        }

        List<Struct> structs = parseState.getStructs();
        for (int i = structs.size() - 1; i >= 0; i--) {
            Struct candidate = structs.get(i);
            if (ROOT_STRUCT_NAME.equals(candidate.name)) {
                return candidate;
            }
        }
        return null;
    }

    private static final class IdentityVisitor extends ILayoutVisitor {
        private final StringBuilder descriptor = new StringBuilder();
        private final Deque<String> namespace = new ArrayDeque<>();
        private int structDepth;

        private IdentityVisitor() {
            record("descriptor", Integer.toString(DESCRIPTOR_VERSION), "utf8-records-v1");
            record("storage_abi",
                    "little-endian",
                    "u8=1,s8=1,u16=2,s16=2,u32=4,s32=4,f32=4",
                    "integer-alignment=width",
                    "struct-alignment=4");
        }

        private String finish() {
            return descriptor.toString();
        }

        @Override
        public void visit(StructLayout struct, PrintStream ps, StructNamePrefixer prefixer,
                          int offsetAdd, int[] arrayDims) {
            boolean isRoot = structDepth == 0;
            int structOffset = offsetAdd + struct.offset;
            String structName = isRoot ? struct.typeName : qualified(struct.name);

            if (isRoot) {
                record("root", struct.typeName, integer(struct.getSize()), integer(struct.getAlignment()),
                        integer(struct.children.size()));
            } else if (arrayDims.length != 0) {
                record("struct_array", structName, struct.typeName, integer(structOffset),
                        integer(struct.getSize()), integer(struct.getAlignment()), shape(arrayDims),
                        integer(struct.children.size()));
            } else {
                record("struct", structName, struct.typeName, integer(structOffset),
                        integer(struct.getSize()), integer(struct.getAlignment()),
                        integer(struct.children.size()));
            }

            namespace.addLast(isRoot ? struct.typeName : struct.name);
            structDepth++;

            // Struct-array prototypes are laid out at offset zero. Other nested structs already
            // contain absolute child offsets because StructLayout was constructed at their offset.
            int childOffsetAdd = arrayDims.length != 0 ? structOffset : offsetAdd;
            for (Layout child : orderedChildren(struct.children)) {
                child.visit(this, ps, prefixer, childOffsetAdd, NO_ARRAY_DIMS);
            }

            structDepth--;
            namespace.removeLast();
            record(isRoot ? "end_root" : "end_struct", structName);
        }

        @Override
        public void visit(ArrayLayout array, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            array.prototypeLayout.visit(this, ps, prefixer, offsetAdd, array.length);
        }

        @Override
        public void visit(ScalarLayout scalar, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            FieldOptions options = scalar.options;
            record("scalar", qualified(scalar.name), integer(offsetAdd + scalar.offset),
                    integer(scalar.getSize()), integer(scalar.getAlignment()), physicalType(scalar.type),
                    number(options.scale), number(options.offset), unit(options.units), shape(arrayDims));
        }

        @Override
        public void visit(EnumLayout enumLayout, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            String fieldName = qualified(enumLayout.name);
            FieldOptions options = enumLayout.options;

            record("enum", fieldName, integer(offsetAdd + enumLayout.offset),
                    integer(enumLayout.getSize()), integer(enumLayout.getAlignment()),
                    physicalType(enumLayout.type), enumLayout.enumType,
                    integer(enumLayout.endBit), number(options.scale), number(options.offset),
                    unit(options.units), shape(arrayDims));

            EnumValues values = enumLayout.values;
            for (int i = 0; i < values.names.length; i++) {
                record("enum_value", fieldName, integer(i), values.names[i]);
            }
            record("end_enum", fieldName);
        }

        @Override
        public void visit(StringLayout string, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            record("string", qualified(string.name), integer(offsetAdd + string.offset),
                    integer(string.getSize()), integer(string.getAlignment()), "ascii8",
                    shape(arrayDims));
        }

        @Override
        public void visit(BitGroupLayout bitGroup, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            int bitGroupOffset = offsetAdd + bitGroup.offset;
            record("bit_group", integer(bitGroupOffset), integer(bitGroup.getSize()),
                    integer(bitGroup.getAlignment()), integer(bitGroup.bits.size()), shape(arrayDims));

            for (int bit = 0; bit < bitGroup.bits.size(); bit++) {
                BitGroupLayout.BitLayout bitLayout = bitGroup.bits.get(bit);
                record("bit", qualified(bitLayout.name), integer(bitGroupOffset), integer(bit), "1");
            }
            record("end_bit_group", integer(bitGroupOffset));
        }

        @Override
        public void visit(UnionLayout union, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            int unionOffset = offsetAdd + union.offset;
            record("union", integer(unionOffset), integer(union.getSize()),
                    integer(union.getAlignment()), integer(union.children.size()), shape(arrayDims));

            for (Layout child : orderedChildren(union.children)) {
                child.visit(this, ps, prefixer, offsetAdd, NO_ARRAY_DIMS);
            }
            record("end_union", integer(unionOffset));
        }

        @Override
        public void visit(UnusedLayout unused, PrintStream ps, StructNamePrefixer prefixer, int offsetAdd,
                          int[] arrayDims) {
            // Padding and explicit unused bytes contain no tune value. Their effect is captured by
            // field offsets and the root size.
        }

        private String qualified(String name) {
            StringBuilder result = new StringBuilder();
            for (String part : namespace) {
                if (result.length() != 0) {
                    result.append('.');
                }
                result.append(part);
            }
            if (result.length() != 0) {
                result.append('.');
            }
            return result.append(name).toString();
        }

        private void record(String tag, String... fields) {
            StringBuilder body = new StringBuilder(tag);
            for (String field : fields) {
                body.append('|')
                        .append(field.getBytes(StandardCharsets.UTF_8).length)
                        .append(':')
                        .append(field);
            }

            descriptor.append(body.toString().getBytes(StandardCharsets.UTF_8).length)
                    .append(':')
                    .append(body)
                    .append('\n');
        }
    }

    private static List<Layout> orderedChildren(List<Layout> children) {
        List<Layout> ordered = new ArrayList<>(children);
        ordered.sort(Comparator.comparingInt((Layout layout) -> layout.offset)
                .thenComparing(PersistentLayoutIdentity::layoutName));
        return ordered;
    }

    private static String layoutName(Layout layout) {
        if (layout instanceof ScalarLayout) {
            return ((ScalarLayout) layout).name;
        } else if (layout instanceof EnumLayout) {
            return ((EnumLayout) layout).name;
        } else if (layout instanceof StringLayout) {
            return ((StringLayout) layout).name;
        } else if (layout instanceof StructLayout) {
            return ((StructLayout) layout).name;
        } else if (layout instanceof ArrayLayout) {
            Layout prototype = ((ArrayLayout) layout).prototypeLayout;
            return layoutName(prototype);
        }
        return "";
    }

    private static String physicalType(Type type) {
        switch (type) {
            case U08:
                return "uint8";
            case S08:
                return "int8";
            case U16:
                return "uint16";
            case S16:
                return "int16";
            case U32:
                return "uint32";
            case S32:
            case I32:
                return "int32";
            case F32:
            case ANGLE_T:
                return "float32";
            default:
                throw new IllegalStateException("Unsupported storage type " + type);
        }
    }

    private static String shape(int[] dims) {
        StringBuilder result = new StringBuilder();
        for (int dim : dims) {
            if (result.length() != 0) {
                result.append('x');
            }
            result.append(dim);
        }
        return result.toString();
    }

    private static String unit(String units) {
        if (units == null || units.length() < 2) {
            return units == null ? "" : units;
        }
        if (units.charAt(0) == '"' && units.charAt(units.length() - 1) == '"') {
            return units.substring(1, units.length() - 1);
        }
        return units;
    }

    private static String number(double value) {
        // Double.toString is round-trippable and independent of source spelling. Normalize the two
        // zero representations because they have the same semantic numeric value.
        return value == 0.0d ? "0.0" : Double.toString(value);
    }

    private static String integer(int value) {
        return Integer.toString(value);
    }
}
