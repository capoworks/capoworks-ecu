package com.rusefi.newparse.parsing;

public class ScalarField extends PrototypeField {
    public final Type type;
    public final FieldOptions options;
    public final Boolean autoscale;
    public final boolean autotemp;
    public final boolean tableValues;

    public ScalarField(Type type, String name, FieldOptions options, boolean autoscale, boolean autotemp) {
        this(type, name, options, autoscale, autotemp, false);
    }

    public ScalarField(Type type, String name, FieldOptions options, boolean autoscale, boolean autotemp, boolean tableValues) {
        super(name);

        this.type = type;
        this.options = options;
        this.autoscale = autoscale;
        this.autotemp = autotemp;
        this.tableValues = tableValues;
    }

    @Override
    public String toString() {
        return type.cType + " " + name;
    }
}
