package frc.robot.network;

import java.util.ArrayList;

import edu.wpi.first.networktables.*;

public class DashboardValue<T> {
    private final String key;
    private final T defaultValue;
    private final Class<?> type;
    private final NetworkTableEntry entry;

    public static ArrayList<DashboardValue<?>> values = new ArrayList<>();

    public DashboardValue(String key, T defaultValue, Class<?> classType) {
        if (defaultValue == null) throw new IllegalArgumentException("defaultValue cannot be null");

        this.key = key;
        this.defaultValue = defaultValue;
        this.type = classType;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("CircuitBreakers");
        this.entry = table.getEntry(key);

        values.add(this);
    }

    public DashboardValue(String key, T defaultValue) {
        this(key, defaultValue, defaultValue.getClass());
    }

    @SuppressWarnings("unchecked")
    public T get() {
        return switch (type.getSimpleName()) {
            case "Boolean" -> (T) Boolean.valueOf(entry.getBoolean((Boolean) defaultValue));
            case "Long" -> (T) Long.valueOf(entry.getInteger((Integer) defaultValue));
            case "Float" -> (T) Float.valueOf(entry.getNumber((Double) defaultValue).floatValue());
            case "Double" -> (T) Double.valueOf(entry.getDouble((Double) defaultValue));
            case "Number" -> (T) entry.getNumber((Number) defaultValue);
            case "String" -> (T) entry.getString((String) defaultValue);

            case "Boolean[]" -> (T) entry.getBooleanArray((Boolean[]) defaultValue);
            case "Long[]" -> (T) entry.getIntegerArray((Long[]) defaultValue);
            case "Float[]" -> (T) entry.getNumberArray((Float[]) defaultValue);
            case "Double[]" -> (T) entry.getDoubleArray((Double[]) defaultValue);
            case "Number[]" -> (T) entry.getNumberArray((Number[]) defaultValue);
            case "String[]" -> (T) entry.getStringArray((String[]) defaultValue);

            case "byte[]" -> (T) entry.getRaw((byte[]) defaultValue);
            default -> throw new UnsupportedOperationException("Unsupported type: " + type);
        };
    }

    public void set(T value) {
        switch (type.getSimpleName()) {
            case "Boolean" -> entry.setBoolean((Boolean) value);
            case "Long" -> entry.setInteger((Long) value);
            case "Float" -> entry.setNumber(((Float) value).doubleValue());
            case "Double" -> entry.setDouble((Double) value);
            case "Number" -> entry.setNumber((Number) value);
            case "String" -> entry.setString((String) value);

            case "Boolean[]" -> entry.setBooleanArray((Boolean[]) value);
            case "Long[]" -> entry.setIntegerArray((Long[]) value);
            case "Float[]" -> entry.setNumberArray((Float[]) value);
            case "Double[]" -> entry.setDoubleArray((Double[]) value);
            case "Number[]" -> entry.setNumberArray((Number[]) value);
            case "String[]" -> entry.setStringArray((String[]) value);

            case "byte[]" -> entry.setRaw((byte[]) value);
            default -> throw new UnsupportedOperationException("Unsupported type: " + type);
        }
    }

    public void setDefault() { set(defaultValue); }

    public String key() { return key; }
    public T defaultValue() { return defaultValue; }
}
