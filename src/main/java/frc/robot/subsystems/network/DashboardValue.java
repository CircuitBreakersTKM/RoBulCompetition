package frc.robot.subsystems.network;

import java.util.ArrayList;

import edu.wpi.first.networktables.*;

/**
 * Generic wrapper for NetworkTables values that provides type-safe access
 * and automatic default value management. Maintains a static list of all
 * dashboard values for bulk initialization.
 * 
 * @param <T> The type of value stored (Boolean, Double, String, etc.)
 */
public class DashboardValue<T extends Object> {
    private final String key;
    private final T defaultValue;
    private final Class<? extends Object> type;
    private final NetworkTableEntry entry;

    public static ArrayList<DashboardValue<? extends Object>> values = new ArrayList<>();

    /**
     * Creates a new DashboardValue with a specific NetworkTable.
     * 
     * @param table The NetworkTable to publish to
     * @param key The key/name of the entry
     * @param defaultValue The default value (also determines type)
     * @param classType The class type for type checking
     */
    public DashboardValue(NetworkTable table, String key, T defaultValue, Class<? extends Object> classType) {
        if (defaultValue == null) throw new IllegalArgumentException("defaultValue cannot be null");

        this.key = key;
        this.defaultValue = defaultValue;
        this.type = classType;
        this.entry = table.getEntry(key);

        values.add(this);
    }

    /**
     * Creates a new DashboardValue on the default "CircuitBreakers" NetworkTable.
     * 
     * @param key The key/name of the entry
     * @param defaultValue The default value (also determines type)
     */
    public DashboardValue(String key, T defaultValue) {
        this(NetworkTableInstance.getDefault().getTable("CircuitBreakers"), key, defaultValue, defaultValue.getClass());
    }

    /**
     * Gets the current value from NetworkTables, or default if not set.
     * 
     * @return The current value
     */
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

    /**
     * Sets a new value to NetworkTables.
     * 
     * @param value The new value to set
     */
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

    /**
     * Resets the value to its default.
     */
    public void setDefault() { set(defaultValue); }

    /**
     * Gets the key name of this dashboard value.
     * 
     * @return The key string
     */
    public String key() { return key; }
    
    /**
     * Gets the default value.
     * 
     * @return The default value
     */
    public T defaultValue() { return defaultValue; }
}
