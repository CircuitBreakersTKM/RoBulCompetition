package frc.robot.network;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardValue<T> {
    private final String key;
    private final T defaultValue;
    private final Class<T> type;

    public static ArrayList<DashboardValue<?>> values = new ArrayList<>();

    public DashboardValue(String key, T defaultValue, Class<T> type) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.type = type;

        values.add(this);
    }

    public T get() {
        return switch (type.getSimpleName()) {
            case "Double" -> type.cast(SmartDashboard.getNumber(key, (Double) defaultValue));
            case "Boolean" -> type.cast(SmartDashboard.getBoolean(key, (Boolean) defaultValue));
            case "String" -> type.cast(SmartDashboard.getString(key, (String) defaultValue));
            default -> throw new UnsupportedOperationException("Unsupported type: " + type);
        };
    }

    public void set(T value) {
        switch (type.getSimpleName()) {
            case "Double" -> SmartDashboard.putNumber(key, (Double) value);
            case "Boolean" -> SmartDashboard.putBoolean(key, (Boolean) value);
            case "String" -> SmartDashboard.putString(key, (String) value);
            default -> throw new UnsupportedOperationException("Unsupported type: " + type);
        }
    }

    public void setDefault() {
        set(defaultValue);
    }

    public String key() { return key; }
    public T defaultValue() { return defaultValue; }
}
