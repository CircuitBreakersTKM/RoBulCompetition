package frc.robot.network;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardHandler {
    public static enum AutoMode {
        NONE,
        DRIVE,
        CENTER_WHEELS,
        CRAB_WALK
    }

    public static final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();

    // JOYSTICK parameters
    public static final DashboardValue<Double> TURN_SENSITIVITY = new DashboardValue<>(
        "CircuitBreakers/Joystick/Turn sensitivity", 0.7, Double.class);
    public static final DashboardValue<Double> JOYSTICK_DEADZONE = new DashboardValue<>(
        "CircuitBreakers/Joystick/Joystick deadzone", 0.1, Double.class);
    
    // CHASSIS parameters
    public static final DashboardValue<Double> MAX_SPEED = new DashboardValue<>(
        "CircuitBreakers/Chassis/Max speed (ms^-1)", 4.8, Double.class);
    public static final DashboardValue<Double> MAX_ANGULAR_SPEED = new DashboardValue<>(
        "CircuitBreakers/Chassis/Max angular speed (rads^-1)", Math.PI, Double.class);
    public static final DashboardValue<Double> MAX_ACCELERATION = new DashboardValue<>(
        "CircuitBreakers/Chassis/Max acceleration (ms^-2)", 3.0, Double.class);
    public static final DashboardValue<Double> MAX_ANGULAR_ACCELERATION = new DashboardValue<>(
        "CircuitBreakers/Chassis/Max angular acceleration (rads^-2)", Math.PI, Double.class);

    public static void Init() {
        autoModeChooser.setDefaultOption("Drive", AutoMode.DRIVE);
        autoModeChooser.addOption("None", AutoMode.NONE);
        autoModeChooser.addOption("Crab Walk", AutoMode.CRAB_WALK);
        autoModeChooser.addOption("Center Wheels", AutoMode.CENTER_WHEELS);
        SmartDashboard.putData("Auto Mode", SmartDashboardHandler.autoModeChooser);

        for (DashboardValue<?> value : DashboardValue.values) {
            value.setDefault();
        }
    }
}
