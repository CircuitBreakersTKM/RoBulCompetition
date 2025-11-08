package frc.robot.subsystems.network;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages NetworkTables dashboard values and auto mode selection.
 * Centralizes all tunable parameters for easy access and modification
 * through the driver station dashboard without redeploying code.
 */
public class NetworkSubsystem {
    /**
     * Available autonomous/teleop operating modes.
     */
    public static enum AutoMode {
        NONE,
        JOYSTICK_DRIVE,
        CENTER_WHEELS,
        CRAB_WALK,
        CAMERA_TOWER_TEST
    }

    /** SendableChooser for auto mode selection on dashboard */
    public static final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();

    // Controller parameters
    public static final DashboardValue<Double> TURN_SENSITIVITY = new DashboardValue<>(
        "Controller/Turn sensitivity", 0.7);
    public static final DashboardValue<Double> JOYSTICK_DEADZONE = new DashboardValue<>(
        "Controller/Joystick deadzone", 0.07);
    public static final DashboardValue<Double> TRIGGER_AXIS_DEADZONE = new DashboardValue<>(
        "Controller/Trigger axis deadzone", 0.05);

        
    // Chassis parameters
    public static final DashboardValue<Boolean> ZERO_ANGLE = new DashboardValue<>(
        "Chassis/Zero angle", false);
    public static final DashboardValue<Double> MAX_SPEED = new DashboardValue<>(
        "Chassis/Max speed (ms^-1)", 2.5);
    public static final DashboardValue<Double> MAX_ANGULAR_SPEED = new DashboardValue<>(
        "Chassis/Max angular speed (rads^-1)", 1.8);
    public static final DashboardValue<Boolean> OVERRIDE_LOW_VOLTAGE_LIMIERS = new DashboardValue<>(
        "Chassis/Advanced/Override low voltage limiters", false);

    // Chassis advanced parameters
    public static final DashboardValue<Double> MAX_ACCELERATION = new DashboardValue<>(
        "Chassis/Advanced/Max acceleration (ms^-2)", 3.0);
    public static final DashboardValue<Double> MAX_ANGULAR_ACCELERATION = new DashboardValue<>(
        "Chassis/Advanced/Max angular acceleration (rads^-2)", 2*Math.PI);

    // Laser turret parameters
    public static final DashboardValue<Double> LASER_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Laser Turret/Motor speed", 0.25);

    // Camera tower parameters
    public static final DashboardValue<Double> CAMERA_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Camera Tower/Motor speed", 0.25);

    /**
     * Initializes the NetworkSubsystem by setting up the auto mode chooser
     * and pushing all default values to NetworkTables.
     * Must be called once during robot initialization.
     */
    public static void Init() {
        autoModeChooser.setDefaultOption("Joystick Drive", AutoMode.JOYSTICK_DRIVE);
        autoModeChooser.addOption("None", AutoMode.NONE);
        autoModeChooser.addOption("Crab Walk", AutoMode.CRAB_WALK);
        autoModeChooser.addOption("Center Wheels", AutoMode.CENTER_WHEELS);
        autoModeChooser.addOption("Camera Tower Test", AutoMode.CAMERA_TOWER_TEST);
        SmartDashboard.putData("Auto Mode", NetworkSubsystem.autoModeChooser);

        for (DashboardValue<?> value : DashboardValue.values) {
            value.setDefault();
        }
    }
}
