package frc.robot.subsystems.network;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkSubsystem {
    public static enum AutoMode {
        NONE,
        JOYSTICK_DRIVE,
        CENTER_WHEELS,
        CRAB_WALK
    }

    public static final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();

    // CONTROLLER parameters
    public static final DashboardValue<Double> TURN_SENSITIVITY = new DashboardValue<>(
        "Controller/Turn sensitivity", 0.7);
    public static final DashboardValue<Double> JOYSTICK_DEADZONE = new DashboardValue<>(
        "Controller/Joystick deadzone", 0.07);
    public static final DashboardValue<Double> TRIGGER_AXIS_DEADZONE = new DashboardValue<>(
        "Controller/Trigger axis deadzone", 0.05);

        
    // CHASSIS parameters
    public static final DashboardValue<Double> MAX_SPEED = new DashboardValue<>(
        "Chassis/Max speed (ms^-1)", 2.5);
    public static final DashboardValue<Double> MAX_ANGULAR_SPEED = new DashboardValue<>(
        "Chassis/Max angular speed (rads^-1)", 1.8);
    public static final DashboardValue<Double> MAX_ACCELERATION = new DashboardValue<>(
        "Chassis/Max acceleration (ms^-2)", 3.0);
    public static final DashboardValue<Double> MAX_ANGULAR_ACCELERATION = new DashboardValue<>(
        "Chassis/Max angular acceleration (rads^-2)", 2*Math.PI);
    public static final DashboardValue<Boolean> ZERO_ANGLE = new DashboardValue<>(
        "Chassis/Zero angle", false);

    // LASER TURRET parameters
    public static final DashboardValue<Double> LASER_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Laser Turret/Motor speed", 0.25);

    // CAMERA TOWER parameters
    public static final DashboardValue<Double> CAMERA_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Camera Tower/Motor speed", 0.25);

    

    public static void Init() {
        autoModeChooser.setDefaultOption("Joystick Drive", AutoMode.JOYSTICK_DRIVE);
        autoModeChooser.addOption("None", AutoMode.NONE);
        autoModeChooser.addOption("Crab Walk", AutoMode.CRAB_WALK);
        autoModeChooser.addOption("Center Wheels", AutoMode.CENTER_WHEELS);
        SmartDashboard.putData("Auto Mode", NetworkSubsystem.autoModeChooser);

        for (DashboardValue<?> value : DashboardValue.values) {
            value.setDefault();
        }
    }
}
