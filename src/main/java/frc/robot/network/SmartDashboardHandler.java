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

    // CONTROLLER parameters
    public static final DashboardValue<Double> TURN_SENSITIVITY = new DashboardValue<>(
        "Controller/Turn sensitivity", 0.7);
    public static final DashboardValue<Double> JOYSTICK_DEADZONE = new DashboardValue<>(
        "Controller/Joystick deadzone", 0.1);
    public static final DashboardValue<Double> TRIGGER_AXIS_DEADZONE = new DashboardValue<>(
        "Controller/Trigger axis deadzone", 0.05);
    public static final DashboardValue<Double> TURN_SENSITIVITY_FLOAT = new DashboardValue<>(
        "Controller/Turn sensitivity float", 0.7);    

        
    // CHASSIS parameters
    public static final DashboardValue<Double> MAX_SPEED = new DashboardValue<>(
        "Chassis/Max speed (ms^-1)", 4.8);
    public static final DashboardValue<Double> MAX_ANGULAR_SPEED = new DashboardValue<>(
        "Chassis/Max angular speed (rads^-1)", 4*Math.PI);
    public static final DashboardValue<Double> MAX_ACCELERATION = new DashboardValue<>(
        "Chassis/Max acceleration (ms^-2)", 3.0);
    public static final DashboardValue<Double> MAX_ANGULAR_ACCELERATION = new DashboardValue<>(
        "Chassis/Max angular acceleration (rads^-2)", 2*Math.PI);

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
