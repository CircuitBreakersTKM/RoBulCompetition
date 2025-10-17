package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.*;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class RobotContainer {
    private static RobotContainer instance = null;
    public enum AutoMode {
        NONE,
        DRIVE,
        CENTER_WHEELS,
        PIDF_TUNER,
        CRAB_WALK
    }

    public final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();

    private final XboxController controller = new XboxController(0);
    private SwerveDrive swerveDrive;
    private double prevSpeedX = 0;
    private double prevSpeedY = 0;
    private double prevRot = 0;

    private double maxSpeed = 3;
    private double minSpeed = 1.5;

    private SwerveModule[] swerveModules;

    private PowerDistribution pdh = new PowerDistribution(10, PowerDistribution.ModuleType.kRev);
    
    public RobotContainer() {
        autoModeChooser.setDefaultOption("Drive", AutoMode.DRIVE);
        autoModeChooser.addOption("None", AutoMode.NONE);
        autoModeChooser.addOption("Crab Walk", AutoMode.CRAB_WALK);
        autoModeChooser.addOption("Center Wheels", AutoMode.CENTER_WHEELS);
        autoModeChooser.addOption("PIDF Tuner", AutoMode.PIDF_TUNER);

        SmartDashboard.putData("Auto Mode", autoModeChooser);
        SmartDashboard.putNumber("Turn sensitivity", 0.7);
        SmartDashboard.putNumber("Joystick deadzone", 0.1);        
        SmartDashboard.putNumber("Max speed (m/s)", maxSpeed);
        SmartDashboard.putNumber("Max speed (m/s)", 3);
        SmartDashboard.putNumber("Max angular velocity (rad/s)", Math.PI);

        if (instance != null) {
            instance.close();
        }
        instance = this;

        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        // Initialize the swerve drive using the configuration files
        swerveDrive = null;

        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(10);
            swerveDrive.useInternalFeedbackSensor();
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        } catch (Exception e) {
            System.out.println("Error initializing SwerveDrive: " + e.getMessage());
        }

        swerveModules = swerveDrive.getModules();
    }
    public void teleopInit() {
        double voltage = pdh.getVoltage();
        double speed = maxSpeed;

        if (voltage < 12.0 && voltage > 11.5) {
            double k = (voltage - 11.5) / 0.5;
            speed = minSpeed + k * (maxSpeed - minSpeed);
        }
        else if (voltage <= 11.5) {
            speed = minSpeed;
        }

        SmartDashboard.putNumber("Max speed (m/s)", speed);
    }
    public void processManualInput() {

        switch (autoModeChooser.getSelected()) {
            case DRIVE:
                double RightX = controller.getRightX() * SmartDashboard.getNumber("Turn sensitivity", 0.7);
                
                double LeftX = controller.getLeftX();
                double LeftY = controller.getLeftY();

                double deadzone = SmartDashboard.getNumber("Joystick deadzone", 0.1);

                if (Math.abs(LeftX) < deadzone) {
                    LeftX = 0;
                }
                if (Math.abs(LeftY) < deadzone) {
                    LeftY = 0;
                }
                if (Math.abs(RightX) < deadzone) {
                    RightX = 0;
                }

                double rot = SmartDashboard.getNumber("Max angular velocity (rad/s)", Math.PI/2)*RightX*RightX;
                if (RightX < 0) {
                    rot = -rot;
                }

                double speedX = -SmartDashboard.getNumber("Max speed (m/s)", 3.0)*LeftX*LeftX;
                if (LeftX < 0) {
                    speedX = -speedX;
                    
                }

                double speedY = -SmartDashboard.getNumber("Max speed (m/s)", 3.0)*LeftY*LeftY;
                if (LeftY < 0) {
                    speedY = -speedY;
                }

                if (Math.abs(prevSpeedX) > Math.abs(speedX)) {
                    prevSpeedX = speedX;
                }
                else {  
                    prevSpeedX += (speedX - prevSpeedX) / 35;
                }

                if (Math.abs(prevSpeedY) > Math.abs(speedY)) {
                    prevSpeedY = speedY;
                }
                else {  
                    prevSpeedY += (speedY - prevSpeedY) / 35;
                }

                if (Math.abs(prevRot) > Math.abs(rot)) {
                    prevRot = rot;
                }
                else {  
                    prevRot += (rot - prevRot) / 15;
                }

                swerveDrive.drive(new Translation2d(prevSpeedY, prevSpeedX), -prevRot, false, true);
            break;
            case CENTER_WHEELS:
                SwerveDriveTest.centerModules(swerveDrive);
            break;
            case PIDF_TUNER:
                for (SwerveModule module : swerveModules) {
                    module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false, false);
                }
            case CRAB_WALK:
                double speed = controller.getRightTriggerAxis();
                if (controller.getAButton()) {
                    swerveDrive.drive(new Translation2d(-1.0 * speed, 0.0), 0.0, false, false);
                }
                else if (controller.getBButton()) {
                    swerveDrive.drive(new Translation2d(0.0, -1.0 * speed), 0.0, false, false);
                }
                else if (controller.getXButton()) {
                    swerveDrive.drive(new Translation2d(0.0, 1.0 * speed), 0.0, false, false);
                }
                else if (controller.getYButton()) {
                    swerveDrive.drive(new Translation2d(1.0 * speed, 0.0), 0.0, false, false);
                }
                else {
                    swerveDrive.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
                }
            default:
                break;
        }
    }

    public void close() {
    }
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    } //53.142857142857
}
