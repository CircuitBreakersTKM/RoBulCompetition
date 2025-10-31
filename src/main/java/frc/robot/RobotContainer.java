package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.laser.LaserTurret;
import frc.robot.network.NetworkHandler;
import frc.robot.network.NetworkHandler.AutoMode;
import frc.robot.swerve.SwerveDriveSubsystem;
import swervelib.SwerveDriveTest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final LaserTurret laserTurret = new LaserTurret(11, 12);

    private AutoMode lastMode = NetworkHandler.autoModeChooser.getSelected();

    private boolean moving = false;
    
    public RobotContainer() {
        if (instance != null) {
            instance.close();
        }
        else {
            NetworkHandler.Init();
        }

        try {
            swerveDriveSubsystem = new SwerveDriveSubsystem();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize SwerveDriveSubsystem", e);
        }
        instance = this;
    }
    public void teleopInit() {
        swerveDriveSubsystem.applyLowBatteryLimiters();
    }
    public void OnLastModeChange(AutoMode lastAutoMode, AutoMode newAutoMode) {
    }
    public void processManualInput() {
        AutoMode currentMode = NetworkHandler.autoModeChooser.getSelected();
        if (lastMode != currentMode) {
            OnLastModeChange(lastMode, currentMode);
            lastMode = NetworkHandler.autoModeChooser.getSelected();
        }

        if (NetworkHandler.ZERO_ANGLE.get()) {
            swerveDriveSubsystem.zeroGyro();
            NetworkHandler.ZERO_ANGLE.set(false);
        }

        switch (currentMode) {
            case DRIVE_LASER: {
                // Get joystick inputs
                double rot = controller.getLeftTriggerAxis() - controller.getRightTriggerAxis();
                double speedX = controller.getLeftX();
                double speedY = controller.getLeftY();
                
                
                // Apply deadzone
                double deadzone = NetworkHandler.JOYSTICK_DEADZONE.get();

                double laserAltitudeSpeed = MathUtil.applyDeadband(controller.getRightY(), deadzone);
                double laserAzimuthSpeed = -MathUtil.applyDeadband(controller.getRightX(), deadzone);

                rot = MathUtil.applyDeadband(rot, NetworkHandler.TRIGGER_AXIS_DEADZONE.get());
                speedX = MathUtil.applyDeadband(speedX, deadzone);
                speedY = MathUtil.applyDeadband(speedY, deadzone);

                // Drive the swerve drive
                swerveDriveSubsystem.processCarteseanInput(new Translation2d(-speedX, -speedY), rot, true, true);
                laserTurret.setSpeeds(laserAzimuthSpeed, laserAltitudeSpeed);
                break;
            }
            case CENTER_WHEELS: {
                // Center all swerve modules
                SwerveDriveTest.centerModules(swerveDriveSubsystem.swerveDrive);
                break;
            }
            case CRAB_WALK: {
                // Get speed and POV from controller
                double speed = MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkHandler.TRIGGER_AXIS_DEADZONE.get());
                double pov = controller.getPOV();

                // Calculate rotation input
                double rot = MathUtil.applyDeadband(controller.getRightX(), NetworkHandler.JOYSTICK_DEADZONE.get());

                if (pov % 90 != 0) {
                    pov = -1;
                }

                if (pov != -1 && speed > 0) {
                    pov *= -1;

                    // Calculate crab walk translation based on POV angle
                    double angleRad = Math.toRadians(pov);

                    moving = true;
                    swerveDriveSubsystem.processPolarInput(speed, angleRad, -rot, false, false);
                } else if (pov != -1 && rot == 0) {
                    pov *= -1;
                    
                    swerveDriveSubsystem.stopIf(moving);
                    //SwerveDriveTest.angleModules(swerveDriveSubsystem.swerveDrive, Rotation2d.fromDegrees(pov % 180));
                    moving = false;
                } else if (rot != 0) {
                    moving = true;

                    swerveDriveSubsystem.processCarteseanInput(new Translation2d(0, 0), -rot, false, false);
                } else {
                    swerveDriveSubsystem.stopIf(moving);
                    //SwerveDriveTest.centerModules(swerveDriveSubsystem.swerveDrive);
                    moving = false;
                }
                break;
            }
            default:
                swerveDriveSubsystem.stopIf(true);
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
    }
}
