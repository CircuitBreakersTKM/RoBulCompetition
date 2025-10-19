package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.MathHelper;
import frc.robot.math.SpeedRateLimiter;
import frc.robot.network.SmartDashboardHandler;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {
    private static RobotContainer instance = null;

    private final XboxController controller = new XboxController(0);
    private SwerveDrive swerveDrive;
    
    private double maxSpeed = 4.8;
    private double minSpeed = 2;

    private PowerDistribution pdh = new PowerDistribution(10, PowerDistribution.ModuleType.kRev);

    private SpeedRateLimiter xSpeedLimiter = new SpeedRateLimiter(3, 9, 0);
    private SpeedRateLimiter ySpeedLimiter = new SpeedRateLimiter(3, 9, 0);
    private SpeedRateLimiter rotLimiter = new SpeedRateLimiter(Math.PI, 3*Math.PI, 0);
    
    public RobotContainer() {
        SmartDashboardHandler.Init();

        if (instance != null) {
            instance.close();
        }
        instance = this;

        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

        // Initialize the swerve drive using the configuration files
        swerveDrive = null;

        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(4.8);
            swerveDrive.useInternalFeedbackSensor();
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        } catch (Exception e) {
            System.out.println("Error initializing SwerveDrive: " + e.getMessage());
        }
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

        SmartDashboardHandler.MAX_SPEED.set(speed);

        xSpeedLimiter = new SpeedRateLimiter(SmartDashboardHandler.MAX_ACCELERATION.get(), 9, 0);
        ySpeedLimiter = new SpeedRateLimiter(SmartDashboardHandler.MAX_ACCELERATION.get(), 9, 0);
        rotLimiter = new SpeedRateLimiter(SmartDashboardHandler.MAX_ANGULAR_ACCELERATION.get(), 3*Math.PI, 0);
    }
    public void processManualInput() {

        switch (SmartDashboardHandler.autoModeChooser.getSelected()) {
            case DRIVE: {
                // Get joystick inputs
                double RightX = controller.getRightX() * SmartDashboardHandler.TURN_SENSITIVITY.get();
                double LeftX = controller.getLeftX();
                double LeftY = controller.getLeftY();

                // Apply deadzone
                double deadzone = SmartDashboardHandler.JOYSTICK_DEADZONE.get();
                RightX = MathUtil.applyDeadband(RightX, deadzone);
                LeftX = MathUtil.applyDeadband(LeftX, deadzone);
                LeftY = MathUtil.applyDeadband(LeftY, deadzone);

                // Scale inputs and apply max speeds
                double rot = MathHelper.ScaleInput(RightX) * SmartDashboardHandler.MAX_ANGULAR_SPEED.get();
                double speedX = -MathHelper.ScaleInput(LeftX)*SmartDashboardHandler.MAX_SPEED.get();
                double speedY = -MathHelper.ScaleInput(LeftY)*SmartDashboardHandler.MAX_SPEED.get();

                // Apply rate limiting
                rot = rotLimiter.calculate(rot);
                speedX = xSpeedLimiter.calculate(speedX);
                speedY = ySpeedLimiter.calculate(speedY);

                // Drive the swerve drive
                swerveDrive.drive(new Translation2d(speedY, speedX), -rot, false, true);
                break;
            }
            case CENTER_WHEELS: {
                // Center all swerve modules
                SwerveDriveTest.centerModules(swerveDrive);
                break;
            }
            case CRAB_WALK: {
                // Get speed and POV from controller
                double speed = MathUtil.applyDeadband(controller.getRightTriggerAxis(), SmartDashboardHandler.TRIGGER_AXIS_DEADZONE.get());
                double pov = controller.getPOV();

                speed = MathHelper.ScaleInput(speed);

                // Calculate rotation input
                double rot = MathUtil.applyDeadband(controller.getRightX(), SmartDashboardHandler.JOYSTICK_DEADZONE.get());
                rot *= SmartDashboardHandler.TURN_SENSITIVITY.get();
                rot = MathHelper.ScaleInput(rot) * SmartDashboardHandler.MAX_ANGULAR_SPEED.get();

                if (pov != -1) {
                    // Calculate crab walk translation based on POV angle
                    double angleRad = Math.toRadians(pov);
                    double crabX = Math.sin(angleRad) * speed * SmartDashboardHandler.MAX_SPEED.get();
                    double crabY = Math.cos(angleRad) * speed * SmartDashboardHandler.MAX_SPEED.get();

                    crabX = xSpeedLimiter.calculate(crabX);
                    crabY = ySpeedLimiter.calculate(crabY);

                    swerveDrive.drive(new Translation2d(crabY, crabX), rot, false, false);
                } else {
                    swerveDrive.drive(new Translation2d(0, 0), rot, false, false);
                }
                break;
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
    }
}
