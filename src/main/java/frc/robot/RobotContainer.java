package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.MathHelper;
import frc.robot.math.SpeedRateLimiter;
import frc.robot.network.NetworkHandler;
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
    
    private SpeedRateLimiter xSpeedLimiter;
    private SpeedRateLimiter ySpeedLimiter;
    private SpeedRateLimiter rotLimiter;
    
    public RobotContainer() {
        NetworkHandler.Init();

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
        double k = (voltage - 11.5) / 0.5;

        NetworkHandler.MAX_SPEED.set(MathHelper.Interpolate(minSpeed, maxSpeed, k));

        xSpeedLimiter = new SpeedRateLimiter(NetworkHandler.MAX_ACCELERATION.get(), 15, 0, 0.5);
        ySpeedLimiter = new SpeedRateLimiter(NetworkHandler.MAX_ACCELERATION.get(), 15, 0, 0.5);
        rotLimiter = new SpeedRateLimiter(NetworkHandler.MAX_ANGULAR_ACCELERATION.get(), 8*Math.PI, 0, 1);
    }
    public void processManualInput() {

        switch (NetworkHandler.autoModeChooser.getSelected()) {
            case DRIVE: {
                // Get joystick inputs
                double RightX = controller.getRightX() * NetworkHandler.TURN_SENSITIVITY.get();
                double LeftX = controller.getLeftX();
                double LeftY = controller.getLeftY();

                // Apply deadzone
                double deadzone = NetworkHandler.JOYSTICK_DEADZONE.get();
                RightX = MathUtil.applyDeadband(RightX, deadzone);
                LeftX = MathUtil.applyDeadband(LeftX, deadzone);
                LeftY = MathUtil.applyDeadband(LeftY, deadzone);

                // Scale inputs and apply max speeds
                double rot = MathHelper.ScaleRotInput(RightX) * NetworkHandler.MAX_ANGULAR_SPEED.get();
                double speedX = -MathHelper.ScaleSpeedInput(LeftX)*NetworkHandler.MAX_SPEED.get();
                double speedY = -MathHelper.ScaleSpeedInput(LeftY)*NetworkHandler.MAX_SPEED.get();

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
                double speed = MathUtil.applyDeadband(controller.getRightTriggerAxis(), NetworkHandler.TRIGGER_AXIS_DEADZONE.get());
                double pov = controller.getPOV();

                speed = MathHelper.ScaleSpeedInput(speed);

                // Calculate rotation input
                double rot = MathUtil.applyDeadband(controller.getRightX(), NetworkHandler.JOYSTICK_DEADZONE.get());
                rot *= NetworkHandler.TURN_SENSITIVITY.get();
                rot = -MathHelper.ScaleRotInput(rot) * NetworkHandler.MAX_ANGULAR_SPEED.get();

                if (pov != -1) {
                    // Calculate crab walk translation based on POV angle
                    double angleRad = Math.toRadians(pov);
                    double crabX = -Math.sin(angleRad) * speed * NetworkHandler.MAX_SPEED.get();
                    double crabY = Math.cos(angleRad) * speed * NetworkHandler.MAX_SPEED.get();

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
