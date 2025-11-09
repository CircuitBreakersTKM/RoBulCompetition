package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;
import frc.robot.math.MathHelper;
import frc.robot.math.RateLimiter;
import frc.robot.math.RateLimiter2D;
import frc.robot.subsystems.network.NetworkSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve drive subsystem using YAGSL (Yet Another Generic Swerve Library).
 * Provides field-relative and robot-relative drive control with rate limiting.
 * Automatically adjusts max speed based on battery voltage to prevent brownouts.
 */
public class SwerveDriveSubsystem extends SubsystemBase implements MotorizedSubsystem{
    public final SwerveDrive swerveDrive;

    public boolean skipRateLimiting = false;

    private double maxSpeed = 2.5;
    private double minSpeed = 2;

    public boolean headingCorrection = true;

    private PowerDistribution pdh = new PowerDistribution(10, PowerDistribution.ModuleType.kRev);
    
    private RateLimiter2D speedLimiter = new RateLimiter2D(NetworkSubsystem.MAX_ACCELERATION.get(), 15, 0.5);
    private RateLimiter rotLimiter = new RateLimiter(NetworkSubsystem.MAX_ANGULAR_ACCELERATION.get(), 8*Math.PI, 1);
    
    private Double targetHeading = null; // Stored heading for drift compensation
    private static final double HEADING_KP = 2.0; // Proportional gain for heading correction
    
    /**
     * Creates a new SwerveDriveSubsystem by loading configuration from JSON files.
     * 
     * @throws IOException if swerve configuration files cannot be read
     */
    public SwerveDriveSubsystem() throws IOException {
        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(4.8);
        swerveDrive.useInternalFeedbackSensor();

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    /**
     * Reduces max speed based on battery voltage to prevent brownouts.
     * Linearly interpolates between minSpeed and maxSpeed based on voltage reading.
     * Typically called at the start of teleop.
     */
    public void applyLowBatteryLimiters() {
        double voltage = pdh.getVoltage();
        double k = (voltage - 11.5) / 0.5;

        NetworkSubsystem.MAX_SPEED.set(MathHelper.Interpolate(minSpeed, maxSpeed, k));
    }

    /**
     * Processes Cartesian (X, Y, rotation) drive input with rate limiting and input scaling.
     * 
     * @param translation Translation vector (X=strafe, Y=forward/back) in m/s
     * @param rotation Rotation speed in rad/s
     * @param fieldRelative If true, translation is relative to field; if false, robot-relative
     * @param isOpenLoop If true, uses open-loop control; if false, uses velocity closed-loop
     */
    public void processCarteseanInput(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkSubsystem.MAX_ANGULAR_SPEED.get() * NetworkSubsystem.TURN_SENSITIVITY.get();
        
        // For translation, we calculate polar coordinates to apply a non-linear scaling without affecting direction
        double magnitude = Math.hypot(translation.getY(), translation.getX());
        double angle = Math.atan2(translation.getX(), translation.getY());

        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkSubsystem.MAX_SPEED.get();
        double speedX = magnitude * Math.cos(angle);
        double speedY = magnitude * Math.sin(angle);

        Translation2d speeds = new Translation2d(speedX, speedY);

        // Apply rate limiting
        if (!skipRateLimiting) {
            rot = rotLimiter.calculate(rot);

            speeds = speedLimiter.calculate(speeds);
        }

        // Apply drift compensation
        rot = applyDriftCompensation(rot);

        swerveDrive.drive(speeds, rot, fieldRelative, isOpenLoop);
    }
    
    /**
     * Processes polar (magnitude, angle, rotation) drive input with rate limiting and input scaling.
     * 
     * @param magnitude Drive speed magnitude (0.0 to 1.0)
     * @param angle Drive direction in radians
     * @param rotation Rotation speed (-1.0 to 1.0)
     * @param fieldRelative If true, angle is relative to field; if false, robot-relative
     * @param isOpenLoop If true, uses open-loop control; if false, uses velocity closed-loop
     */
    public void processPolarInput(double magnitude, double angle, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkSubsystem.MAX_ANGULAR_SPEED.get() * NetworkSubsystem.TURN_SENSITIVITY.get();
        
        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkSubsystem.MAX_SPEED.get();

        double speedX = magnitude * Math.cos(angle);
        double speedY = magnitude * Math.sin(angle);

        Translation2d speeds = new Translation2d(speedX, speedY);

        // Apply rate limiting
        if (!skipRateLimiting) {
            rot = rotLimiter.calculate(rot);
            speeds = speedLimiter.calculate(speeds);
        }
        
        // Apply drift compensation
        rot = applyDriftCompensation(rot);
        
        swerveDrive.drive(speeds, rot, fieldRelative, isOpenLoop);
    }
    
    /**
     * Applies drift compensation to rotation input.
     * If rotation is 0, maintains the stored heading by calculating correction.
     * If rotation is non-zero, updates the stored heading to current gyro angle.
     * 
     * @param rotation The rotation input from the driver
     * @return The compensated rotation value
     */
    private double applyDriftCompensation(double rotation) {
        if (!headingCorrection) return rotation;
        
        double currentHeading = swerveDrive.getYaw().getRadians();
        
        if (Math.abs(rotation) < 0.01) { // rotation == 0 (with small tolerance)
            // Driver wants to maintain heading - apply drift compensation
            if (targetHeading == null) {
                // First time with no rotation, store current heading
                targetHeading = currentHeading;
            }
            
            // Calculate heading error and apply proportional correction
            double headingError = targetHeading - currentHeading;
            
            // Normalize error to [-PI, PI]
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            
            // Return correction to compensate for drift
            return headingError * HEADING_KP;
        } else {
            // Driver is actively rotating - update target heading
            targetHeading = currentHeading;
            return rotation;
        }
    }
    
    /**
     * Resets the gyro heading to zero (current direction becomes forward).
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
        targetHeading = null; // Reset drift compensation
    }

    /**
     * Not supported for swerve drive - use processCarteseanInput or processPolarInput instead.
     * 
     * @throws UnsupportedOperationException always
     */
    @Override
    public void setSpeed(double... speeds) {
        throw new UnsupportedOperationException("Use processCarteseanInput or processPolarInput methods to control the swerve drive.");
    }
    @Override
    public void stop() {
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
    }
    @Override
    public void stopIf(boolean required) {
        if (required) {
            stop();
        }
    }
}
