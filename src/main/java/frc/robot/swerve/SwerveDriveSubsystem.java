package frc.robot.swerve;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.math.MathHelper;
import frc.robot.math.RateLimiter;
import frc.robot.math.RateLimiter2D;
import frc.robot.network.NetworkHandler;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem {
    public final SwerveDrive swerveDrive;

    private double maxSpeed = 2.5;
    private double minSpeed = 2;

    private PowerDistribution pdh = new PowerDistribution(10, PowerDistribution.ModuleType.kRev);
    
    private RateLimiter2D speedLimiter;
    private RateLimiter rotLimiter;

    private final SwerveModule[] modules;
    
    public SwerveDriveSubsystem() throws IOException {
        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(4.8);
        swerveDrive.useInternalFeedbackSensor();
        // swerveDrive.setHeadingCorrection(false);
        // swerveDrive.setCosineCompensator(false);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        modules = swerveDrive.getModules();
    }

    public void applyLowBatteryLimiters() {
        double voltage = pdh.getVoltage();
        double k = (voltage - 11.5) / 0.5;

        NetworkHandler.MAX_SPEED.set(MathHelper.Interpolate(minSpeed, maxSpeed, k));

        speedLimiter = new RateLimiter2D(NetworkHandler.MAX_ACCELERATION.get(), 15, 0.5);
        rotLimiter = new RateLimiter(NetworkHandler.MAX_ANGULAR_ACCELERATION.get(), 8*Math.PI, 1);
    }

    public void processCarteseanInput(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkHandler.MAX_ANGULAR_SPEED.get() * NetworkHandler.TURN_SENSITIVITY.get();
        
        // For translation, we calculate polar coordinates to apply a non-linear scaling without affecting direction
        double magnitude = Math.hypot(translation.getY(), translation.getX());
        double angle = Math.atan2(translation.getX(), translation.getY());

        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkHandler.MAX_SPEED.get();
        double speedX = magnitude * Math.cos(angle);
        double speedY = magnitude * Math.sin(angle);

        // Apply rate limiting
        rot = rotLimiter.calculate(rot);

        Translation2d limitedSpeeds = speedLimiter.calculate(new Translation2d(speedX, speedY));

        swerveDrive.drive(limitedSpeeds, rot, fieldRelative, isOpenLoop);
    }
    public void processPolarInput(double magnitude, double angle, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkHandler.MAX_ANGULAR_SPEED.get() * NetworkHandler.TURN_SENSITIVITY.get();
        
        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkHandler.MAX_SPEED.get();

        double speedX = magnitude * Math.cos(angle);
        double speedY = magnitude * Math.sin(angle);

        // Apply rate limiting
        rot = rotLimiter.calculate(rot);
        Translation2d limitedSpeeds = speedLimiter.calculate(new Translation2d(speedX, speedY));
        swerveDrive.drive(limitedSpeeds, rot, fieldRelative, isOpenLoop);
    }
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void stopIf(boolean required) {
        if (!required) return;
        
        for (SwerveModule module : modules) {
            module.getDriveMotor().set(0);
            module.getAngleMotor().set(0);
        }
    }
}
