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

public class SwerveDriveSubsystem extends SubsystemBase implements MotorizedSubsystem{
    public final SwerveDrive swerveDrive;

    private double maxSpeed = 2.5;
    private double minSpeed = 2;

    private PowerDistribution pdh = new PowerDistribution(10, PowerDistribution.ModuleType.kRev);
    
    private RateLimiter2D speedLimiter = new RateLimiter2D(NetworkSubsystem.MAX_ACCELERATION.get(), 15, 0.5);
    private RateLimiter rotLimiter = new RateLimiter(NetworkSubsystem.MAX_ANGULAR_ACCELERATION.get(), 8*Math.PI, 1);
    
    public SwerveDriveSubsystem() throws IOException {
        // Specify the directory containing your JSON configuration files
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(4.8);
        swerveDrive.useInternalFeedbackSensor();

        // swerveDrive.setHeadingCorrection(false);
        // swerveDrive.setCosineCompensator(false);

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    public void applyLowBatteryLimiters() {
        double voltage = pdh.getVoltage();
        double k = (voltage - 11.5) / 0.5;

        NetworkSubsystem.MAX_SPEED.set(MathHelper.Interpolate(minSpeed, maxSpeed, k));
    }

    public void processCarteseanInput(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkSubsystem.MAX_ANGULAR_SPEED.get() * NetworkSubsystem.TURN_SENSITIVITY.get();
        
        // For translation, we calculate polar coordinates to apply a non-linear scaling without affecting direction
        double magnitude = Math.hypot(translation.getY(), translation.getX());
        double angle = Math.atan2(translation.getX(), translation.getY());

        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkSubsystem.MAX_SPEED.get();
        double speedX = magnitude * Math.cos(angle);
        double speedY = magnitude * Math.sin(angle);

        // Apply rate limiting
        rot = rotLimiter.calculate(rot);

        Translation2d limitedSpeeds = speedLimiter.calculate(new Translation2d(speedX, speedY));

        swerveDrive.drive(limitedSpeeds, rot, fieldRelative, isOpenLoop);
    }
    public void processPolarInput(double magnitude, double angle, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Scale inputs and apply max speeds
        double rot = MathHelper.ScaleRotInput(rotation) * NetworkSubsystem.MAX_ANGULAR_SPEED.get() * NetworkSubsystem.TURN_SENSITIVITY.get();
        
        magnitude = MathHelper.ScaleSpeedInput(magnitude) * NetworkSubsystem.MAX_SPEED.get();

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
