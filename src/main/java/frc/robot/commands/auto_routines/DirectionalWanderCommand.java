package frc.robot.commands.auto_routines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DirectionalWanderCommand extends TrackedCommand {
    private final SwerveDriveSubsystem drive;        // your YAGSL drivetrain
    private double direction;              // +1 or -1
    private final double speed;
    private double startX;                 // Initial X position when command starts

    // FIELD LIMITS (modify for your field coords)
    private final double minX = 0.5;
    private final double maxX = 7.5;

    public DirectionalWanderCommand(SwerveDriveSubsystem drive,
                                    double speed) {
        this.drive = drive;
        this.direction = Math.signum(Math.random() - 0.5); // start random direction
        this.speed = speed;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Assume bot starts in middle of field
        startX = (minX + maxX) / 2.0;
        drive.swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void execute() {
        // Get current position from gyro odometry
        // Displacement in X direction = initial X + integrated X movement from odometry
        double currentX = startX + drive.swerveDrive.getPose().getX();

        // Distance to nearest wall
        double distToLeft = currentX - minX;
        double distToRight = maxX - currentX;

        // Force direction AWAY from edges
        if (distToLeft < 0.3) { 
            direction = +1;
        } else if (distToRight < 0.3) { 
            direction = -1;
        } else {

            double norm = currentX / 3.5; // 0 at edges, 1 at midpoint

            double flip = Math.exp(-6.9078 * norm); // 1 at edges, 0.001 at center

            if (Math.random() < flip) {
                direction *= -1;
            }
        }

        // Drive in X direction only
        drive.processCarteseanInput(
            new Translation2d(0, direction * speed),
            0,
            false, false
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.processCarteseanInput(new Translation2d(), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

