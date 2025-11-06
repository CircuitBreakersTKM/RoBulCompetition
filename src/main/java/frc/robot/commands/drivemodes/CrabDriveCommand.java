package frc.robot.commands.drivemodes;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CrabDriveCommand extends TrackedCommand {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier angleSupplier;
    private final DoubleSupplier rotSupplier;
    private boolean allowOnlyCardinalDirections;

    public CrabDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem,
                                DoubleSupplier speedSupplier,
                                DoubleSupplier angleSupplier,
                                DoubleSupplier rotSupplier,
                                boolean allowOnlyCardinalDirections) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        
        this.speedSupplier = speedSupplier;
        this.angleSupplier = angleSupplier;
        this.rotSupplier = rotSupplier;
        this.allowOnlyCardinalDirections = allowOnlyCardinalDirections;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        double angle = angleSupplier.getAsDouble();
        double rotation = rotSupplier.getAsDouble();

        if (Math.abs(angle) == 1) {
            swerveDriveSubsystem.stop();
            return; // Ignore when using POV and not pressed
        }

        if (angle > 4*Math.PI) {
            angle = Rotation2d.fromDegrees(angle).getRadians(); // Convert from degrees to radians if needed
        }

        if (allowOnlyCardinalDirections) {
            if (angle % 90 == 0) // Ignore when using POV and pressed between directions
            {
                swerveDriveSubsystem.stop();
                return;
            }

            angle = Math.round(rotation / (Math.PI / 2)) * (Math.PI / 2);
        }

        // Drive the swerve drive
        swerveDriveSubsystem.processPolarInput(speed, angle, rotation, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
    
}
