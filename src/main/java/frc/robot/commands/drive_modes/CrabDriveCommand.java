package frc.robot.commands.drive_modes;

import java.util.function.DoubleSupplier;

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
            swerveDriveSubsystem.processPolarInput(0, 0, rotation, false, false);
            return; // Ignore when using POV and not pressed
        }

        angle = angle % 360;

        if (allowOnlyCardinalDirections) {
            if (angle % 90 == 0) // Ignore when using POV and pressed between directions
            {
                swerveDriveSubsystem.processPolarInput(0, 0, rotation, false, false);
                return;
            }

            angle = Math.round(rotation / 90.0) * 90.0;
        }

        angle = Math.toRadians(angle);

        // Drive the swerve drive
        swerveDriveSubsystem.processPolarInput(speed, angle, rotation, false, false);
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
