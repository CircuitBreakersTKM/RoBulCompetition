package frc.robot.commands.drive_modes;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.TrackedCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class JoystickDriveCommand extends TrackedCommand {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final DoubleSupplier speedXSupplier;
    private final DoubleSupplier speedYSupplier;
    private final DoubleSupplier rotSupplier;

    /**
     * Creates a new JoystickDriveCommand.
     *
     * @param subsystem The swerve drive subsystem this command will control.
     * @param speedXSupplier A DoubleSupplier that provides the desired speed in the X direction.
     * @param speedYSupplier A DoubleSupplier that provides the desired speed in the Y direction.
     * @param rotSupplier A DoubleSupplier that provides the desired rotational speed.
     */
    public JoystickDriveCommand(SwerveDriveSubsystem subsystem, DoubleSupplier speedXSupplier, 
                                DoubleSupplier speedYSupplier, DoubleSupplier rotSupplier) {
        this.swerveDriveSubsystem = subsystem;
        this.speedXSupplier = speedXSupplier;
        this.speedYSupplier = speedYSupplier;
        this.rotSupplier = rotSupplier;

        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        double speedX = speedXSupplier.getAsDouble();
        double speedY = speedYSupplier.getAsDouble();
        double rot = rotSupplier.getAsDouble();

        // Drive the swerve drive
        swerveDriveSubsystem.processCarteseanInput(new Translation2d(-speedX, -speedY), rot, true, true);
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
