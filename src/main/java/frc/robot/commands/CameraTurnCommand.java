package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.CameraTowerSubsystem;

/**
 * Command that continuously rotates the camera tower based on controller input.
 * Extends TrackedCommand to allow bulk cancellation when switching modes.
 */
public class CameraTurnCommand extends TrackedCommand {
    private final CameraTowerSubsystem cameraTower;
    private final DoubleSupplier cameraSpeedSupplier;

    /**
     * Creates a new CameraTurnCommand.
     *
     * @param cameraTower The camera tower subsystem this command will control.
     * @param cameraSpeedSupplier A DoubleSupplier that provides the desired camera rotation speed.
     *                           Typically from bumper buttons: left=1.0, right=-1.0.
     */
    public CameraTurnCommand(CameraTowerSubsystem cameraTower, DoubleSupplier cameraSpeedSupplier) {
        this.cameraTower = cameraTower;
        this.cameraSpeedSupplier = cameraSpeedSupplier;
        addRequirements(cameraTower);
    }

    @Override
    public void execute() {
        cameraTower.setSpeed(cameraSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        cameraTower.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
