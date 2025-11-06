package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.CameraTowerSubsystem;

public class CameraTurnCommand extends TrackedCommand {
    private final CameraTowerSubsystem cameraTower;
    private final DoubleSupplier cameraSpeedSupplier;

    /**
     * Creates a new CameraTowerCommand.
     *
     * @param cameraTower The camera tower subsystem this command will control.
     * @param cameraSpeedSupplier A DoubleSupplier that provides the desired camera speed.
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
