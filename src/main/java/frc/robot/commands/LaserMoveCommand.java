package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.LaserTurretSubsystem;

public class LaserMoveCommand extends TrackedCommand {
    private final LaserTurretSubsystem laserTurret;
    private final DoubleSupplier azimuthSupplier;
    private final DoubleSupplier altitudeSupplier;

    /**
     * Creates a new LaserMoveCommand.
     *
     * @param cameraTower The laser turret subsystem this command will control.
     * @param azimuthSupplier A DoubleSupplier that provides the desired azimuth speed.
     * @param altitudeSupplier A DoubleSupplier that provides the desired altitude speed.
     */
    public LaserMoveCommand(LaserTurretSubsystem cameraTower, DoubleSupplier azimuthSupplier, DoubleSupplier altitudeSupplier) {
        this.laserTurret = cameraTower;
        this.azimuthSupplier = azimuthSupplier;
        this.altitudeSupplier = altitudeSupplier;
        addRequirements(cameraTower);
    }

    @Override
    public void execute() {
        laserTurret.setSpeed(azimuthSupplier.getAsDouble(), altitudeSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        laserTurret.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until cancelled
    }
}
