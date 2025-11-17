package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.LaserTurretSubsystem;

/**
 * Command that controls the laser turret's azimuth and altitude based on joystick input.
 * Provides continuous control of both axes for manual targeting.
 */
public class LaserMoveCommand extends TrackedCommand {
    private final LaserTurretSubsystem laserTurret;
    private final DoubleSupplier azimuthSupplier;
    private final DoubleSupplier altitudeSupplier;

    /**
     * Creates a new LaserMoveCommand.
     *
     * @param laserTurret The laser turret subsystem this command will control.
     * @param azimuthSupplier A DoubleSupplier that provides the desired azimuth (horizontal) speed.
     * @param altitudeSupplier A DoubleSupplier that provides the desired altitude (vertical) speed.
     */
    public LaserMoveCommand(LaserTurretSubsystem laserTurret, DoubleSupplier azimuthSupplier, DoubleSupplier altitudeSupplier) {
        this.laserTurret = laserTurret;
        this.azimuthSupplier = azimuthSupplier;
        this.altitudeSupplier = altitudeSupplier;
        addRequirements(laserTurret);
    }

    @Override
    public void execute() {
        laserTurret.setSpeed(azimuthSupplier.getAsDouble() * 0.5, altitudeSupplier.getAsDouble() * 0.5);
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
