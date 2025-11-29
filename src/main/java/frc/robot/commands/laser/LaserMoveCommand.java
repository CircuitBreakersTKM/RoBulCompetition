package frc.robot.commands.laser;

import java.util.function.DoubleSupplier;

import frc.robot.commands.TrackedCommand;
import frc.robot.math.MathHelper;
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
        double aziSpeed = azimuthSupplier.getAsDouble();
        double altSpeed = altitudeSupplier.getAsDouble();

        aziSpeed = MathHelper.ScaleLaserInput(aziSpeed) * 0.1;
        altSpeed = MathHelper.ScaleLaserInput(altSpeed) * 0.1;

        laserTurret.setSpeed(aziSpeed, altSpeed);
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
