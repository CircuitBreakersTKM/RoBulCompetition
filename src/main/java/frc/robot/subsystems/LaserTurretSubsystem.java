package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;

/**
 * Subsystem controlling a two-axis laser turret with azimuth and altitude control.
 * Uses two SparkMax motor controllers for independent axis movement.
 * Speed is configurable via NetworkTables dashboard.
 */
public class LaserTurretSubsystem extends SubsystemBase implements MotorizedSubsystem {
    public final SparkMax azimuthMotor;
    public final SparkMax altitudeMotor;

    /**
     * Creates a new LaserTurretSubsystem.
     * 
     * @param azimuthMotorCANID The CAN ID of the azimuth (horizontal rotation) motor controller
     * @param altitudeMotorCANID The CAN ID of the altitude (vertical tilt) motor controller
     */
    public LaserTurretSubsystem(int azimuthMotorCANID, int altitudeMotorCANID) {
        azimuthMotor = new SparkMax(azimuthMotorCANID, SparkMax.MotorType.kBrushless);
        altitudeMotor = new SparkMax(altitudeMotorCANID, SparkMax.MotorType.kBrushless);
    }

    /**
     * Sets the speed of both turret axes.
     * 
     * @param speeds Expects exactly 2 values: [0]=azimuth speed, [1]=altitude speed
     * @throws IllegalArgumentException if not exactly 2 speeds are provided
     */
    @Override
    public void setSpeed(double... speeds) {
        if (speeds.length != 2) {
            throw new IllegalArgumentException("LaserTurretSubsystem requires exactly 2 speed values: azimuth and altitude.");
        }
        
        double azimuthSpeed = speeds[0];
        double altitudeSpeed = speeds[1];

        azimuthSpeed *= NetworkSubsystem.LASER_MOTOR_MAX_SPEED.get();
        altitudeSpeed *= NetworkSubsystem.LASER_MOTOR_MAX_SPEED.get();

        azimuthMotor.set(azimuthSpeed);
        altitudeMotor.set(altitudeSpeed);
    }
    @Override
    public void stop() {
        azimuthMotor.stopMotor();
        altitudeMotor.stopMotor();
    }

    @Override
    public void stopIf(boolean condition) {
        if (condition) {
            stop();
        }
    }
}
