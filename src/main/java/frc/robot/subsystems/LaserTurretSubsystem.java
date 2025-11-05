package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;

public class LaserTurretSubsystem extends SubsystemBase implements MotorizedSubsystem {
    public final SparkMax azimuthMotor;
    public final SparkMax altitudeMotor;

    public LaserTurretSubsystem(int azimuthMotorCANID, int altitudeMotorCANID) {
        azimuthMotor = new SparkMax(azimuthMotorCANID, SparkMax.MotorType.kBrushless);
        altitudeMotor = new SparkMax(altitudeMotorCANID, SparkMax.MotorType.kBrushless);
    }

    @Override
    public void setSpeed(double... speeds) {
        if (speeds.length != 2) {
            throw new IllegalArgumentException("LaserTurretSubsystem requires exactly 2 speed values: azimuth and altitude.");
        }
        
        double azimuthSpeed = speeds[0];
        double altitudeSpeed = speeds[1];

        azimuthSpeed *= NetworkHandlerSubsystem.LASER_MOTOR_MAX_SPEED.get();
        altitudeSpeed *= NetworkHandlerSubsystem.LASER_MOTOR_MAX_SPEED.get();

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
