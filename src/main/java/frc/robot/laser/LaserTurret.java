package frc.robot.laser;

import com.revrobotics.spark.SparkMax;

import frc.robot.network.NetworkHandler;

public class LaserTurret {
    public final SparkMax azimuthMotor;
    public final SparkMax altitudeMotor;

    public LaserTurret(int azimuthMotorCANID, int altitudeMotorCANID) {
        azimuthMotor = new SparkMax(azimuthMotorCANID, SparkMax.MotorType.kBrushless);
        altitudeMotor = new SparkMax(altitudeMotorCANID, SparkMax.MotorType.kBrushless);
    }

    public void setSpeeds(double azimuthSpeed, double altitudeSpeed) {
        azimuthSpeed *= NetworkHandler.LASER_MOTOR_MAX_SPEED.get();
        altitudeSpeed *= NetworkHandler.LASER_MOTOR_MAX_SPEED.get();

        azimuthMotor.set(azimuthSpeed);
        altitudeMotor.set(altitudeSpeed);
    }
    public void stop() {
        azimuthMotor.stopMotor();
        altitudeMotor.stopMotor();
    }
}
