package frc.robot.camera;

import com.revrobotics.spark.SparkMax;

import frc.robot.network.NetworkHandler;

public class CameraTower {
    public final SparkMax azimuthMotor;

    public CameraTower(int azimuthMotorCANID) {
        azimuthMotor = new SparkMax(azimuthMotorCANID, SparkMax.MotorType.kBrushless);
    }

    public void setSpeeds(double azimuthSpeed) {
        azimuthSpeed *= NetworkHandler.CAMERA_MOTOR_MAX_SPEED.get();

        azimuthMotor.set(azimuthSpeed);
    }
    public void stop() {
        azimuthMotor.stopMotor();
    }
}
