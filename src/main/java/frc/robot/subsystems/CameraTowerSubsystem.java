package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;

public class CameraTowerSubsystem extends SubsystemBase implements MotorizedSubsystem {
    public final SparkMax azimuthMotor;

    public CameraTowerSubsystem(int azimuthMotorCANID) {
        azimuthMotor = new SparkMax(azimuthMotorCANID, SparkMax.MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void stop() {
        azimuthMotor.stopMotor();
    }

    @Override
    public void setSpeed(double... speeds) {
        if (speeds.length != 1) {
            throw new IllegalArgumentException("CameraTowerSubsystem requires exactly one speed value.");
        }
        double azimuthSpeed = speeds[0];
        azimuthSpeed *= NetworkSubsystem.CAMERA_MOTOR_MAX_SPEED.get();

        azimuthMotor.set(azimuthSpeed);
    }

    @Override
    public void stopIf(boolean condition) {
        if (condition) {
            stop();
        }
    }
}
