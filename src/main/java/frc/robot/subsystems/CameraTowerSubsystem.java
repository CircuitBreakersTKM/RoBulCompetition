package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.MotorizedSubsystem;
import frc.robot.subsystems.network.NetworkSubsystem;

/**
 * Subsystem controlling a single-axis camera tower with azimuth rotation.
 * Uses a SparkMax motor controller with encoder for position tracking.
 * Speed is configurable via NetworkTables dashboard.
 */
public class CameraTowerSubsystem extends SubsystemBase implements MotorizedSubsystem {
    public final SparkMax azimuthMotor;

    /**
     * Creates a new CameraTowerSubsystem.
     * 
     * @param azimuthMotorCANID The CAN ID of the azimuth motor controller
     */
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

    /**
     * Resets the azimuth encoder position to zero.
     * Call at the start of teleop to establish a known reference position.
     */
    public void zeroEncoder() {
        azimuthMotor.getEncoder().setPosition(0);
    }
    
    @Override
    public void stopIf(boolean condition) {
        if (condition) {
            stop();
        }
    }
}
