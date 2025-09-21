package frc.robot;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SwerveModule {
  public SparkMax driveMotor;
  public SparkMax turnMotor;

  private double maxDriveSpeed = 0.8;
  private double maxTurnSpeed = 0.5;
  private double minMotorSpeed = 0.05;
  private double desiredAngle = 0;

  /*
   * Returns the current angle of the turn motor in degrees (-180 to 180)
   */
  private double currentAngle() {
    double theta = turnMotor.getAbsoluteEncoder().getPosition(); // Encoder ticks = 4096 ticks / 360 degrees

    theta = theta * (360.0 / 4096.0); // Convert ticks to degrees
    
    return ((theta + 180) % 360 + 360) % 360 - 180; //clamp to always be within -180 to 180
  }

  /*
   * Constructor for a SwerveModule
   * @param driveID The CAN ID of the drive motor
   * @param turnID The CAN ID of the turn motor
   */
  public SwerveModule(int driveID, int turnID) {
    driveMotor = new SparkMax(driveID, SparkLowLevel.MotorType.kBrushless);
    turnMotor = new SparkMax(turnID, SparkLowLevel.MotorType.kBrushless);
  }

  /*
   * Sets the speed of the drive motor, scaled by maxSpeed
   * @param speed The speed to set the drive motor to, from -1 to 1
   */
  public void setSpeed(double speed) {
    speed = Math.max(-1, Math.min(1, speed)); // Clamp speed to -1 to 1
    driveMotor.set(speed * maxDriveSpeed);
  }

  /*
   * Sets the angle of the turn motor
   * @param angle The angle to set the turn motor to, in degrees (-180 to 180)
   */
  public void setAngle(double angle) {
    desiredAngle = ((angle + 180) % 360 + 360) % 360 - 180; //clamp to always be within -180 to 180
  }

  /*
   * Stops both motors on the spot
   */
  public void Stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  /*
   * Updates the turn motor to move towards the desired angle
   * Should be called in a loop periodically
   */
  public void Tick() {
    double currentAngle = currentAngle();
    double angleDifference = desiredAngle - currentAngle;
    
    // If we need to turn more than 180 degrees, take the shorter path
    if (angleDifference > 180) {
      angleDifference -= 360;
    } else if (angleDifference < -180) {
      angleDifference += 360;
    }

    // Simple proportional control for turning
    double kP = 0.01;

    double turnSpeed = kP * angleDifference;
    if (Math.abs(turnSpeed) < minMotorSpeed && Math.abs(angleDifference) > 1) {
      turnSpeed = minMotorSpeed * Math.signum(turnSpeed);
    }

    // Clamp turn speed to a reasonable range
    turnSpeed = Math.max(-maxTurnSpeed, Math.min(maxTurnSpeed, turnSpeed));

    turnMotor.set(turnSpeed);
  }
}
