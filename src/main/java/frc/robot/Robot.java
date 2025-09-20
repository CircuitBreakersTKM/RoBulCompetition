// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.spark.*;

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final double maxSpeed = 0.4;
  private final double maxRot = 0.4;
  private final double deadZone = 0.25;

  private SparkMax[] motors = new SparkMax[8];

  @Override
  public void teleopInit() {
    for (int i = 0; i < 8; i++) {
      motors[i] = new SparkMax(i + 1, SparkLowLevel.MotorType.kBrushless);
    }
  }

  @Override
  public void teleopPeriodic() {
    double LeftX = controller.getLeftX();
    double LeftY = controller.getLeftY();

    if (Math.abs(LeftY) < deadZone) {
      LeftY = 0;
    }
    if (Math.abs(LeftX) < deadZone) {
      LeftX = 0;
    }

    for (int i = 0; i < 4; i++) {
      motors[i].set(-LeftY * maxSpeed); // Forward/backward => Forward is negative
    }
    for (int i = 4; i < 8; i++) {
      motors[i].set(-LeftX * maxRot); // Left/right => Right is positive but wheels reverse the movement
    }
  }

  @Override
  public void teleopExit() {
    for (int i = 0; i < 8; i++) {
      motors[i].set(0);
      motors[i].close();
    }
  }
}
