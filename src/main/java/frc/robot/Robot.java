// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;*/

import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.spark.*;
import static frc.robot.Shared.*;

public class Robot extends TimedRobot {
  /*private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    //m_swerve.updateOdometry();
  }*/
  private SparkMax[] motors = new SparkMax[8];

  @Override
  public void teleopInit() {
    for (int i = 1; i <= 8; i++) {
      motors[i - 1] = new SparkMax(i, SparkLowLevel.MotorType.kBrushless);
    }

    SparkMax motor;
    for (int i = 0; i < 8; i++) {
      motor = motors[i];

      motor.set(0.2);
      System.out.println("Motor " + i + " Temperature: " + motor.getMotorTemperature() + " C");

      Sleep(1000);  
      motor.set(-0.2);
      
      Sleep(1000);
      motor.set(0);

      Sleep(1000);
    }
  }

  @Override
  public void teleopExit() {
    for (int i = 0; i < 8; i++) {
      motors[i].set(0);
      motors[i].close();
    }
  }

  @Override
  public void teleopPeriodic() {
    //driveWithJoystick(true);
  }

  /*private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }*/
}
