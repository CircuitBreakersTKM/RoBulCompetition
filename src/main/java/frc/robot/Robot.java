// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Main robot class that extends TimedRobot for periodic execution.
 * Manages the lifecycle of the robot and coordinates with RobotContainer.
 */
public class Robot extends TimedRobot {
    public RobotContainer robotContainer;

    /**
     * Called once when the robot is first powered on.
     * Initializes the RobotContainer singleton.
     */
    @Override 
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
    }
    
    /**
     * Called when teleop mode begins.
     * Applies low battery limiters and resets camera encoder.
     */
    @Override
    public void teleopInit() {
        robotContainer.init(false);
    }

    /**
     * Called periodically during teleop mode.
     * Runs the command scheduler and processes manual controller input.
     */
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.teleopPeriodic();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void autonomousInit() {
        robotContainer.init(true);
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.autonomousPeriodic();
    }
}
