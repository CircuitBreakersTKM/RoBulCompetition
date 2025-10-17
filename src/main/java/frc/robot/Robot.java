// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    public RobotContainer robotContainer;

    @Override 
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
    }
    @Override
    public void teleopInit() {
        robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.processManualInput();
    }

    @Override
    public void teleopExit() {}
}
