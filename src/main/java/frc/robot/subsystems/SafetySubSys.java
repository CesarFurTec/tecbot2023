// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SafetySubSys extends SubsystemBase {
  /** Creates a new SafetySubSys. */
  public SafetySubSys() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void safetyOn()
  {
    Robot.getRobotContainer().setSafety(true);
  }

  public void safetyOff()
  {
    Robot.getRobotContainer().setSafety(false);
  }
}
