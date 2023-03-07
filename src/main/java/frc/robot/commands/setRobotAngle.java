// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class setRobotAngle extends CommandBase {
    public boolean isLeveled = false;
  public setRobotAngle() {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Robot.getRobotContainer().getDriveTrain().setAngle();
    double currentAngle = Robot.getRobotContainer().getDriveTrain().getAngle();
    if(currentAngle > -10 && currentAngle < 10)
    {
        isLeveled = true;
    }else
    {
        isLeveled = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return isLeveled;
  }
}
