// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class driveRobot extends CommandBase {
  boolean isSafetyOn = false;
   
  /** Creates a new driveRobot. */
  public driveRobot() {
   addRequirements(Robot.getRobotContainer().getDriveTrain()); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isSafetyOn = Robot.getRobotContainer().getSafety();
    if(isSafetyOn)
    {
      Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX(true) * RobotMap.childSafetySpeed, Robot.getRobotContainer().getOI().getPilot().getLeftAxisY(true)* RobotMap.childSafetySpeed);
    }else
    {
      Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX(true), Robot.getRobotContainer().getOI().getPilot().getLeftAxisY(true));
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
