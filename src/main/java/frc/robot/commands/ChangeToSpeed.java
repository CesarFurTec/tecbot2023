// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChangeToSpeed extends CommandBase {
  boolean isSafetyOn = false;
  /** Creates a new ChangeToSpeed. */
  public ChangeToSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getDriveTrain());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getRobotContainer().getDriveTrain().changeToSpeed();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    isSafetyOn = Robot.getRobotContainer().getSafety();
    if(isSafetyOn)
    {

    }else{
      Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX(true), Robot.getRobotContainer().getOI().getPilot().getLeftAxisY(true));
    Robot.driveRetake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
