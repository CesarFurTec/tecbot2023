// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TransmissionChange extends CommandBase {
  boolean TransmisionType;
  public TransmissionChange(boolean transType) {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
    TransmisionType = transType;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    System.out.println("transmissionchange");
    Robot.getRobotContainer().getDriveTrain().switchTransmission(TransmisionType);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
  @Override
  public void end(boolean interrupted) {}


}
