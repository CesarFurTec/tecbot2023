// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.TecbotConstants;



public class DriveRobotBackwards extends CommandBase {
  /** Creates a new DriveRobotBackwards. */
  boolean finished = false;
  public DriveRobotBackwards() {
    
    addRequirements(Robot.getRobotContainer().getDriveTrain());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getRobotContainer().getDriveTrain().driveBackwards();
    double distance = TecbotConstants.setpointBakcwards - Robot.getRobotContainer().getDriveTrain().getDriveTrainFeetL();
    
    if(distance*TecbotConstants.kP <= 0.03){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;

  }
}
