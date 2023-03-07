// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.resources.TecbotConstants;
import frc.robot.subsystems.driveTrain;


public class balanceCommand extends CommandBase {
  boolean isLeveled = false;
  /** Creates a new onMotors. */
  public balanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getDriveTrain());
  
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // Robot.getRobotContainer().getDriveTrain().driveForwardWithEncoders();
   double currentEncoderR = Robot.getRobotContainer().getDriveTrain().getRightEncoder();
   double currentEncoderL = Robot.getRobotContainer().getDriveTrain().getLeftEncoder();

   Robot.getRobotContainer().getDriveTrain().balanceL(currentEncoderL, currentEncoderR);
    
   boolean leftBalance = Robot.getRobotContainer().getDriveTrain().getLeftBalance();
   boolean rightBalance = Robot.getRobotContainer().getDriveTrain().getRightBalance();
    
   
    if(leftBalance == true && rightBalance == true){
        isLeveled = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isLeveled;
  }
}