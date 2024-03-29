// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.time.format.TextStyle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class driveRobot extends CommandBase {
  boolean isSafetyOn = false;
  double isMarioControllerActiveOffsetNecessary = 1.0;
  int testX,testY;
   
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
    testX = 0;
    testY = 0;
    isSafetyOn = Robot.getRobotContainer().getSafety();
    SmartDashboard.putNumber("x pilot: ", Robot.getRobotContainer().getOI().getPilot().getLeftAxisX() );
        SmartDashboard.putNumber("Y pilot: ", Robot.getRobotContainer().getOI().getPilot().getLeftAxisY() );
        SmartDashboard.putNumber("x copilot: ", Robot.getRobotContainer().getOI().getCopilot().getLeftAxisX() );
        SmartDashboard.putNumber("Y copilot: ", Robot.getRobotContainer().getOI().getCopilot().getLeftAxisY() );

        if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX() < 0 )
        {
         // isMarioControllerActiveOffsetNecessary = 3.0;
         testX = -1;
        }else if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX() > 0 )
        {
          testX = 1;
        }

        if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisY() < 0 )
        {
         // isMarioControllerActiveOffsetNecessary = 3.0;
         testY = -1;
        }else if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisY() > 0 )
        {
          testY = 1;
        }
        

    if(isSafetyOn)
    {
      if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX() != 0 || Robot.getRobotContainer().getOI().getPilot().getLeftAxisY() != 0)
      {
        Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX() * RobotMap.childSafetySpeed * isMarioControllerActiveOffsetNecessary, Robot.getRobotContainer().getOI().getPilot().getLeftAxisY()* RobotMap.childSafetySpeed);
        //Robot.getRobotContainer().getDriveTrain().drive(testX* RobotMap.childSafetySpeed , testY* RobotMap.childSafetySpeed);
      }else {
        Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getCopilot().getLeftAxisX() * RobotMap.childSafetySpeed* isMarioControllerActiveOffsetNecessary, Robot.getRobotContainer().getOI().getCopilot().getLeftAxisY()* RobotMap.childSafetySpeed);
        //Robot.getRobotContainer().getDriveTrain().drive(testX* RobotMap.childSafetySpeed , testY* RobotMap.childSafetySpeed);
      }
      
      //
    }else
    {
      if(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX(true) != 0 || Robot.getRobotContainer().getOI().getPilot().getLeftAxisY(true) != 0)
      {
        //Robot.getRobotContainer().getDriveTrain().drive(testX,testY);
        Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getPilot().getLeftAxisX(true) , Robot.getRobotContainer().getOI().getPilot().getLeftAxisY(true));
      }else {
       // Robot.getRobotContainer().getDriveTrain().drive(testX,testY);
        Robot.getRobotContainer().getDriveTrain().drive(Robot.getRobotContainer().getOI().getCopilot().getLeftAxisX(true) , Robot.getRobotContainer().getOI().getCopilot().getLeftAxisY(true));
      }
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
