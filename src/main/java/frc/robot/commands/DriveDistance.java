// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveDistance extends CommandBase {
  double distance, threshold;
  int direction;
  boolean finished;
  /** Creates a new DriveDistance. */
  public DriveDistance(double d, double t, int dir) {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
     distance = d;
     threshold = t;
     direction = dir;

      //Poner t como signo y que double t sea 1 o -1 para que actualDistance se pueda utilizar tanto al ir para enfrente como para atrás 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Robot.getRobotContainer().getDriveTrain().resetEncoderDt();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualDistance = distance - Robot.getRobotContainer().getDriveTrain().getDriveTrainFeet();

    Robot.getRobotContainer().getDriveTrain().driveWithEncoders(distance, direction);
    System.out.println("drivedistance : " + actualDistance + "  " + threshold);

      if(Math.abs( actualDistance ) < (threshold) )
       {
        System.out.println("finished >");
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
