// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveDistance extends CommandBase {
  double distanceL, distanceR;
  int directionL, directionR;
  boolean finished;
  boolean velChange;
  double speed;

  /** Creates a new DriveDistance. */
  public DriveDistance(double dL,  int dirL, double dR, int dirR, boolean vC) {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
     distanceL = dL;
     distanceR = dR;
     directionL = dirL;
     directionR = dirR;
     velChange = vC;

      //Poner t como signo y que double t sea 1 o -1 para que actualDistance se pueda utilizar tanto al ir para enfrente como para atrás 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.getRobotContainer().getDriveTrain().resetEncoderDt();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    double actualDistanceL =  (Robot.getRobotContainer().getDriveTrain().getDriveTrainFeetL());
    double actualDistanceR =  (Robot.getRobotContainer().getDriveTrain().getDriveTrainFeetR());
    double actualDistanceL2 =  (Robot.getRobotContainer().getDriveTrain().getDriveTrainFeetL2());
    double actualDistanceR2 =  (Robot.getRobotContainer().getDriveTrain().getDriveTrainFeetR2());

    if (velChange = false)
    {
        speed = RobotMap.autonomusSpeed;
    }

    if (velChange = true)
    {
        speed = RobotMap.autonomusCharge;
    }

    boolean a,b,c,d;
      a = Robot.getRobotContainer().getDriveTrain().moveWithEncodersL(actualDistanceL, distanceL, directionL, speed);
      b = Robot.getRobotContainer().getDriveTrain().moveWithEncodersR(actualDistanceR, distanceR, directionR, speed); 
      c = Robot.getRobotContainer().getDriveTrain().moveWithEncodersL2(actualDistanceL2, distanceL, directionL, speed);
      d = Robot.getRobotContainer().getDriveTrain().moveWithEncodersR2(actualDistanceR2, distanceR, directionR, speed);


    if (a && b && c && d){
      finished= true;
  }
    else {
      finished=false;
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
