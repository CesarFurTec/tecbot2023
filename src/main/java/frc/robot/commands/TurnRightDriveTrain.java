// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotConstants;

public class TurnRightDriveTrain extends CommandBase {
  /** Creates a new TurnRight. */
  boolean finished = false;
  double distanceL, distanceR;
  double speed;
  
  public TurnRightDriveTrain(double dis) {
    addRequirements(Robot.getRobotContainer().getDriveTrain());
     distanceL = dis;
     distanceR = -dis;
     //new DriveDistance(-3.6, 1, 3.6, 1),    IZQ
     //new DriveDistance(3.7, 1, -3.7, 1),  DER
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

    speed = RobotMap.autonomusSpeed;

    boolean a,b,c,d;
      a = Robot.getRobotContainer().getDriveTrain().moveWithEncodersL(actualDistanceL, distanceL, 1, speed);
      b = Robot.getRobotContainer().getDriveTrain().moveWithEncodersR(actualDistanceR, distanceR, 1, speed); 
      c = Robot.getRobotContainer().getDriveTrain().moveWithEncodersL2(actualDistanceL2, distanceL, 1, speed);
      d = Robot.getRobotContainer().getDriveTrain().moveWithEncodersR2(actualDistanceR2, distanceR, 1, speed);


   

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
