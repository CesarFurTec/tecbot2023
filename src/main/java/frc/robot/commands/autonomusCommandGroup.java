// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.driveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autonomusCommandGroup extends SequentialCommandGroup {
  /** Creates a new Autonomus. */
  public autonomusCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(Robot.getRobotContainer().getDriveTrain());
    int x = 1;
    while(x<10){
      addCommands(new onMotors(), 
      new WaitCommand(3.2), 
      new OffMotors(), 
      new WaitCommand(1), 
      //new turnLeftDriveTrain(), 
      new WaitCommand(1.0), 
      new onMotors(), 
      new WaitCommand(3.6), 
      new OffMotors(), 
      new WaitCommand(1), 
      //new turnLeftDriveTrain(),
       new WaitCommand(1.0));
      x++; 
    }
    //addCommands(new onMotors(),
//new WaitCommand(4), 
  //   new OffMotors(), 
    // new WaitCommand(1),
     //new turnLeftDriveTrain(), 
     //new WaitCommand(1.0),
      //new onMotors(), 
      //new WaitCommand(3.6),
      //new OffMotors(),
       //new WaitCommand(1),
      // new turnLeftDriveTrain(), 
      // new WaitCommand(1.0),
        //new OffMotors(), 
        //new WaitCommand(1),
         //new onMotors(), 
         //new WaitCommand(3.2), 
        // new OffMotors(),
         //new WaitCommand(1),
         //new turnLeftDriveTrain(),
         //new WaitCommand(1.0),
        /*  new onMotors(), 
         new WaitCommand(3.6),
          new OffMotors(), 
          new WaitCommand(1), 
          new turnLeftDriveTrain(),
           new WaitCommand(0.75),
            new OffMotors());*/ 
            //First Automonous
    //addCommands(new IntakeOn(), new WaitCommand(2), new onMotors(),new WaitCommand(1), new OffMotors(),new WaitCommand(2),  new IntakeOff());
  }
}