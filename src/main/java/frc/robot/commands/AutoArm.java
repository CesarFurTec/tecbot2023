// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArm extends SequentialCommandGroup {
  /** Creates a new AutoArm. */
  public AutoArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      //new OnArmT()   levantar
      //new OffArm()   bajar
      // new IntakeOn()  abrir 
      //new IntakeOff()  cerrar 
      // new ExtendArm() extender
      // new RetractArm()  retraer
      //new IntakeOff(),
      new OffArm(),
      new WaitCommand(0.6),
      new ExtendArm(),
      new WaitCommand(0.8),
      new IntakeOn(),
      new WaitCommand(0.5),
      new RetractArm(),
      new WaitCommand(0.3),
      new OnArmT(),
      new WaitCommand(0.3),
     // new OutSafe().withTimeout(3.0),
      new ResetEncoderDt(),
      new WaitCommand(0.3),
      //new DriveDistance(2,0.5,1), 
      new DriveDistance(6, -1, 6, -1),
      new WaitCommand(0.3),
      
      //new DriveDistance(5,0.5,-1), 
      new DriveDistance(15, 1, 15, 1), 
      new WaitCommand(0.3),
      new IntakeOff(),
      new WaitCommand(0.3),
      //new DriveDistance(-3,0.5,1), 
      //new OffMotors()
      new DriveDistance(-5, 1, 5, 1),
      new WaitCommand(0.3),
      new DriveDistance(10, 1, 10, 1),
      new WaitCommand(0.3),
      new DriveDistance(3, -1, 3, -1),
      new WaitCommand(0.3),
      /*new OffArm(),
      new WaitCommand(0.3),
      new ExtendArm(),
      new WaitCommand(0.3),*/
      new DriveDistance(-9, 1, 9, 1),
      new WaitCommand(0.3),
      new DriveDistance(12, 1, 12, 1)
    
    );

  }
}
