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
      // new IntakeOn()  abrir cerrar
      //new IntakeOff()  cerrar abrir
      // new ExtendArm() extender
      // new RetractArm()  retraer
    
      new OffArm(),
      new WaitCommand(2.0),
      new ExtendArm(),
      new WaitCommand(2.5),
      new IntakeOn(),
      new WaitCommand(1.5),
      new RetractArm(),
      new WaitCommand(1.5),
      new OnArmT(),
      new WaitCommand(1.5),
      new OutSafe().withTimeout(3.0),
      new WaitCommand(1.5)
      //new DriveDistance(3,0.5,1), 
      //new WaitCommand(1.5),
      //new DriveDistance(-3,0.5,1), 
      //new OffMotors()

    );

  }
}
