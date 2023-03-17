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

      //////////////////////////////////////////////// AUTONOMO CENTRO  ////////////////////////////////////////////////////////////

      new OffArm(),    //sube brazo
    new WaitCommand(0.8),
    new ExtendArm(),  //extiende brazo
    new WaitCommand(0.8),
    new IntakeOn(),  //abre y suelta
    new WaitCommand(0.5),
    new RetractArm(),  //retrae brazo
    new WaitCommand(0.5),
    new OnArmT(),  //baja brazo
    new WaitCommand(0.3),
    new DriveDistance(14.9, -1, 14.9, -1, true)   //retrocede 9.5 pies
     
      // estaba en 15.8, solution 15.45

    );

  }
}


// meter multiplicador a variable de velocidad 