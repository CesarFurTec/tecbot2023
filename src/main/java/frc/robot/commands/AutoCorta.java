// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCorta extends SequentialCommandGroup {
  /** Creates a new AutoCorta. */
  public AutoCorta() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //new OnArmT()   levantar
      //new OffArm()   bajar
      // new IntakeOn()  abrir 
      //new IntakeOff()  cerrar 
      // new ExtendArm() extender
      // new RetractArm()  retraer
    addCommands(

    //////////////////////////////////////////////// AUTONOMO CORTA  ////////////////////////////////////////////////////////////

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
    new DriveDistance(18, -1, 18, -1),   //retrocede 18 pies
    new WaitCommand(0.3),

    //new DriveDistance(-3.6, 1, 3.6, 1),
    new turnLeftDriveTrain(3.6),   //vuelta a la izquierda de 45 grados
    new WaitCommand(0.3),
    new DriveDistance(2, -1, 2, -1),   //retrocede 2 pies para prepararse para mas  vuelta
    new WaitCommand(0.3),
    //new DriveDistance(-3.6, 1, 3.6, 1),
    new turnLeftDriveTrain(3.6),  //vuelta a la izquierda de 45 grados
    new WaitCommand(0.3),

    new DriveDistance(4, 1, 4, 1),    //avanza a altura de charge station
    new WaitCommand(0.3),
    
    //new DriveDistance(-3.6, 1, 3.6, 1),
    new turnLeftDriveTrain(3.6),  //vuelta a la izquierda de 45 grados
    new WaitCommand(0.3),
    new DriveDistance(2, -1, 2, -1),    //retrocede 2 pies para prepararse para mas  vuelta
    new WaitCommand(0.3),
    //new DriveDistance(-3.9, 1, 3.9, 1),
    new turnLeftDriveTrain(3.6),   //vuelta a la izquierda de 45 grados
    new WaitCommand(0.3),

    new DriveDistance(13, -1, 13, -1),  //sube a charge station, es preferible subir y bajarnos del otro lado que  quedarnos a medias
    new WaitCommand(0.3)


    );
  }
}
