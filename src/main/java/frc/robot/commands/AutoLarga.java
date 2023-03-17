// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLarga extends SequentialCommandGroup {
  /** Creates a new AutoCorta. */
  public AutoLarga() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    //////////////////////////////////////////////// AUTONOMO LARGA  ////////////////////////////////////////////////////////////

    new OffArm(),
    new WaitCommand(0.8),
    new ExtendArm(),
    new WaitCommand(0.8),
    new IntakeOn(),
    new WaitCommand(0.5),
    new RetractArm(),
    new WaitCommand(0.5),
    new OnArmT(),
    new WaitCommand(0.3),
    new DriveDistance(15, -1, 15, -1, false)    //retrocede 18 pies
    //Edgar quiere 12
    
    /*
    new WaitCommand(0.3),

    //new DriveDistance(3.7, 1, -3.7, 1),
    new TurnRightDriveTrain(3.67),  //vuelta a la derecha de 45 grados
    new WaitCommand(0.1),
    new DriveDistance(2, -1, 2, -1),   //retrocede 2 pies para prepararse para mas  vuelta
    new WaitCommand(0.1),
    //new DriveDistance(3.7, 1, -3.7, 1),
    new TurnRightDriveTrain(3.67),       //vuelta a la derecha de 45 grados
    new WaitCommand(0.1),

    new DriveDistance(3, 1, 3, 1),   //avanza a altura de charge station
    new WaitCommand(0.1),
  
    //new DriveDistance(3.7, 1, -3.7, 1),
    new TurnRightDriveTrain(3.67),       //vuelta a la derecha de 45 grados
    new WaitCommand(0.1),
    new DriveDistance(2, -1, 2, -1),  //retrocede 2 pies para prepararse para mas  vuelta
    new WaitCommand(0.1),
    new TurnRightDriveTrain(3.67),       //vuelta a la derecha de 45 grados
   // new DriveDistance(3.7, 1, -3.7, 1),
    new WaitCommand(0.1),

    new DriveDistance(12.7, -1, 12.7, -1),  //sube a charge station, es preferible subir y bajarnos del otro lado que  quedarnos a medias
    new WaitCommand(0.3)

    */
    );
  }
}
