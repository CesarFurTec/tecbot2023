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
    addCommands(

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
    new DriveDistance(18, -1, 18, -1),
    new WaitCommand(0.3),
    //new DriveDistance(-5.7, 1, 5.7, 1),

    new DriveDistance(-3.6, 1, 3.6, 1),
    new WaitCommand(0.3),
    new DriveDistance(2, -1, 2, -1),
    new WaitCommand(0.3),
    new DriveDistance(-3.6, 1, 3.6, 1),
    new WaitCommand(0.3),

    new DriveDistance(1.5, 1, 1.5, 1),
    new WaitCommand(0.3),

    /*new IntakeOn(),
    new WaitCommand(0.3),
    new IntakeOff(),*/
    
    new DriveDistance(2, 1, 2, 1),

    new DriveDistance(-3.6, 1, 3.6, 1),
    new WaitCommand(0.3),
    new DriveDistance(2, -1, 2, -1),
    new WaitCommand(0.3),
    new DriveDistance(-3.9, 1, 3.9, 1),
    new WaitCommand(0.3),

    /*new DriveDistance(-2, 1, 2, 1),
    new WaitCommand(0.3),
    new DriveDistance(1.1, -1, 1.1, -1),
    new WaitCommand(0.3),
    new DriveDistance(-2, 1, 2, 1),
    new WaitCommand(0.3),
    new DriveDistance(1.1, 1, 1.1, 1),
    new WaitCommand(0.3),
    new DriveDistance(-2, 1, 2, 1),
    new WaitCommand(0.3),
    new DriveDistance(1.1, -1, 1.1, -1),
    new WaitCommand(0.3),*/

    new DriveDistance(13, -1, 13, -1),
    new WaitCommand(0.3)






    );
  }
}