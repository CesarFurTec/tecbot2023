// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMonterrey01 extends SequentialCommandGroup {
  /** Creates a new AutonomousSequence1. */
  public AutoMonterrey01() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println("viva monterrey ajua");
    addCommands(
    new TransmissionChange(true),
    new WaitCommand(1.0),
    new ResetEncoderDt(),
    new WaitCommand(1.0),
    new DriveDistance(2, 2, 1, -1),
    new WaitCommand(1.0),
    new ResetEncoderDt(),
    new WaitCommand(1.0),
    new DriveDistance(2, 2,- 1, 1), 
    new WaitCommand(1.0),
    new OffMotors()

    );
  }
}
