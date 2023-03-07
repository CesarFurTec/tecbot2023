// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;
import frc.robot.resources.TecbotController.ButtonType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ChangeToSpeed;
import frc.robot.commands.ChangeToTorque;
import frc.robot.commands.EncoderArmBackwards;
import frc.robot.commands.EncoderArmForward;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.OffArm;
import frc.robot.commands.OffArmT;
import frc.robot.commands.OnArm;
import frc.robot.commands.OnArmT;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ResetEncoderDt;
import frc.robot.commands.RetractArm;

/** Add your docs here. */
public class OI {
   public static OI instance;
 private TecbotController Regina;
 private TecbotController Mario;
 

    public OI(){
      Regina = new TecbotController(RobotMap.pilotPort, TecbotConstants.CONTROLLER_TYPE_PILOT);
      Mario = new TecbotController(RobotMap.copilotPort, TecbotConstants.CONTROLLER_TYPE_COPILOT);

   }

   public void configureButtonBindings(){
      
     /* Regina.whenPressed(TecbotController.ButtonType.A, new OnArmT());
      Regina.whenPressed(TecbotController.ButtonType.B, new OffArm());
      Regina.whenPressed(TecbotController.ButtonType.X, new ChangeToSpeed());
      Regina.whenPressed(TecbotController.ButtonType.Y, new ChangeToTorque());
*/
     Regina.whenPressed(TecbotController.ButtonType.A, new IntakeOn());
      Regina.whenPressed(TecbotController.ButtonType.B, new IntakeOff());
      Regina.whenPressed(TecbotController.ButtonType.X, new ExtendArm());
      Regina.whenPressed(TecbotController.ButtonType.Y, new RetractArm());

      Regina.whenPressed(TecbotController.ButtonType.LB, new ChangeToSpeed());
      Regina.whenPressed(TecbotController.ButtonType.RB, new ChangeToTorque());
      
      

      Mario.whenPressed(TecbotController.ButtonType.A, new OnArmT()); // x
      Mario.whenPressed(TecbotController.ButtonType.B, new OffArm()); // o
     // Mario.whileHeld(TecbotController.ButtonType.X, new ChangeToSpeed()); // []
     // Mario.whileHeld(TecbotController.ButtonType.Y, new ChangeToTorque()); // A
   }

   public static OI getInstance() {
      if (instance == null)
          instance = new OI();

      return instance;
  }

   public TecbotController getPilot(){
      return Regina;
   }

   public TecbotController getCopilot(){
      return Mario;
   }

}
