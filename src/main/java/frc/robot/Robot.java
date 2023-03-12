// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmEncoder;
import frc.robot.commands.AutoMonterrey01;
import frc.robot.commands.EncoderArmBackwards;
import frc.robot.commands.GetArmAngleL;
import frc.robot.commands.GoUpChargedStation;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.MoveArm;
import frc.robot.commands.OnArm;
import frc.robot.commands.OpenAndCloseClaw;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ResetEncoderDt;
import frc.robot.commands.ReturnRobot;
import frc.robot.commands.TurnLeftDTE;
import frc.robot.commands.autonomusCommandGroup;
import frc.robot.commands.calibrateGyro;
import frc.robot.commands.driveForwardDT;
import frc.robot.commands.driveRobot;
import frc.robot.commands.AutoArm;
import frc.robot.commands.AutoCorta;
import frc.robot.commands.AutoLarga;
import frc.robot.commands.CameraCommand;
import frc.robot.commands.onMotors;
import frc.robot.commands.EncoderReader;
import frc.robot.commands.turnLeftDriveTrain;
import frc.robot.resources.Navx;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.driveTrain;
import frc.robot.RobotContainer;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static RobotContainer robotcontainer;
  private Command m_autonomousCommand;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    robotcontainer = new RobotContainer();
    robotcontainer.configureButtonBindings();
    cameraRetake();
   
  }
  public static RobotContainer getRobotContainer() {

    return robotcontainer;
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("angle", Navx.getGyro());
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotcontainer.getAutonomousCommand();
    

    AutoLarga auto_larga = new AutoLarga();
    System.out.println("autoLarga_On");
    auto_larga.schedule();

    /* 
    AutoCorta auto_corta = new AutoCorta();
    System.out.println("autoCorta_On");
    auto_corta.schedule();
    */

    /* 
    AutoArm auto_media = new AutoArm();
    System.out.println("autoMedia_On");
    auto_media.schedule(); 
    */



  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    driveRetake();
  }

  public static void cameraRetake()
  {
    new Thread(() -> {

      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setWhiteBalanceAuto();
      camera.setFPS(12);

      CvSink cvSink = CameraServer.getVideo();

      CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start(); 
  }

  public static void driveRetake()
  {

    driveRobot t2 = new driveRobot();
    
   

    t2.schedule();
    //
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


  }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

 
}
