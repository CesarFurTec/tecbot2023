// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.driveForwardDT;
import frc.robot.commands.onMotors;
import frc.robot.commands.turnLeftDriveTrain;
import frc.robot.resources.Navx;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSpeedController;

public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain2. */
  TecbotSpeedController m1;
  TecbotSpeedController m2;
  TecbotSpeedController m3;
  TecbotSpeedController m4; 
  boolean leftSideBalanced = false, rightSideBalanced = false;
  


  RelativeEncoder driveTrainEncoderL1, driveTrainEncoderL2, driveTrainEncoderR1, driveTrainEncoderR2;

  DoubleSolenoid transmition;

  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort);
  
 
// m1 & m2 left 

  public driveTrain() {
  m1 = new TecbotSpeedController(RobotMap.driveTrainPorts[1], RobotMap.chassisMotor[0]);
  m2 = new TecbotSpeedController(RobotMap.driveTrainPorts[3], RobotMap.chassisMotor[1]); 
  m3 = new TecbotSpeedController(RobotMap.driveTrainPorts[0], RobotMap.chassisMotor[1]);
  m4 = new TecbotSpeedController(RobotMap.driveTrainPorts[2], RobotMap.chassisMotor[1]);
  
  m1.getCANSparkMax().setIdleMode(IdleMode.kBrake);
  m2.getCANSparkMax().setIdleMode(IdleMode.kBrake);
  m3.getCANSparkMax().setIdleMode(IdleMode.kBrake);
  m4.getCANSparkMax().setIdleMode(IdleMode.kBrake);
  
  driveTrainEncoderL1 = m1.getCANSparkMax().getEncoder();
  driveTrainEncoderL2 = m2.getCANSparkMax().getEncoder();
  driveTrainEncoderR1 = m3.getCANSparkMax().getEncoder();
  driveTrainEncoderR2 = m4.getCANSparkMax().getEncoder();
  
  transmition = RobotConfigurator.buildDoubleSolenoid(RobotMap.SolenoidPortTransmition);
  
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void driveForwardWithEncoders(){
  double sensorPosition =  -1 * driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;

  double error = TecbotConstants.setpoint - sensorPosition;

  System.out.println("distance: " + TecbotConstants.setpoint);

  double outputSpeed =( TecbotConstants.kP*error/TecbotConstants.setpoint) * RobotMap.chassisSpeedR;

  SmartDashboard.putNumber("Drivetrain distance: ", sensorPosition);
  SmartDashboard.putNumber("setpoint: ", TecbotConstants.setpoint);
  SmartDashboard.putNumber("error: ", error);
  SmartDashboard.putNumber("output speed: ", outputSpeed);
   
   m1.set(-outputSpeed);
   m2.set(-outputSpeed); 
   m3.set(outputSpeed);
   m4.set(outputSpeed); 
  }

  public void driveWithEncoders(double newSetPoint,int dir){

    System.out.println("drivewencoders" + newSetPoint);
    //double sensorPosition =  -1 * driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;
    double sensorPosition =  driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;

    double error = newSetPoint - Math.abs(sensorPosition); //adding the abs
  
    System.out.println("distance: " + newSetPoint);


    double outputSpeed = TecbotConstants.kP*error/(newSetPoint + 0.0001);
  
    SmartDashboard.putNumber("Drivetrain distance: ", sensorPosition);
    SmartDashboard.putNumber("setpoint: ",newSetPoint);
    SmartDashboard.putNumber("error: ", error);
    SmartDashboard.putNumber("output speed: ", outputSpeed);

     m1.set(-outputSpeed*dir);
     m2.set(-outputSpeed*dir); 
     m3.set(outputSpeed*dir);
     m4.set(outputSpeed*dir); 
    }

  public boolean moveWithEncodersL (double current, double max, double dir)
  {
    System.out.println("current:"+ current + "max"+ max);
    if(dir == 1)
    {
      if(current >= max - 1)
      {
        System.out.println(" FRONT  FRONT  STOP STOP STOP");
        m1.set(0);
        //m2.set(0);
        //m3.set(0);
        //m4.set(0); 
        return true;
      }else
      {
        System.out.println("////////////////////////  FRONT  ////////////////////////////////");
        m1.set(-RobotMap.chassisSpeedL);
        //m2.set(-RobotMap.chassisSpeedL); 
        //m3.set(RobotMap.chassisSpeedL);
        //m4.set(RobotMap.chassisSpeedL); 
        return false;
      }
    }else if(dir == -1)
    {
      if(current <= (max * dir) + 1)
      {
        System.out.println("BACK  BACK   STOP STOP STOP");
        m1.set(0);
        //m2.set(0);
        //m3.set(0);
        //m4.set(0);
        return true; 
      }else
      {
        System.out.println("////////////////////  BACK   ///////////////////////");
        m1.set(RobotMap.chassisSpeedL);
        //m2.set(RobotMap.chassisSpeedL); 
        //m3.set(-RobotMap.chassisSpeedL);
        //m4.set(-RobotMap.chassisSpeedL); 
        return false;
      }
    }
     
    return false;
  }

  public boolean moveWithEncodersL2 (double current, double max, double dir)
  {
    System.out.println("current:"+ current + "max"+ max);
    if(dir == 1)
    {
      if(current >= max - 1)
      {
        System.out.println(" FRONT  FRONT  STOP STOP STOP");
        m2.set(0);
      
        return true;
      }else
      {
        System.out.println("////////////////////////  FRONT  ////////////////////////////////");
       
        m2.set(-RobotMap.chassisSpeedL); 
         
        return false;
      }
    }else if(dir == -1)
    {
      if(current <= (max * dir) + 1)
      {
        System.out.println("BACK  BACK   STOP STOP STOP");
        
        m2.set(0);
        
        return true; 
      }else
      {
        System.out.println("////////////////////  BACK   ///////////////////////");
        
        m2.set(RobotMap.chassisSpeedL); 
        
        return false;
      }
    }
     
    return false;
  }

  public boolean moveWithEncodersR (double current, double max, double dir)
  {
    System.out.println("current:"+ current + "max"+ max);
    if(dir == 1)
    {
      if(current >= max - 1)
      {
        System.out.println(" FRONT  FRONT  STOP STOP STOP");
        m3.set(0);
        //m4.set(0); 
        return true;
      }else
      {
        System.out.println("////////////////////////  FRONT  ////////////////////////////////");
        m3.set(RobotMap.chassisSpeedL);
        //m4.set(RobotMap.chassisSpeedL); 
        return false;
      }
    }else if(dir == -1)
    {
      if(current <= (max * dir) + 1)
      {
        System.out.println("BACK  BACK   STOP STOP STOP");
        m3.set(0);
        //m4.set(0);
        return true; 
      }else
      {
        System.out.println("////////////////////  BACK   ///////////////////////");
        m3.set(-RobotMap.chassisSpeedL);
        //m4.set(-RobotMap.chassisSpeedL); 
        return false;
      }
    }
     
    return false;
  }

  public boolean moveWithEncodersR2 (double current, double max, double dir)
  {
    System.out.println("current:"+ current + "max"+ max);
    if(dir == 1)
    {
      if(current >= max - 1)
      {
        System.out.println(" FRONT  FRONT  STOP STOP STOP");
        m4.set(0); 
        return true;
      }else
      {
        System.out.println("////////////////////////  FRONT  ////////////////////////////////");

        m4.set(RobotMap.chassisSpeedL); 

        return false;
      }
    }else if(dir == -1)
    {
      if(current <= (max * dir) + 1)
      {
        System.out.println("BACK  BACK   STOP STOP STOP");
        
        m4.set(0);
        return true; 
      }else
      {
        System.out.println("////////////////////  BACK   ///////////////////////");
        
        m4.set(-RobotMap.chassisSpeedL); 
        return false;
      }
    }
     
    return false;
  }




  public void driveForwardWithEncodersShort(){
    double sensorPosition =  driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;
  
    double error = TecbotConstants.setpointShort - sensorPosition;
  
    System.out.println("distance: " + TecbotConstants.setpoint);
  
    double outputSpeed = TecbotConstants.kP*error/TecbotConstants.setpoint;
  
    SmartDashboard.putNumber("Drivetrain distance: ", sensorPosition);
    SmartDashboard.putNumber("setpoint: ", TecbotConstants.setpoint);
    SmartDashboard.putNumber("error: ", error);
    SmartDashboard.putNumber("output speed: ", outputSpeed);
     
     m1.set(-outputSpeed);
     m2.set(-outputSpeed); 
     m3.set(outputSpeed);
     m4.set(outputSpeed); 
    }

  public void resetEncoderDt(){
    driveTrainEncoderL1.setPosition(0);
    driveTrainEncoderL2.setPosition(0);
    driveTrainEncoderR1.setPosition(0);
    driveTrainEncoderR2.setPosition(0);
  }  

  public double getDriveTrainFeetL(){
   return -1 * driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;
  }
  
  public double getDriveTrainFeetR(){
    //return driveTrainEncoderR1.getPosition();
    return  driveTrainEncoderR1.getPosition() * TecbotConstants.kDriveTick2Feet;
  }

  public double getDriveTrainFeetL2(){
    return -1 * driveTrainEncoderL2.getPosition() * TecbotConstants.kDriveTick2Feet;
   }
   
   public double getDriveTrainFeetR2(){
     //return driveTrainEncoderR1.getPosition();
     return  driveTrainEncoderR2.getPosition() * TecbotConstants.kDriveTick2Feet;
   }

  public void driveForward(){
   m1.set(-RobotMap.chassisSpeedL);
   m2.set(-RobotMap.chassisSpeedL); 
   m3.set(RobotMap.chassisSpeedR);
   m4.set(RobotMap.chassisSpeedR); 
  }

  public void driveBackwards(){
    double sensorPositionBackWards = driveTrainEncoderL1.getPosition() * TecbotConstants.kDriveTick2Feet;

    double error = TecbotConstants.setpointBakcwards - sensorPositionBackWards; //maybe Math.abs

    double outputSpeedBackwards = TecbotConstants.kP*error/TecbotConstants.setpointBakcwards;

     m1.set(outputSpeedBackwards);
     m2.set(outputSpeedBackwards); 
     m3.set(-outputSpeedBackwards);
     m4.set(-outputSpeedBackwards); 
    }
  
  

  public void stop(){
    m1.set(0);
    m2.set(0);
    m3.set(0);
    m4.set(0);
  }
  
 // public boolean hasElapsed (double sec) {
    //sec = 1.5;
//return t.get() >= sec;
 //}
  public void turnLeft(){
    double sensorPostionTurn = driveTrainEncoderL1.getPosition()*TecbotConstants.kDriveTick2Feet;

    double errorTurn = TecbotConstants.setpointTurn - sensorPostionTurn;

    double outputSpeedTurn = TecbotConstants.kP*errorTurn/TecbotConstants.setpointTurn;

    SmartDashboard.putNumber("Encoder Position: ", sensorPostionTurn);
    SmartDashboard.putNumber("setpoint: ", TecbotConstants.setpointTurn);
    SmartDashboard.putNumber("error: ", errorTurn);
    SmartDashboard.putNumber("output speed: ", outputSpeedTurn);

    System.out.println("Encoder Position: " + sensorPostionTurn);
    System.out.println("setpoint: " + TecbotConstants.setpointTurn);
    System.out.println("error: " + errorTurn);
    System.out.println("output speed: " + outputSpeedTurn);
    

    //m1.set(outputSpeedTurn);
    //m2.set(outputSpeedTurn); 
    //m3.set(outputSpeedTurn);
    //m4.set(outputSpeedTurn); 
  }

  public void turnRight(){
    double sensorPostionTurn = -1*driveTrainEncoderR1.getPosition();

    double errorTurn = TecbotConstants.setpointTurnR - sensorPostionTurn;

    double outputSpeedTurn = TecbotConstants.kP*errorTurn/TecbotConstants.setpointTurn;

    SmartDashboard.putNumber("Encoder Position: ", sensorPostionTurn);
    SmartDashboard.putNumber("setpoint: ", TecbotConstants.setpointTurn);
    SmartDashboard.putNumber("error: ", errorTurn);
    SmartDashboard.putNumber("output speed: ", outputSpeedTurn);
    
    m1.set(-outputSpeedTurn);
    m2.set(-outputSpeedTurn); 
    m3.set(-outputSpeedTurn);
    m4.set(-outputSpeedTurn);
  }

  /*public void turn45(){
    double sensorPositionTurn = (-1*driveTrainEncoderR1.getPosition()) * TecbotConstants.kDriveTick2Feet;

   // double angleGet = sensorPositionTurn/45;

    double errorTurn = TecbotConstants.setpointTurnR - sensorPositionTurn;

    double outputSpeedTurn = TecbotConstants.kP*errorTurn/TecbotConstants.setpointTurn;

    m1.set(-outputSpeedTurn);
    m2.set(-outputSpeedTurn); 
    m3.set(-outputSpeedTurn);
    m4.set(-outputSpeedTurn);
  } 
  */

  //Nota de que solo hay que modificar el setpoint -_-

  public void drive(double x, double y){
    Robot.getRobotContainer().getOI().getPilot().setOffset(RobotMap.OFFSET);
    double rightSpeed = (-x - y);
    double leftSpeed = (-x + y);
    m1.set(leftSpeed);
    m2.set(leftSpeed);
    m3.set(rightSpeed);
    m4.set(rightSpeed);
  } 

  public void changeToSpeed(){
    transmition.set(Value.kForward);
    m1.getCANSparkMax().setIdleMode(IdleMode.kBrake);
    m2.getCANSparkMax().setIdleMode(IdleMode.kBrake);
    m3.getCANSparkMax().setIdleMode(IdleMode.kBrake);
    m4.getCANSparkMax().setIdleMode(IdleMode.kBrake);
  }

  public void changeToTorque(){
    transmition.set(Value.kReverse);
    m1.getCANSparkMax().setIdleMode(IdleMode.kCoast);
    m2.getCANSparkMax().setIdleMode(IdleMode.kCoast);
    m3.getCANSparkMax().setIdleMode(IdleMode.kCoast);
    m4.getCANSparkMax().setIdleMode(IdleMode.kCoast);
  }

  public void switchTransmission(Boolean transType)
  {
    if(transType == true)
    {
      changeToTorque();
    }else
    {
      changeToSpeed();
    }
  }

  public double getAngle(){
  System.out.println(gyro.getAngle());
  SmartDashboard.putNumber("Angle: ", gyro.getAngle());
  return gyro.getAngle();
  }

  public void driveBackwardsTime(){
    m1.set(RobotMap.chassisSpeedL);
    m2.set(RobotMap.chassisSpeedL);
    m3.set(-RobotMap.chassisSpeedR);
    m4.set(-RobotMap.chassisSpeedR); 
  }

  public void balanceL (double initEncoderL, double initEncoderR)
  {
    double decimalValL =initEncoderL %= 1;  

    System.out.println("Valor decimal ="+ decimalValL);

    double encoderMultL= initEncoderL - decimalValL;

    int sideVarianceNegative = 1;

    if(driveTrainEncoderL1.getPosition() > (initEncoderL - RobotMap.encoderTolerance) &&  driveTrainEncoderL1.getPosition() < (initEncoderL + RobotMap.encoderTolerance)){ //no girado
      m1.set(0);
      m2.set(0);
      leftSideBalanced = true;
    }
    else if (driveTrainEncoderL1.getPosition() < (initEncoderL - RobotMap.encoderTolerance)) //giradohaciatras
    {    
      m1.set(RobotMap.chassisSpeedL *sideVarianceNegative* ((encoderMultL + RobotMap.encoderTolerance)/10));
      m2.set(RobotMap.chassisSpeedL *sideVarianceNegative* ((encoderMultL + RobotMap.encoderTolerance)/10));  
      leftSideBalanced = false;    
    }

    else if(driveTrainEncoderL1.getPosition() > (initEncoderL + RobotMap.encoderTolerance)) //girado haia adelante
    {
      
      m1.set(-RobotMap.chassisSpeedL *sideVarianceNegative* ((encoderMultL + RobotMap.encoderTolerance)/10));
      m2.set(-RobotMap.chassisSpeedL *sideVarianceNegative* ((encoderMultL + RobotMap.encoderTolerance)/10));
      leftSideBalanced = false;
    }

    double decimalValR = initEncoderR %= 1;  

    System.out.println("Valor decimal ="+ decimalValR);

    double encoderMultR= initEncoderR - decimalValR;

    if(driveTrainEncoderR1.getPosition() > (initEncoderR - RobotMap.encoderTolerance) &&  driveTrainEncoderR1.getPosition() < (initEncoderR + RobotMap.encoderTolerance)){ //no girado
      m3.set(0);
      m4.set(0);
      rightSideBalanced = true;
    }
    else if (driveTrainEncoderR1.getPosition() < (initEncoderR - RobotMap.encoderTolerance)) //giradohaciatras
    {    
      m3.set(-RobotMap.chassisSpeedR *sideVarianceNegative* ((encoderMultR + RobotMap.encoderTolerance)/10));
      m4.set(-RobotMap.chassisSpeedR *sideVarianceNegative* ((encoderMultR + RobotMap.encoderTolerance)/10));      
      rightSideBalanced = false;
    }

    else if(driveTrainEncoderR1.getPosition() > (initEncoderR + RobotMap.encoderTolerance)) //girado haia adelante
    {
      
      m3.set(RobotMap.chassisSpeedR *sideVarianceNegative* ((encoderMultR + RobotMap.encoderTolerance)/10));
      m4.set(RobotMap.chassisSpeedR *sideVarianceNegative* ((encoderMultR + RobotMap.encoderTolerance)/10));
      rightSideBalanced = false;
    }
    
  }

  public boolean getLeftBalance()
  {
    return leftSideBalanced;
  }
  public boolean getRightBalance()
  {
    return rightSideBalanced;
  }

  public double getLeftEncoder()
  {
    return driveTrainEncoderL1.getPosition();
  }
  public double getRightEncoder()
  {
    return driveTrainEncoderR1.getPosition();
  }

  public void resetEncoders()
  {
    driveTrainEncoderL1.setPosition(0);
    driveTrainEncoderL2.setPosition(0);
    driveTrainEncoderR2.setPosition(0);
    driveTrainEncoderR1.setPosition(0);
  }

  public void calibrateGyro()
  {

  }

  public void setAngle()
  {
    
  }

}


