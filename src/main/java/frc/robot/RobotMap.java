// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

/** Add your docs here. */
public class RobotMap {

public static final double OFFSET = 0.05;

public static final int driveTrainPorts[] = {13, 15, 20, 51, 50}; 

//Left:13,15
//Right: 

public static final int armPorts[] = {1,2};

public static final TypeOfMotor chassisMotor[] = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};

public static final int pilotPort = 0;

public static final int copilotPort = 1;

public static final double chassisSpeedL = 0.5;

public static final double chassisSpeedR = 0.5;

public static final double autonomusCharge = 0.6; // 

public static final double autonomusSpeed = 0.5; // no mover, estaba en 0.28 -_-

public static final double childSafetySpeed = 0.1; // 

public static final double armSpeedB = 1;

public static final double armSpeedE = 0.2;

public static final double armspeedF = 0.30;

public static final double teleop_armspeed = 0.3;

public static final int PCM_1_PORT = 7;

public static final int SolenoidPortClaw[] = {PCM_1_PORT, 5, 10};

public static final int SolenoidPortTransmition[] = {PCM_1_PORT, 4, 11};

public static final int SolenoidPortArm[] = {PCM_1_PORT, 2 , 13};

public static final int SolenoidPortExtentionArm[] = {PCM_1_PORT, 3, 12};

public static final int kGyroPort = 0;

public static final double encoderTolerance = 0.12;
}

