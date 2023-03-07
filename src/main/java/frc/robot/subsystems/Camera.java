// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

//import edu.wpi.first.wpilibj.CameraServer;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  public Camera() {}

  @Override
  public void periodic() {
    System.out.println("camera_periodic");
    // This method will be called once per scheduler run
    new Thread(() -> {
                //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(640, 480);

                //CvSink cvSink = CameraServer.getInstance().getVideo();
                CvSink cvSink = CameraServer.getVideo();
                
                //CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

                CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

                Mat source = new Mat();
                Mat output = new Mat();

                while(!Thread.interrupted()) {
                    cvSink.grabFrame(source);
                    Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                    outputStream.putFrame(output);
                }
            }).start();
  }
}
