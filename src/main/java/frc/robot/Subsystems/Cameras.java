// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {
  /** Creates a new Camera. */
  public Cameras() {}

public void StartCamera(){
UsbCamera RobotEye = CameraServer.startAutomaticCapture();
RobotEye.setResolution(1280, 720);
RobotEye.setFPS(30);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
