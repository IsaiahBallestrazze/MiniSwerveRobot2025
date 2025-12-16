// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.SPI;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class Gyro extends SubsystemBase {

  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);  
  public AnalogOutput EnabledSafetyOutput = new AnalogOutput(1); //tells the arduino when enabled or disabled





  public Gyro() {}

  public double getGyroYaw(){
    return navx.getYaw();
  }

  public double getGyroPitch(){
    return navx.getYaw();
  }

  public double getGyroRoll(){
    return navx.getRoll();
  }

  public void SetSafetyPinEnabled(){
    EnabledSafetyOutput.setVoltage(5);
    System.out.println("SAFETY ENABLED");
  }

  public void SetSafetyPinDisabled(){
    EnabledSafetyOutput.setVoltage(0);
    System.out.println("SAFETY DISABLED");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetGyro(){
    navx.reset();
  }
  
}
