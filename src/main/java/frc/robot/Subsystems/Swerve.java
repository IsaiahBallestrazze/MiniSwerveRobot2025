// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//shift + alt + f to autoformat

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  // Direction Pins to Arduino
  private final VictorSP FRDriveDirectionPin = new VictorSP(0); //PWM
  private final VictorSP FRAzimuthDirectionPin = new VictorSP(1); //PWM

  private final DigitalOutput FLDriveDirectionPin = new DigitalOutput(2);
  private final DigitalOutput FLAzimuthDirectionPin = new DigitalOutput(3);

  private final DigitalOutput BRDriveDirectionPin = new DigitalOutput(6);
  private final DigitalOutput BRAzimuthDirectionPin = new DigitalOutput(7);

  private final DigitalOutput BLDriveDirectionPin = new DigitalOutput(5);
  private final DigitalOutput BLAzimuthDirectionPin = new DigitalOutput(4);

  // Speed Pins to arduino

  private final VictorSP FRDriveSpeedPin = new VictorSP(2);
  private final VictorSP FRAzimuthSpeedPin = new VictorSP(3);

  private final VictorSP FLDriveSpeedPin = new VictorSP(4);
  private final VictorSP FLAzimuthSpeedPin = new VictorSP(5);

  private final VictorSP BRDriveSpeedPin = new VictorSP(6);
  private final VictorSP BRAzimuthSpeedPin = new VictorSP(7);

  private final VictorSP BLDriveSpeedPin = new VictorSP(8);
  private final VictorSP BLAzimuthSpeedPin = new VictorSP(9);

  // Quadrature Encoders for azimuth motors

    // --- Front Right Azimuth Encoder ---



  public Swerve() {
    //distance per pulse. obtained by getting the CPR (counts per revolution), wheel diameter, and gear ratio
  }

  
//SPECIAL CASE FR
public void setFRDrive(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  
  double outputVolts = direction ? 5.0 : 0.0;
  FRDriveDirectionPin.setVoltage(outputVolts);
  FRDriveSpeedPin.set(newSpeed);

  System.out.println("MovingMotor at " + newSpeed + " in " + direction + "direction");

}

public void setFRAzimuth(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  double outputVolts = direction ? 5.0 : 0.0;
  FRAzimuthDirectionPin.setVoltage(outputVolts);
  FRAzimuthSpeedPin.set(newSpeed);
}

//FL
public void setFLDrive(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  FLDriveDirectionPin.set(direction);
  FLDriveSpeedPin.set(newSpeed);
}

public void setFLAzimuth(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  FLAzimuthDirectionPin.set(direction);
  FLAzimuthSpeedPin.set(newSpeed);
}

//BR
public void setBRDrive(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  BRDriveDirectionPin.set(direction);
  BRDriveSpeedPin.set(newSpeed);
}

public void setBRAzimuth(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  BRAzimuthDirectionPin.set(direction);
  BRAzimuthSpeedPin.set(newSpeed);
}
//BL
public void setBLDrive(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  BLDriveDirectionPin.set(direction);
  BLDriveSpeedPin.set(newSpeed);
  System.out.print("Swerve Says ");
  System.out.println(direction);
}

public void setBLAzimuth(double speed, boolean direction){
  double newSpeed = map(speed, 0, 1, -1, 1);
  BLAzimuthDirectionPin.set(direction);
  BLAzimuthSpeedPin.set(newSpeed);
}


public void StopAllMotors(){
  setFRDrive(0, false);
  setFRAzimuth(0, false);

  setFLDrive(0, false);
  setFLAzimuth(0, false);

  setBRDrive(0, false);
  setBRAzimuth(0, false);

  setBLDrive(0, false);
  setBLAzimuth(0, false);
}

double map(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

// public double CalculateDriveDirection(){

// }

public double CalculateDriveSpeed(double controllerMagnitude, double controllerAngle, double AzimuthAngle){
  return Math.abs(Math.cos((AzimuthAngle - controllerAngle)) * controllerMagnitude);
}

public boolean CalculateDriveDirection(double controllerMagnitude, double controllerAngle, double AzimuthAngle){
  double speed = Math.cos((AzimuthAngle - controllerAngle)) * controllerMagnitude;
  boolean direction = (speed > 0) ? true : false;
  return direction;
}

public double CalculateAzimuthSpeed(double controllerAngle, double azimuthAngle, PIDController pidController, double gyroAngle){
  double difference = (azimuthAngle - controllerAngle) - gyroAngle;
  double setPoint;

  boolean method1 = ((difference <= 180) && (difference >= -180)) ? true: false;
  boolean method2 = (azimuthAngle >= 0) ? true: false;
  boolean method3 = (azimuthAngle < 0) ? true: false;

  if(method1){ // determines which equation to use to find shortest route go to https://www.desmos.com/calculator/tqoycuy5sz for a graph
    setPoint = -difference;
  } else if(method2){
    setPoint = -(difference-360);
  } else if(method3){
    setPoint = -(difference+360);
  } else{
    setPoint = 0;
  }

  double azimuthSpeed = pidController.calculate(0,setPoint);
  if(azimuthSpeed > 1) azimuthSpeed = 1;

  SmartDashboard.putNumber("Setpoint", setPoint);
  SmartDashboard.putNumber("Condition1", Math.abs(controllerAngle - azimuthAngle));  
  SmartDashboard.putNumber("Condition2", Math.abs((360 - (azimuthAngle - controllerAngle))));
  SmartDashboard.putNumber("Azimuth Speed", azimuthSpeed);

  return Math.abs(azimuthSpeed);
}

public Boolean CalculateAzimuthDirection(double controllerAngle, double azimuthAngle, PIDController pidController, double gyroAngle){

  double difference = (azimuthAngle - controllerAngle) - gyroAngle;
  double setPoint;

  boolean method1 = ((difference <= 180) && (difference >= -180)) ? true: false;
  boolean method2 = (azimuthAngle >= 0) ? true: false;
  boolean method3 = (azimuthAngle < 0) ? true: false;

  if(method1){
    setPoint = -difference;
  } else if(method2){
    setPoint = -(difference-360);
  } else if(method3){
    setPoint = -(difference+360);
  } else{
    setPoint = 0;
  }

  double azimuthSpeed = pidController.calculate(0,setPoint);
  boolean returnValue = (azimuthSpeed > 0) ? true : false;
  SmartDashboard.putBoolean("Direciton", returnValue);
  return returnValue;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
