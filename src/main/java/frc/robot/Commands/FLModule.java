// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.QuadEncoders;
import frc.robot.Subsystems.Controller;

public class FLModule extends Command {

//Subsystems
Swerve s_Swerve;
Gyro s_Gyro;
QuadEncoders s_QuadEncoders;
Controller s_Controller;


//Swerve Variables
private double kp = 0.009; //proportional 
private double ki = 0.001; //integral
private double kd = 0.001; //derivative
PIDController pidSpeedControl = new PIDController(kp, ki, kd); 

//Gyro Variables
private double gyroAngle;


//Quad Variables
private double swervePosition;

//Controller Variables
private double controllerAngle;
private double controllermagnitude;
private double joystickDeadband = .1;
boolean previousAzimuthDirection = false;
boolean previousDriveDirection = false;

private CommandXboxController xboxController;

  public FLModule(Swerve d_Swerve, Gyro d_Gyro, QuadEncoders d_QuadEncoders, Controller d_Controller, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = d_Swerve;
    s_Gyro = d_Gyro;
    s_QuadEncoders = d_QuadEncoders;
    s_Controller = d_Controller;
    xboxController = driverController;

    addRequirements(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //take in quadrature encoder value
    //take in gyro rotation value
    //take in joystick magnitude and rotation values

    // put all inputs through math method 
    // get direciton and speed of motor
    //output direcoitn and speed of motor to arduino

    swervePosition = (s_QuadEncoders.getFLAzimuthEncoder()); // gets angle from -180 to 180 
    gyroAngle = -s_Gyro.getGyroYaw(); //gets angle from -180 to 180
    controllerAngle = Math.toDegrees(s_Controller.getControllerAngle(xboxController.getLeftX(), xboxController.getLeftY())); // from 0 to 2pi
    controllermagnitude = s_Controller.getControllerMagnitude(xboxController.getLeftX(), xboxController.getLeftY(), controllerAngle); // form 0 to 1

    //System.out.println(gyroAngle);


    // System.out.println("Drive Direciton: " + s_Swerve.CalculateDriveDirection(controllermagnitude, controllerAngle, swervePosition));
    // System.out.println("Drive speed: " + s_Swerve.CalculateDriveSpeed(controllermagnitude,controllerAngle,swervePosition));
    boolean azimuthdirection = s_Swerve.CalculateAzimuthDirection(controllerAngle, swervePosition, pidSpeedControl, gyroAngle);
    Double azimuthSpeed = s_Swerve.CalculateAzimuthSpeed(controllerAngle, swervePosition, pidSpeedControl, gyroAngle);

    boolean driveDirection = s_Swerve.CalculateDriveDirection(controllermagnitude, controllerAngle, swervePosition);
    Double driveSpeed = s_Swerve.CalculateDriveSpeed(controllermagnitude, controllerAngle, swervePosition, gyroAngle);

    //System.out.println(controllerAngle);
    //System.out.println(swervePosition);
    // System.out.println("Azimuth Direciton: " + azimuthdirection);
    // System.out.println("Azimuth speed: " + azimuthSpeed);

    if(controllermagnitude > joystickDeadband) {
      s_Swerve.setFLAzimuth(azimuthSpeed,azimuthdirection); //------------------------------------------
      s_Swerve.setFLDrive(driveSpeed,true); //------------------------------------------
      previousAzimuthDirection = azimuthdirection;
      previousDriveDirection = driveDirection;
    } else{
      s_Swerve.setFLAzimuth(0,previousAzimuthDirection);
      s_Swerve.setFLDrive(0,false); //------------------------------------------

      //s_Swerve.setFLDrive(0,previousDriveDirection);
    }     

    

    // pidSpeedControl.setSetpoint(100);           // target value
    // pidSpeedControl.setTolerance(2);            // +- x units from target value is considered target reached
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
