// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Controller;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.QuadEncoders;
import frc.robot.Subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Modules extends Command {
  /** Creates a new Modules. */

  // Subsystems
  Swerve s_Swerve;
  Gyro s_Gyro;
  QuadEncoders s_QuadEncoders;
  Controller s_Controller;

  // Swerve Variables
  PIDController FRpidSpeedControl = new PIDController(0.003, 0.001, 0.001); // (kp, ki, kd) proportional, integral,
                                                                            // derivative
  PIDController FLpidSpeedControl = new PIDController(0.008, 0.001, 0.001); // (kp, ki, kd) proportional, integral,
                                                                            // derivative
  PIDController BRpidSpeedControl = new PIDController(0.008, 0.001, 0.001); // (kp, ki, kd) proportional, integral,
                                                                            // derivative
  PIDController BLpidSpeedControl = new PIDController(0.008, 0.001, 0.001); // (kp, ki, kd) proportional, integral,
                                                                            // derivative

  // Gyro Variables
  private double gyroAngle;

  // Quad Variables
  private double FRQuad;
  private double FLQuad;
  private double BRQuad;
  private double BLQuad;

  // Controller Variables
  private double controllerAngle;
  private double controllermagnitude;
  private double joystickDeadband = .1;
  boolean previousAzimuthDirection = false;
  boolean previousDriveDirection = false;

  private CommandXboxController xboxController;

  //memory variables
  boolean previousFLAzimuthDirection;
      boolean previousFRAzimuthDirection;
      boolean previousBLAzimuthDirection;
      boolean previousBRAzimuthDirection;

  public Modules(Swerve d_Swerve, Gyro d_Gyro, QuadEncoders d_QuadEncoders, Controller d_Controller,
      CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = d_Swerve;
    s_Gyro = d_Gyro;
    s_QuadEncoders = d_QuadEncoders;
    s_Controller = d_Controller;
    xboxController = driverController;

    addRequirements(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Azimuth encoder Positions
    FRQuad = -(s_QuadEncoders.getFRAzimuthEncoder()); // gets angle from -180 to 180
    FLQuad = (s_QuadEncoders.getFLAzimuthEncoder()); // gets angle from -180 to 180
    BRQuad = (s_QuadEncoders.getBRAzimuthEncoder()); // gets angle from -180 to 180
    BLQuad = -(s_QuadEncoders.getBLAzimuthEncoder()); // gets angle from -180 to 180

    SmartDashboard.putNumber("FR Encoder", FRQuad);  //puts in shuffleboard
    SmartDashboard.putNumber("FL Encoder", FLQuad);  
    SmartDashboard.putNumber("BR Encoder", BRQuad);  
    SmartDashboard.putNumber("BL Encoder", BLQuad);  

    // Gyro
    gyroAngle = -s_Gyro.getGyroYaw(); // gets angle from -180 to 180
    SmartDashboard.putNumber("Gyro", gyroAngle);  

    // Controller angle and magnitude
    controllerAngle = Math.toDegrees(s_Controller.getControllerAngle(xboxController.getLeftX(), xboxController.getLeftY())); // from 0 to 2pi
    controllermagnitude = s_Controller.getControllerMagnitude(xboxController.getLeftX(), xboxController.getLeftY(),controllerAngle); // form 0 to 1
    SmartDashboard.putNumber("Controller Angle", controllerAngle);  
    SmartDashboard.putNumber("Controller Magnitude", controllermagnitude);  
    
    // === AZIMUTH DIRECTIONS ===
    boolean FLAzimuthDirection = s_Swerve.CalculateAzimuthDirection(controllerAngle, FLQuad, FLpidSpeedControl, gyroAngle);
    boolean FRAzimuthDirection = s_Swerve.CalculateAzimuthDirection(controllerAngle, FRQuad, FRpidSpeedControl, gyroAngle);
    boolean BLAzimuthDirection = s_Swerve.CalculateAzimuthDirection(controllerAngle, BLQuad, BLpidSpeedControl, gyroAngle);
    boolean BRAzimuthDirection = s_Swerve.CalculateAzimuthDirection(controllerAngle, BRQuad, BRpidSpeedControl, gyroAngle);
    SmartDashboard.putBoolean("FL_AZ Direction", FLAzimuthDirection);  
    SmartDashboard.putBoolean("FR_AZ Direction", FRAzimuthDirection);  
    SmartDashboard.putBoolean("BL_AZ Direction", BLAzimuthDirection);  
    SmartDashboard.putBoolean("BR_AZ Direction", BRAzimuthDirection);  
    
    // === AZIMUTH SPEEDS ===
    double FLAzimuthSpeed = s_Swerve.CalculateAzimuthSpeed(controllerAngle, FLQuad, FLpidSpeedControl, gyroAngle);
    double FRAzimuthSpeed = s_Swerve.CalculateAzimuthSpeed(controllerAngle, FRQuad, FRpidSpeedControl, gyroAngle);
    double BLAzimuthSpeed = s_Swerve.CalculateAzimuthSpeed(controllerAngle, BLQuad, BLpidSpeedControl, gyroAngle);
    double BRAzimuthSpeed = s_Swerve.CalculateAzimuthSpeed(controllerAngle, BRQuad, BRpidSpeedControl, gyroAngle);
    SmartDashboard.putNumber("FL_AZ Speed", FLAzimuthSpeed);  
    SmartDashboard.putNumber("FR_AZ Speed", FRAzimuthSpeed);  
    SmartDashboard.putNumber("BL_AZ Speed", BLAzimuthSpeed);  
    SmartDashboard.putNumber("BR_AZ Speed", BRAzimuthSpeed);  

    // === DRIVE SPEEDS ===
    double FLDriveSpeed = s_Swerve.CalculateDriveSpeed(controllermagnitude, controllerAngle, FLQuad, gyroAngle);
    double FRDriveSpeed = s_Swerve.CalculateDriveSpeed(controllermagnitude, controllerAngle, FRQuad, gyroAngle);
    double BLDriveSpeed = s_Swerve.CalculateDriveSpeed(controllermagnitude, controllerAngle, BLQuad, gyroAngle);
    double BRDriveSpeed = s_Swerve.CalculateDriveSpeed(controllermagnitude, controllerAngle, BRQuad, gyroAngle);
    SmartDashboard.putNumber("FL_DR Speed", FLDriveSpeed);  
    SmartDashboard.putNumber("FR_DR Speed", FRDriveSpeed);  
    SmartDashboard.putNumber("BL_DR Speed", BLDriveSpeed);  
    SmartDashboard.putNumber("BR_DR Speed", BRDriveSpeed);  

    // === APPLY OUTPUTS ===
    if (controllermagnitude > joystickDeadband) {
      // --- Azimuth Motors ---
       s_Swerve.setFLAzimuth(FLAzimuthSpeed, FLAzimuthDirection);
       s_Swerve.setFRAzimuth(FRAzimuthSpeed, FRAzimuthDirection);
       s_Swerve.setBLAzimuth(BLAzimuthSpeed, BLAzimuthDirection);
       s_Swerve.setBRAzimuth(BRAzimuthSpeed, BRAzimuthDirection);

      // --- Drive Motors ---
       s_Swerve.setFLDrive(FLDriveSpeed, true);
       s_Swerve.setFRDrive(FRDriveSpeed, true);
       s_Swerve.setBLDrive(BLDriveSpeed, true);
       s_Swerve.setBRDrive(BRDriveSpeed, true);

      // Store previous azimuth directions per module
      boolean previousFLAzimuthDirection = FLAzimuthDirection;
      boolean previousFRAzimuthDirection = FRAzimuthDirection;
      boolean previousBLAzimuthDirection = BLAzimuthDirection;
      boolean previousBRAzimuthDirection = BRAzimuthDirection;

    } else {
      // --- Azimuth Motors (hold position with previous direction) ---
      s_Swerve.setFLAzimuth(0, previousFLAzimuthDirection);
      s_Swerve.setFRAzimuth(0, previousFRAzimuthDirection);
      s_Swerve.setBLAzimuth(0, previousBLAzimuthDirection);
      s_Swerve.setBRAzimuth(0, previousBRAzimuthDirection);

      // --- Drive Motors (stop all) ---
      s_Swerve.setFLDrive(0, false);
      s_Swerve.setFRDrive(0, false);
      s_Swerve.setBLDrive(0, false);
      s_Swerve.setBRDrive(0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
