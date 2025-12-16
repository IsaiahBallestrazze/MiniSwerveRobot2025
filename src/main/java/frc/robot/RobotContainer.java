// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AzimuthRotate;
import frc.robot.Commands.FLModule;
import frc.robot.Commands.Modules;
import frc.robot.Commands.MotorMove;
import frc.robot.Commands.RotateTest;
import frc.robot.Subsystems.Cameras;
import frc.robot.Subsystems.Controller;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.QuadEncoders;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  Swerve s_swerve = new Swerve();
  Gyro s_gyro = new Gyro();
  QuadEncoders s_QuadEncoders = new QuadEncoders();
  Controller s_controller = new Controller();
  Cameras s_Cameras = new Cameras();


    private final CommandXboxController driverController = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    s_swerve.setDefaultCommand(getModuleCommand());
    driverController.a().onTrue(new InstantCommand(()-> s_QuadEncoders.resetEncoders()));
    driverController.a().onTrue(new InstantCommand(()-> s_gyro.resetGyro()));
    driverController.b().onTrue(new InstantCommand(() -> s_Cameras.StartCamera()));
    driverController.y().onTrue(new RotateTest(s_swerve, driverController, s_controller));
  }

  public double GetleftJoyX(){
    return driverController.getLeftX();
  }

  public double GetleftJoyY(){
    return driverController.getLeftY();
  }

public Swerve GetSwerve() {
  return s_swerve;
}

public Gyro GetGyro() {
  return s_gyro;
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getModuleCommand(){
    return new Modules(s_swerve, s_gyro, s_QuadEncoders, s_controller, s_Cameras, driverController);
  }

}
