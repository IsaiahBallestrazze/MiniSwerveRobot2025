// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AzimuthRotate;
import frc.robot.Commands.FLModule;
import frc.robot.Commands.Modules;
import frc.robot.Commands.MotorMove;
import frc.robot.Subsystems.Controller;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.QuadEncoders;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  Swerve s_swerve = new Swerve();
  Gyro s_gyro = new Gyro();
  QuadEncoders s_QuadEncoders = new QuadEncoders();
  Controller s_controller = new Controller();


    private final CommandXboxController driverController = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // driverController.povRight().onTrue(new AzimuthRotate(s_swerve, 1));
    // driverController.povRight().onFalse(new AzimuthRotate(s_swerve, 0));

    // driverController.povLeft().onTrue(new AzimuthRotate(s_swerve, -1));
    // driverController.povLeft().onFalse(new AzimuthRotate(s_swerve, 0));
    
    // // driverController.povUp().onTrue(new MotorMove(s_swerve, 1));
    // // driverController.povUp().onFalse(new MotorMove(s_swerve, 0));

    // // driverController.povDown().onTrue(new MotorMove(s_swerve, -1));
    // // driverController.povDown().onFalse(new MotorMove(s_swerve, 0));

    // //driverController.axisMagnitudeGreaterThan(1,.1).onTrue(new PrintJoy(s_controller, driverController));
    // driverController.a().onTrue(new FLModule(s_swerve, s_gyro, s_QuadEncoders, s_controller, driverController));
    s_swerve.setDefaultCommand(getModuleCommand());
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
    return new Modules(s_swerve, s_gyro, s_QuadEncoders, s_controller, driverController);
  }

}
