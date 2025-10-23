// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrintJoy extends Command {
  /** Creates a new printJoy. */

private double joystickX;
private double joystickY;
private double joystickMagnitude;

private Controller s_controller;
private CommandXboxController s_CommandXboxController;

  public PrintJoy(Controller d_controller,  CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_controller = d_controller;
    s_CommandXboxController = driverController;
    addRequirements(s_controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    joystickX = s_CommandXboxController.getLeftX();
    joystickY =  s_CommandXboxController.getLeftY();
    double joystickAngle;
    joystickAngle = s_controller.getControllerAngle(joystickX, joystickY);
    joystickMagnitude = s_controller.getControllerMagnitude(joystickX,joystickY,joystickAngle);
    System.out.println("Joystick Magnitude = " + joystickMagnitude + " Joystick Angle = " + joystickAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(joystickMagnitude) < .1;
  }
}
