// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Controller;
import frc.robot.Subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTest extends Command {
  /** Creates a new RotateTest. */
  Swerve s_Swerve;
  CommandXboxController s_Controller;
  Controller s_ControllerSub;
  public RotateTest(Swerve d_Swerve, CommandXboxController d_Controller, Controller d_ControllerSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = d_Swerve;
    s_Controller = d_Controller; 
    s_ControllerSub = d_ControllerSub;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.setBLDrive(1,true); //bevel to the left
    s_Swerve.setFLDrive(1,true);
    s_Swerve.setBRDrive(1,true);
    s_Swerve.setFRDrive(1,true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.StopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_ControllerSub.Ypressed(s_Controller);
  }
}
