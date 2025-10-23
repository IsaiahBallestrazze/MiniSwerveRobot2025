// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AzimuthRotate extends Command {
  /** Creates a new MotorMove. */
  Swerve s_swerve;
  double s_speed;
  boolean s_direction;
  public AzimuthRotate(Swerve d_Swerve, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_swerve = d_Swerve;
    s_speed = speed;
    addRequirements(s_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //call a motor

    System.out.println("Execute called");


    s_direction = s_speed < 0 ? false : true;
    s_swerve.setFLAzimuth(Math.abs(s_speed),s_direction);
    // s_swerve.setFRAzimuth(Math.abs(s_speed),s_direction);
    // s_swerve.setBLAzimuth(Math.abs(s_speed),s_direction);
    // s_swerve.setBRAzimuth(Math.abs(s_speed),s_direction);

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
