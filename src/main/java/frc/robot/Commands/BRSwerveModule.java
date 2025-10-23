// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BRSwerveModule extends Command {
  /** Creates a new BRSwerveModule. */
  public BRSwerveModule() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //take in the current quadrature encoder position of the azimuth motor
    //take in the current joystick angle
    // take in the current gyro angle 
    //calculate the needed speed and direction of the azimuth motor to get to that direction
    //calculate the needed speed direction of the drive motor based on the joystick input and the direction of the azimuth motor and correct using cos correction
    // output the speed and direction to the azimuth motor
    // output the speed and direction of the drive motor. 

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
