// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AnalogOut extends InstantCommand {

private Gyro s_gyro;
private boolean realState;

  public AnalogOut(Gyro d_gyro, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_gyro = d_gyro;
    realState = state;
    addRequirements(s_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(realState){
      s_gyro.SetSafetyPinEnabled();
    }else{
      s_gyro.SetSafetyPinDisabled();
    }

  }
}
