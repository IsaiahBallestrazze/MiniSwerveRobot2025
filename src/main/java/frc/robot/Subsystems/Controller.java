// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends SubsystemBase {
  /** Creates a new Controller. */
  public Controller() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean Ypressed(CommandXboxController controller){
    System.out.println("Y " + controller.a().getAsBoolean());
    return controller.y().getAsBoolean();
  }

  public double getControllerAngle(double joystickX, double joystickY){
    double angle = (-Math.atan2(joystickY, -joystickX));
    return angle;
  }

  public double getControllerMagnitude(double joystickX, double joystickY, double angle){
    double rawMagnitude = Math.sqrt((joystickX*joystickX) + (joystickY* joystickY));
    // double scaledMagnitude1 = (rawMagnitude / ((joystickY / joystickX) /Math.sin(angle))); //for square joysticks
    // double scaledMagnitude2 = (rawMagnitude / ((joystickX / joystickY) /Math.cos(angle)));
    //return (Math.abs(scaledMagnitude1) > Math.abs(scaledMagnitude2)) ? Math.abs(scaledMagnitude1) : Math.abs(scaledMagnitude2);
    return (rawMagnitude > 1) ? 1.0 : rawMagnitude;
  }
  
}
