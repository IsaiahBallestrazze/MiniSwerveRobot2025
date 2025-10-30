// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.QuadEncoders;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Controller;
import frc.robot.Subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ModuleGroup extends SequentialCommandGroup {
  /** Creates a new ModuleGroup. */
  public ModuleGroup(Swerve s_Swerve, Gyro s_Gyro, QuadEncoders s_QuadEncoders, Controller s_Controller,CommandXboxController driverController) {

    addCommands( //calls each swerve module to do its thing
    new FRModule(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller, driverController),
    new FLModule(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller, driverController),
    new BRModule(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller, driverController),
    new BLModule(s_Swerve, s_Gyro, s_QuadEncoders, s_Controller, driverController)
    );
  }
}
