// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.GetSwerve().StopAllMotors();
    m_robotContainer.GetGyro().SetSafetyPinDisabled();

  }

  @Override
  public void disabledPeriodic() {}


  @Override
  public void disabledExit() {
    m_robotContainer.GetSwerve().StopAllMotors();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.GetGyro().SetSafetyPinEnabled();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    m_robotContainer.GetSwerve().StopAllMotors();
    m_robotContainer.GetGyro().SetSafetyPinEnabled();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    m_robotContainer.GetSwerve().StopAllMotors();
    m_robotContainer.GetGyro().SetSafetyPinDisabled();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.GetGyro().SetSafetyPinEnabled();

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    m_robotContainer.GetGyro().SetSafetyPinDisabled();
  }
}
