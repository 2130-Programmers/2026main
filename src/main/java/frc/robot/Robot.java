// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
  
  // Print absolute encoder positions - access modules directly
  //System.out.println("Module 0 abs: " + m_robotContainer.swerve.frontLeft.getAbsolutePosition());
  //System.out.println("Module 1 abs: " + m_robotContainer.swerve.frontRight.getAbsolutePosition());
  //System.out.println("Module 2 abs: " + m_robotContainer.swerve.backLeft.getAbsolutePosition());
  //System.out.println("Module 3 abs: " + m_robotContainer.swerve.backRight.getAbsolutePosition());
}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
  @Override
  public void autonomousInit() {
      Command auto = m_robotContainer.getAutonomousCommand();
      
      // Reset pose to match path start
      if (auto instanceof PathPlannerAuto) {
          m_robotContainer.getSwerve().resetOdometry(
              ((PathPlannerAuto) auto).getStartingPose()
          );
      }
      
      m_autonomousCommand = auto;
      if (m_autonomousCommand != null) {
          CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
  }
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}