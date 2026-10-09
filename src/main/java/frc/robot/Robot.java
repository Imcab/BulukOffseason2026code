// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.test.TestScheduler;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.Manifest;

public class Robot extends TimedRobot {


  private Command m_autonomousCommand;
  private final IRobotContainer m_robotContainer;

  public Robot() {

    Environment.setMode(Manifest.CURRENT_MODE);
    
    m_robotContainer = new RobotContainer();
  
    NetworkIO.set("System", "IO", Environment.getMode().name());
    NetworkIO.set("System", "isOnSim", RobotBase.isSimulation());

    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.updateNodes();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    CommandScheduler.getInstance()
        .schedule(TestScheduler.runTest(m_robotContainer.getTestRoutine()));
  }

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testExit() {}
}
