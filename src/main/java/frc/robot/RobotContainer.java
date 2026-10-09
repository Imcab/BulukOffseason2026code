// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.tests.EmptyTest;

public class RobotContainer implements IRobotContainer{

  public RobotContainer() {}

  @Override
  public void updateNodes() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  @Override
  public TestRoutine getTestRoutine() {
    return new EmptyTest();
  }
}
