// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.Manifest;
import frc.robot.configuration.Manifest.ControlsBuilder;
import frc.robot.configuration.Manifest.DrivetrainBuilder;
import frc.robot.configuration.bindings.DriverBindings;
import frc.robot.configuration.bindings.OperatorBindings;
import frc.robot.modules.individuals.Dumper.Dumper;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.indexer.Indexer;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.tests.EmptyTest;

public class RobotContainer implements IRobotContainer{

  public final ControllerOI driver;

  public final ControllerOI operator;

  public final CommandSwerveDrivetrain drivetrain;

  public final Intake intake;

  public final Flywheels shooter;

  public final Indexer indexer;

  public final Dumper dumper;

  public RobotContainer() {

    this.driver = ControlsBuilder.buildDriver();

    this.operator = ControlsBuilder.buildOperator();

    // null si HAS_DRIVETRAIN = false en el Manifest
    this.drivetrain = DrivetrainBuilder.buildModule();

    this.intake = Manifest.buildIntake();

    this.shooter = Manifest.buildShooter();

    this.indexer = Manifest.buildIndexer();

    this.dumper = Manifest.builDumper();

    if (drivetrain != null) {
      DriverBindings.create(drivetrain, driver).bind();
    }

    OperatorBindings.create(operator, intake, shooter, indexer, dumper).bind();
  }

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
