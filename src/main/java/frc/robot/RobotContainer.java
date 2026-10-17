// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.Manifest;
import frc.robot.configuration.Manifest.ControlsBuilder;
import frc.robot.configuration.Manifest.DrivetrainBuilder;
import frc.robot.configuration.bindings.AutoBindings;
import frc.robot.configuration.bindings.DriverBindings;
import frc.robot.configuration.bindings.OperatorBindings;
import frc.robot.configuration.constants.swerve.ChassisTuningDashboard;
import frc.robot.modules.individuals.Dumper.Dumper;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.indexer.Indexer;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.vision.Localization;
import frc.tests.EmptyTest;

public class RobotContainer implements IRobotContainer{

  public final ControllerOI driver;

  public final ControllerOI operator;

  public final CommandSwerveDrivetrain drivetrain;

  public final Intake intake;

  public final Flywheels shooter;

  public final Flywheels intakFlywheels;

  public final Indexer indexer;

  public final Dumper dumper;

  // null si no hay drivetrain: sin odometria no hay a que fusionarle la vision
  public final Localization localization;

  // Autos y comandos con nombre de PathPlanner. null si AutoBuilder no se configuro.
  private final AutoBindings autoBindings;

  public RobotContainer() {

    this.driver = ControlsBuilder.buildDriver();

    this.operator = ControlsBuilder.buildOperator();

    // null si HAS_DRIVETRAIN = false en el Manifest
    this.drivetrain = DrivetrainBuilder.buildModule();

    this.intake = Manifest.buildIntake();

    this.shooter = Manifest.buildShooter();

    this.indexer = Manifest.buildIndexer();

    this.intakFlywheels = Manifest.buildFlywheelIntake();

    this.dumper = Manifest.builDumper();

    this.localization = drivetrain != null ? new Localization(drivetrain) : null;

    if (drivetrain != null) {
      DriverBindings.create(drivetrain, driver, localization::secondsSinceLastVision).bind();

       ChassisTuningDashboard.publish(drivetrain);
    }

    // AutoBuilder se configura en el constructor del drivetrain (configurePathPlanner); si fallo
    // (sin drivetrain o sin settings.json) no hay autos que construir.
    if (drivetrain != null && AutoBuilder.isConfigured()) {
      // AutoBindings registra los comandos con nombre y crea los autos (en ese orden)
      this.autoBindings =
          AutoBindings.create(
              intake,
              shooter,
              indexer,
              intakFlywheels,
              drivetrain,
              localization::secondsSinceLastVision);
      this.autoBindings.bind();
      // Calienta el pathfinder (LocalADStar) para que el primer HubAlign no tarde en planear.
      CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    } else {
      this.autoBindings = null;
    }

    OperatorBindings.create(operator, intake, shooter, indexer, dumper, intakFlywheels).bind();
  }

  @Override
  public void updateNodes() {}

  public Command getAutonomousCommand() {
    if (autoBindings == null) {
      return Commands.print("PathPlanner no esta configurado: no hay auto");
    }
    return autoBindings.get();
  }

  @Override
  public TestRoutine getTestRoutine() {
    return new EmptyTest();
  }
}