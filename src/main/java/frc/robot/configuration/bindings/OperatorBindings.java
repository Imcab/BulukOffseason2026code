// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.operator.ControllerOI;

import frc.robot.configuration.KeyManager;
import frc.robot.configuration.constants.modules.FlywheelsConstants.shooterWheelsConstants;
import frc.robot.configuration.constants.modules.IntakeConstants;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.individuals.intake.IntakeSpark.intakeMODE;
import frc.robot.requests.FlywheelsRequestFactory;
import frc.robot.requests.IntakeRequestFactory;

public class OperatorBindings implements Binding {

  private final ControllerOI operator;

  private final Intake intake;
  private final Flywheels shooter;

  // Valores de prueba del shooter editables en vivo desde el dashboard
  private static final String TEST_VOLTS_KEY = "Tuning/TestVolts";
  private static final String TEST_RPM_KEY = "Tuning/TestRPM";

  private OperatorBindings(ControllerOI operator, Intake intake, Flywheels shooter) {
    this.operator = operator;
    this.intake = intake;
    this.shooter = shooter;
  }

  public static OperatorBindings create(ControllerOI operator, Intake intake, Flywheels shooter) {
    return new OperatorBindings(operator, intake, shooter);
  }

  @Override
  public void bind() {
    var buttons = operator.getActionButtons();
    var bumpers = operator.getBumpers();
    var triggers = operator.getAnalogTriggers();

    // ----- Intake (mientras se mantenga presionado, al soltar regresa a idle) -----

    // A: bajar el intake
    buttons
        .bottom()
        .whileTrue(
            intake.setControl(
                () ->
                    IntakeRequestFactory.setAngle()
                        .withAngle(IntakeConstants.kDownAngle)
                        .Tolerance(IntakeConstants.kToleranceDegrees)
                        .withMode(intakeMODE.kDOWN)));

    // Y: subir el intake
    buttons
        .top()
        .whileTrue(
            intake.setControl(
                () ->
                    IntakeRequestFactory.setAngle()
                        .withAngle(IntakeConstants.kUpAngle)
                        .Tolerance(IntakeConstants.kToleranceDegrees)
                        .withMode(intakeMODE.kUP)));

    // X: resetear el encoder a 0 en la posicion actual
    buttons.left().onTrue(intake.seed());

    // B: voltaje positivo de prueba / RB: voltaje negativo de prueba
    buttons.right().whileTrue(intake.voltageCommand(IntakeConstants.kTestVolts));
    bumpers.right().whileTrue(intake.voltageCommand(-IntakeConstants.kTestVolts));

    // ----- Shooter (mientras se mantenga presionado, al soltar regresa a idle) -----

    // Publicar los valores iniciales una sola vez para poder editarlos desde el dashboard
    NetworkIO.set(KeyManager.SHOOTER_KEY, TEST_VOLTS_KEY, shooterWheelsConstants.kTestVolts);
    NetworkIO.set(KeyManager.SHOOTER_KEY, TEST_RPM_KEY, shooterWheelsConstants.kTestRPM);

    // LT: voltaje fijo (para medir kS y kV)
    triggers
        .left()
        .whileTrue(
            shooter.setControl(
                () ->
                    FlywheelsRequestFactory.moveVoltage()
                        .withVolts(
                            () ->
                                NetworkIO.get(
                                    KeyManager.SHOOTER_KEY,
                                    TEST_VOLTS_KEY,
                                    shooterWheelsConstants.kTestVolts))));

    // RT: control de velocidad en RPM (para afinar kP)
    triggers
        .right()
        .whileTrue(
            shooter.setControl(
                () ->
                    FlywheelsRequestFactory.setRPM()
                        .toRPM(
                            () ->
                                NetworkIO.get(
                                    KeyManager.SHOOTER_KEY,
                                    TEST_RPM_KEY,
                                    shooterWheelsConstants.kTestRPM))
                        .withTolerance(shooterWheelsConstants.kRPMTolerance)));
  }
}
