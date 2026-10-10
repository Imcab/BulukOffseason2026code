// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.KeyManager;
import frc.robot.configuration.constants.modules.FlywheelsConstants.shooterWheelsConstants;
import frc.robot.configuration.constants.modules.DumperConstants;
import frc.robot.configuration.constants.modules.IndexerConstants;
import frc.robot.configuration.constants.modules.IntakeConstants;
import frc.robot.modules.individuals.Dumper.Dumper;
import frc.robot.modules.individuals.Dumper.DumperSpark.DumperMODE;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.indexer.Indexer;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.individuals.intake.IntakeSpark.intakeMODE;
import frc.robot.requests.DumperRequest;
import frc.robot.requests.DumperRequestFactory;
import frc.robot.requests.FlywheelsRequestFactory;
import frc.robot.requests.IndexerRequest;
import frc.robot.requests.IndexerRequestFactory;
import frc.robot.requests.IntakeRequestFactory;
import frc.robot.requests.DumperRequest.setAngle;


public class OperatorBindings implements Binding {

  private final ControllerOI operator;

  private final Intake intake;
  private final Flywheels shooter;
  private final Indexer indexer;
  private final Dumper dumper;
private final double DEADBAND = 0.1;


  // Valores de prueba del shooter editables en vivo desde el dashboard
  private static final String TEST_VOLTS_KEY = "Tuning/TestVolts";
  private static final String TEST_RPM_KEY = "Tuning/TestRPM";

  private OperatorBindings(
      ControllerOI operator, Intake intake, Flywheels shooter, Indexer indexer, Dumper dumper) {
    this.operator = operator;
    this.intake = intake;
    this.shooter = shooter;
    this.indexer = indexer;
    this.dumper = dumper;
  }

  public static OperatorBindings create(
      ControllerOI operator, Intake intake, Flywheels shooter, Indexer indexer, Dumper dumper) {
    return new OperatorBindings(operator, intake, shooter, indexer, dumper);
  }

  @Override
  public void bind() {
    var buttons = operator.getActionButtons();
    var bumpers = operator.getBumpers();
    var triggers = operator.getAnalogTriggers();
    var dpad = operator.getDPadTriggers();

    var leftStick = operator.getLeftStick();
    var rightStick = operator.getRightStick();
    var pov = operator.getDPadTriggers();

    Trigger rightStickXTrigger =
        new Trigger(() -> Math.abs(rightStick.x().getAsDouble()) > DEADBAND);
    Trigger rightStickYTrigger =
        new Trigger(() -> Math.abs(rightStick.y().getAsDouble()) > DEADBAND);
    Trigger leftStickXTrigger = new Trigger(() -> Math.abs(leftStick.x().getAsDouble()) > DEADBAND);
    Trigger leftStickYTrigger = new Trigger(() -> Math.abs(leftStick.y().getAsDouble()) > DEADBAND);

    // ----- Intake (mientras se mantenga presionado, al soltar regresa a idle) -----



    // A: bajar el intake
    buttons
        .bottom()
        .whileTrue(
            intake.setControl(
                () ->
                    IntakeRequestFactory.setAngle()
                        .withAngle(-138 )
                        .Tolerance(IntakeConstants.kToleranceDegrees)
                        .withMode(intakeMODE.kDOWN)));

    // X: resetear el encoder a 0 en la posicion actual
    buttons.left().onTrue(intake.seed());


    // Y: subir el intake
    buttons
        .top()
        .whileTrue(
            intake.setControl(
                () ->
                    IntakeRequestFactory.setAngle()
                        .withAngle(0)
                        .Tolerance(IntakeConstants.kToleranceDegrees)
                        .withMode(intakeMODE.kUP)));


    // B: voltaje positivo de prueba / RB: voltaje negativo de prueba
    buttons.right().whileTrue(dumper.voltageCommand(1));
    //bumpers.right().whileTrue(intake.voltageCommand(-IntakeConstants.kTestVolts));

    // ----- Indexer (mientras se mantenga presionado, al soltar regresa a idle) -----

    // LB: index a voltaje fijo
    // bumpers
    //     .left()
    //     .whileTrue(
    //         indexer.setControl(
    //             () -> IndexerRequestFactory.processing().withIndex(-12).withRollers(-12)));

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

    
  
    dpad.right().whileTrue(
        dumper.setControl(
            () -> DumperRequestFactory.moveVoltage().withVolts(8)
        )
    );
    // ----- Disparo -----

    dpad.down().whileTrue(new DumperTestCommand(shooter.getActor().getMotor(), shooter));

    // D-pad arriba: shooter a kShootRPM; cuando llega, el indexer libera las piezas.
    // Una vez que empieza a liberar sigue alimentando aunque las RPM bajen por el disparo.
    // Al soltar, ambos regresan a idle.
    dpad.up()
        .whileTrue(
            Commands.parallel(
    shooter.setControl(() ->
        FlywheelsRequestFactory.setRPM()
            .toRPM(shooterWheelsConstants.kShootRPM)
            .withTolerance(shooterWheelsConstants.kRPMTolerance)),
    Commands.repeatingSequence(
        // 1) espera a estar en velocidad 0.3 s seguidos
        Commands.waitUntil(
            new Trigger(() -> shooter.isAtRPM(
                shooterWheelsConstants.kShootRPM,
                shooterWheelsConstants.kRPMTolerance)).debounce(0.3)),
        // 2) alimenta solo mientras las RPM no caigan más de 400 RPM
        indexer.setControl(() ->
                IndexerRequestFactory.processing()
                    .withRollers(IndexerConstants.kShootRollerVolts)
                    .withIndex(IndexerConstants.kShootIndexVolts))
            .onlyWhile(() -> shooter.isAtRPM(
                shooterWheelsConstants.kShootRPM, 400)))));

        //MOVER ROLLERS DUMPER INDIVIDUALES
        bumpers.right().whileTrue(
            Commands.parallel(
                indexer.setControl(
                    () -> IndexerRequestFactory.setIndex().withRPM(-4500)
                )
            
            )
        );
        
        bumpers.left().whileTrue(
            Commands.parallel(
                indexer.setControl(
                    () -> IndexerRequestFactory.setRollers().withRPM(-4500)
                )
            )
        );

  }



}