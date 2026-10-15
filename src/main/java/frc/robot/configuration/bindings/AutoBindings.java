package frc.robot.configuration.bindings;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stzteam.mars.models.containers.Binding;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration.constants.modules.FlywheelsConstants.shooterWheelsConstants;
import frc.robot.configuration.constants.modules.IndexerConstants;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.indexer.Indexer;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.individuals.intake.IntakeSpark.intakeMODE;
import frc.robot.requests.FlywheelsRequestFactory;
import frc.robot.requests.IndexerRequestFactory;
import frc.robot.requests.IntakeRequestFactory;

public class AutoBindings implements Binding {

    public SendableChooser<Command> chooser = new SendableChooser<>();
    public PathPlannerAuto bumpBlueR;
    public PathPlannerAuto bumpBlueL;

    public PathPlannerAuto back_shoot_right_r;
    public PathPlannerAuto back_shoot_left;

    public PathPlannerAuto back_shoot_middle;
    public PathPlannerAuto back_shoot_right_m;

    public PathPlannerAuto shootm_pass_short_right;
    public PathPlannerAuto shootr_pass_short_right;

    public PathPlannerAuto shoot_pass_left;

    public PathPlannerAuto vomit_pass_long_left;

    public PathPlannerAuto vomit_pass_short_left;

    public PathPlannerAuto vomit_pass_short_right;

    private final Intake intake;
    private final Flywheels shooter;
    private final Flywheels intakFlywheels;
    private final Indexer indexer;
 
    private AutoBindings(Intake intake, Flywheels shooter, Indexer indexer, Flywheels intakFlywheels) {
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intakFlywheels = intakFlywheels;
  }

   public static AutoBindings create(
      Intake intake, Flywheels shooter, Indexer indexer, Flywheels intakFlywheels) {
        return new AutoBindings(intake, shooter, indexer, intakFlywheels);
  }

     private void registerAutoCommands() {
        // A del operator: bajar el intake
    NamedCommands.registerCommand(
        "Intake_down",
        intake
            .setControl(
                () ->
                    IntakeRequestFactory.setAngle().withAngle(60).withMode(intakeMODE.kDOWN))
            .withTimeout(1.2));
 
    // LT del operator: ruedas del intake (comer)
    NamedCommands.registerCommand(
        "Intake_eat",
        intakFlywheels
            .setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(-5000))
            .withTimeout(4));
 
    // No existe en el operator: es el LT al reves (escupir)
    NamedCommands.registerCommand(
        "Intake_vomit",
        intakFlywheels
            .setControl(() -> FlywheelsRequestFactory.setRPM().toRPM(5000))
            .withTimeout(1));
 
    // RT del operator: shooter a kShootRPM; cuando llega, el indexer libera las piezas.
    // Una vez que empieza a liberar sigue alimentando aunque las RPM bajen por el disparo.
    NamedCommands.registerCommand(
        "Flywheels_shoot",
        Commands.parallel(
                shooter.setControl(
                    () ->
                        FlywheelsRequestFactory.setRPM()
                            .toRPM(shooterWheelsConstants.kShootRPM)
                            .withTolerance(shooterWheelsConstants.kRPMTolerance)),
                Commands.repeatingSequence(
                    // 1) espera a estar en velocidad 0.3 s seguidos
                    Commands.waitUntil(
                        new Trigger(
                                () ->
                                    shooter.isAtRPM(
                                        shooterWheelsConstants.kShootRPM,
                                        shooterWheelsConstants.kRPMTolerance))
                            .debounce(0.3)),
                    // 2) alimenta solo mientras las RPM no caigan mas de 400 RPM
                    indexer
                        .setControl(
                            () ->
                                IndexerRequestFactory.processing()
                                    .withRollers(IndexerConstants.kShootRollerVolts)
                                    .withIndex(IndexerConstants.kShootIndexVolts))
                        .onlyWhile(
                            () -> shooter.isAtRPM(shooterWheelsConstants.kShootRPM, 400))))
            .withTimeout(4));
  }
     


    @Override
    public void bind() {
    registerAutoCommands();

    

    SmartDashboard.putData("AutoSelector", chooser);

    }   

    public Command get() {
    return chooser.getSelected();
  }

}

    

