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

        public PathPlannerAuto blueAuto131;
        public PathPlannerAuto blueAuto142;
        
        public PathPlannerAuto blueBackShootLeft;
        public PathPlannerAuto blueBackShootMiddle;
        public PathPlannerAuto blueBackShootRightM;
        public PathPlannerAuto blueBackShootRightR;
        
        public PathPlannerAuto blueShootPassLeft;
        public PathPlannerAuto blueShootMPassShortRight;
        public PathPlannerAuto blueShootRPassShortRight;
        
        public PathPlannerAuto blueVomitPassLongLeft;
        public PathPlannerAuto blueVomitPassShortLeft;
        public PathPlannerAuto blueVomitPassShortRight;
        
        // ----- Red -----
        public PathPlannerAuto redPosition1;
        public PathPlannerAuto redPosition2;
        public PathPlannerAuto redPosition31;
        public PathPlannerAuto redPosition32;
        public PathPlannerAuto redPosition33;
        public PathPlannerAuto redPosition34;
 
        public PathPlannerAuto redShootMPassShortLeft;
 
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

    blueAuto131 = new PathPlannerAuto("Blue_Auto 1.3.1");
    blueAuto142 = new PathPlannerAuto("Blue_Auto 1.4.2");
 
    blueBackShootLeft = new PathPlannerAuto("blue_back_shoot_left");
    blueBackShootMiddle = new PathPlannerAuto("blue_back_shoot_middle");
    blueBackShootRightM = new PathPlannerAuto("blue_back_shoot_right_m");
    blueBackShootRightR = new PathPlannerAuto("blue_back_shoot_right_r");
 
    blueShootPassLeft = new PathPlannerAuto("blue_shoot_pass_left");
    blueShootMPassShortRight = new PathPlannerAuto("blue_shootm_pass_short_right");
    // OJO: el archivo se llama "righbt" (typo). Si lo renombras en PathPlanner, cambialo aqui.
    blueShootRPassShortRight = new PathPlannerAuto("blue_shootr_pass_short_righbt");
 
    blueVomitPassLongLeft = new PathPlannerAuto("blue_vomit_pass_long_left");
    blueVomitPassShortLeft = new PathPlannerAuto("blue_vomit_pass_short_left");
    blueVomitPassShortRight = new PathPlannerAuto("blue_vomit_pass_short_right");
 
    redPosition1 = new PathPlannerAuto("red_position_1");
    redPosition2 = new PathPlannerAuto("red_position_2");
    redPosition31 = new PathPlannerAuto("red_position_3.1");
    redPosition32 = new PathPlannerAuto("red_position_3.2");
    redPosition33 = new PathPlannerAuto("red_position_3.3");
    redPosition34 = new PathPlannerAuto("red_position_3.4");
 
    redShootMPassShortLeft = new PathPlannerAuto("shootm_pass_short_left_red");
 
    // Opcion por defecto: no correr nada (evita que arranque un auto sin querer)
    chooser.setDefaultOption("Ninguno", Commands.none());
 
    chooser.addOption("Blue Auto 1.3.1", blueAuto131);
    chooser.addOption("Blue Auto 1.4.2", blueAuto142);
 
    chooser.addOption("Blue Back Shoot Left", blueBackShootLeft);
    chooser.addOption("Blue Back Shoot Middle", blueBackShootMiddle);
    chooser.addOption("Blue Back Shoot Right (M)", blueBackShootRightM);
    chooser.addOption("Blue Back Shoot Right (R)", blueBackShootRightR);
 
    chooser.addOption("Blue Shoot Pass Left", blueShootPassLeft);
    chooser.addOption("Blue ShootM Pass Short Right", blueShootMPassShortRight);
    chooser.addOption("Blue ShootR Pass Short Right", blueShootRPassShortRight);
 
    chooser.addOption("Blue Vomit Pass Long Left", blueVomitPassLongLeft);
    chooser.addOption("Blue Vomit Pass Short Left", blueVomitPassShortLeft);
    chooser.addOption("Blue Vomit Pass Short Right", blueVomitPassShortRight);
 
    chooser.addOption("Red Position 1", redPosition1);
    chooser.addOption("Red Position 2", redPosition2);
    chooser.addOption("Red Position 3.1", redPosition31);
    chooser.addOption("Red Position 3.2", redPosition32);
    chooser.addOption("Red Position 3.3", redPosition33);
    chooser.addOption("Red Position 3.4", redPosition34);
 
    chooser.addOption("Red ShootM Pass Short Left", redShootMPassShortLeft);

    

    SmartDashboard.putData("AutoSelector", chooser);

    }   

    public Command get() {
    return chooser.getSelected();
  }

}

    

