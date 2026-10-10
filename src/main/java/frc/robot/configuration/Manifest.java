package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Environment.RunMode;
import com.stzteam.mars.builder.Injector;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.operator.PS5OI;
import com.stzteam.mars.operator.XboxOI;

import frc.robot.configuration.constants.swerve.TunerConstants;
import frc.robot.modules.individuals.Dumper.Dumper;
import frc.robot.modules.individuals.Dumper.DumperIO;
import frc.robot.modules.individuals.Dumper.DumperIOFallback;
import frc.robot.modules.individuals.Dumper.DumperSpark;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.flywheels.FlywheelsIO;
import frc.robot.modules.individuals.flywheels.FlywheelsIOFallback;
import frc.robot.modules.individuals.flywheels.FlywheelsIOKrakenShooter;
import frc.robot.modules.individuals.indexer.Indexer;
import frc.robot.modules.individuals.indexer.IndexerIO;
import frc.robot.modules.individuals.indexer.IndexerIOFallback;
import frc.robot.modules.individuals.indexer.IndexerIOSpark;
import frc.robot.modules.individuals.intake.Intake;
import frc.robot.modules.individuals.intake.IntakeIO;
import frc.robot.modules.individuals.intake.IntakeIOFallback;
import frc.robot.modules.individuals.intake.IntakeSpark;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveTelemetry;

public class Manifest {

    public static final RunMode CURRENT_MODE = RunMode.REAL;

    static{Environment.setMode(CURRENT_MODE);}

    public enum ControllerType {
        PS5,
        XBOX
    }

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    public static final ControllerType DRIVER_CONTROLLER = ControllerType.XBOX;
    public static final ControllerType OPERATOR_CONTROLLER = ControllerType.XBOX;

    public static final boolean HAS_DRIVETRAIN = true;
    public static final boolean HAS_INDEXER = true;
    public static final boolean HAS_DUMPER = true;
    public static final boolean HAS_SHOOTER_WHELLS = true;
    public static final boolean HAS_INTAKE_FLYWHELLS = true;
    public static final boolean HAS_INTAKE_WHEELS = true;
    public static final boolean HAS_INTAKE = true;

    /* CTRE SignalLogger (hoot logs on the USB drive). Must be on to record SysId data */
    public static final boolean ENABLE_SIGNAL_LOGGER = false;

    public static class ControlsBuilder {
    
        public static ControllerOI buildDriver() {
          return DRIVER_CONTROLLER == ControllerType.PS5
              ? new PS5OI(DRIVER_PORT)
              : new XboxOI(DRIVER_PORT);
        }
    
        public static ControllerOI buildOperator() {
          return OPERATOR_CONTROLLER == ControllerType.PS5
              ? new PS5OI(OPERATOR_PORT)
              : new XboxOI(OPERATOR_PORT);
        }
    }

    public static class DrivetrainBuilder {
    
        public static CommandSwerveDrivetrain buildModule() {
            if (!HAS_DRIVETRAIN) return null;
    
            CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
            SwerveTelemetry telemetry = new SwerveTelemetry(ENABLE_SIGNAL_LOGGER);
            drivetrain.registerTelemetry(telemetry::telemeterize);
    
            return drivetrain;
            }
    }

    public static Intake buildIntake() {
        // Sin IO de simulacion por ahora: en SIM usa el fallback
        IntakeIO io =
            Injector.createIO(HAS_INTAKE, IntakeIOFallback::new, IntakeSpark::new, IntakeIOFallback::new);
        return new Intake(io);
    }

    public static Flywheels buildShooter() {
        // Sin IO de simulacion por ahora: en SIM usa el fallback
        FlywheelsIO io =
            Injector.createIO(
                HAS_SHOOTER_WHELLS,
                FlywheelsIOFallback::new,
                FlywheelsIOKrakenShooter::new,
                FlywheelsIOFallback::new);
        return new Flywheels(io, KeyManager.SHOOTER_KEY);
    }

    public static Indexer buildIndexer() {
        // Sin IO de simulacion por ahora: en SIM usa el fallback
        IndexerIO io =
            Injector.createIO(
                HAS_INDEXER,
                IndexerIOFallback::new,
                IndexerIOSpark::new,
                IndexerIOFallback::new);
        return new Indexer(io);
    }

    public static Dumper builDumper(){
        DumperIO io =
            Injector.createIO(
                HAS_DUMPER,
                DumperIOFallback::new, 
                DumperSpark::new, 
                DumperIOFallback::new);

        return new Dumper(io);

    }

    

}
