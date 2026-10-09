package frc.robot.configuration.constants.modules;

//This is a constant file. Only change the values of the constants if you know what you are doing.
public class IndexerConstants {

    public static final int rollerID = 12;
    public static final int indexID = 15;
    public static double rollerRatio= 3;
    public static double indexRatio = 3;
    public static double kIndexMOI = 1;
    public static double kRollerMOI = 1;

    public static final boolean kRollerMotorInverted = false;
    public static final boolean kRollerEncoderInverted = false;

    public static final boolean kIndexInverted = false;
    public static final boolean kIndexEncoderInverted = false;

    // ---------------- ROLLERS EN RPM (disparo) ----------------
    /** RPM del motor de los rollers al disparar. Negativo = alimenta (igual que ProcessSpeed(-x)). */
    public static final double kRollerFeedRPM = -3000;

    // PID del SparkMax (slot 0). kP en salida por RPM de error; empieza bajo y sube si tarda en llegar.
    public static final double kRollerP = 0.0002;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;
    // Feedforward en VOLTS (se manda como arbFeedforward): kV = 12 V / 5676 RPM (NEO libre) = 0.0021.
    // A 3000 RPM: 0.1 + 3000*0.0021 = 6.4 V.
    public static final double kRollerS = 0.1;
    public static final double kRollerV = 0.0021;

    // ---------------- INDEX EN RPM (prueba del indexer) ----------------
    /** RPM del motor del index en la prueba. Negativo = alimenta (igual que ProcessSpeed(-x)). */
    public static final double kIndexFeedRPM = -4000;
    public static final double kIndexP = 0.0002;
    public static final double kIndexI = 0;
    public static final double kIndexD = 0;
    // FF en volts, mismo NEO: kV = 12 / 5676 = 0.0021 (3000 RPM ~ 6.4 V)
    public static final double kIndexS = 0.1;
    public static final double kIndexV = 0.0021;

    // ---------------- ROLLERS + INDEX POR CORRIENTE (prueba) ----------------
    /** Amperes de la prueba. Negativo = alimenta (mismo sentido que ProcessSpeed(-x)). Empezar en 40 A. */
    public static final double kRollerFeedAmps = -40;
    public static final double kIndexFeedAmps = -40;
    // Ganancias del lazo de corriente del SparkMax (slot 1, ControlType.kCurrent). Valores de PARTIDA:
    // el lazo de corriente casi no esta documentado por REV, hay que afinarlo en el robot.
    public static final double kCurrentP = 0.05;
    public static final double kCurrentI = 0.0;
    public static final double kCurrentD = 0.0;

    public static final int SmartCurrentLimit = 80;
    public static final double VoltageCompesation = 12;
    
}