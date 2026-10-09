package frc.robot.configuration.constants.modules;

public class IntakeConstants {
        
    public static int intakeRatio = 1;
    public static int rollsSparkID = 0;
    public static int angulatorSparkID = 13;

    public static final double kGearRatio = 9;

    // ====================== ANGULADOR (brazo que baja a recoger) ======================
    // Convencion: 0 grados = brazo GUARDADO (arriba, donde enciende el robot).
    // Positivo = hacia el piso. Si tu motor gira al reves, cambia kAngulatorInverted.

    /** true = kP/kI/kD/kG/kS editables en SmartDashboard ("Intake/kP", ...). false en competencia. */
    public static final boolean kAngulatorTuningMode = true;

    /** CONFIRMAR: vueltas del motor por 1 vuelta del brazo. */
    public static final double kAngulatorReduction = kGearRatio;
    public static final double kDegreesPerMotorRotation = 360.0 / kAngulatorReduction;
    public static final boolean kAngulatorInverted = false;

    public static final double kStowAngleDeg = 0.0;
    /** MEDIR: grados desde guardado hasta tocar el piso / posicion de recoleccion. */
    public static final double kDeployAngleDeg = 130.0;

    public static final double kSoftLimitMarginDeg = 5.0;

    // Valores de PARTIDA, no calibrados. kP = duty cycle por grado de error.
    public static final double kAngulatorP = 0.005;
    public static final double kAngulatorI = 0.0;
    public static final double kAngulatorD = 0.0;
    /** Feedforward en volts: kG*cos(angulo + offset) + kS*signo(error). kG puede salir NEGATIVO. */
    public static final double kAngulatorG = 0.0;
    public static final double kAngulatorS = 0.0;
    /** Guardado vertical -> -90 (la gravedad crece con sin(angulo)); guardado horizontal -> 0. */
    public static final double kAngulatorCosOffsetDeg = 0.0;

    public static final double kAngulatorMinOutput = -0.5;
    public static final double kAngulatorMaxOutput = 0.5;
    public static final int kAngulatorCurrentLimit = 30;
    public static final double kMaxVolts = 12.0;

    //sim
    public static double MOIRolls = 0.0; // I =1/3 mL^2
    public static double rollsGearing = 0.0;
    public static int numRollsMotors = 0;

    public static double MOIAngulator = 0.0; // I =1/3 mL^2
    public static double angulatorGearing = 0.0;
    public static int numAngulatorMotors = 0;
    public static double armLength = 0.0; //metros
    public static double minAngle = 0.0;
    public static double maxAngle = 0.0;

    //SJA
    public static double maxVelocity = 0.0; 
    public static double maxAcceleration = 0.0;

    //request
    public static double toleranceDegrees = 3.0; //tolerancia para angulador (grados)

    //pid
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kMinOutput = 0.0;
    public static double kMaxOutput = 0.0;  

}
