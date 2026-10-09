package frc.robot.configuration.constants.modules;

public class IntakeConstants {

  public static final int Angulator_MOTOR_CAN_ID = 13;

  // Solo para simulacion (SingleJointedArmSim)
  public static final double kGearRatio = 20.0;
  public static final double kIntakeLengthMeters = 0.4;
  public static final double kIntakeMassKg = 6.7;

  // ---------------- Motor (NEO + SparkMax) ----------------
  public static final boolean kMotorInverted = false; // Equivale a CounterClockwise_Positive del Kraken, verificar en robot
  public static final int kCurrentLimit = 40;
  public static final double kMaxVolts = 12.0;

  // ---------------- Conversion ----------------
  // Misma reduccion que el Kraken (SensorToMechanismRatio = 36)
  public static final double kMotorToMechanismRatio = 36.0;
  // El modulo completo trabaja en grados del mecanismo
  public static final double kPositionFactor = 360.0 / kMotorToMechanismRatio; // grados por vuelta de motor
  public static final double kVelocityFactor = kPositionFactor / 60.0; // RPM -> grados/s

  // ---------------- Setpoints (grados, rango del intake: 0 a 40) ----------------
  public static final double kUpAngle = 0;
  public static final double kDownAngle = 35;
  public static final double kToleranceDegrees = 3.0;

  // Voltaje de prueba para configurar (B = positivo, RB = negativo). Empezar bajo y subir poco a poco
  public static final double kTestVolts = 1;

  // ---------------- Soft limits (grados) ----------------
  // Activar en cuanto se confirme la direccion del motor (B debe hacer que la posicion suba).
  // Forward debe ser el mayor.
  public static final boolean kSoftLimitsEnabled = false;
  public static final double kForwardLimitDegrees = 40;
  public static final double kReverseLimitDegrees = 0;

  // ---------------- MAXMotion (grados/s, grados/s^2) ----------------
  // NEO libre ~5676 RPM / 36 ~= 945 grados/s en el mecanismo.
  // Recorrido de 40 grados: empezar lento (~0.5 s) y subir cuando ya este afinado
  public static final double kCruiseVelocity = 120;
  public static final double kMaxAcc = 240;
  public static final double kAllowedErrorDegrees = 1.0;

  // ---------------- Slot 0 (UP) ----------------
  // Convertido desde el Kraken: PID en V/rot -> duty cycle/grado ( / 12 / 360 ), FF en volts
  public static final double kP_Up = 0; // ~0.0042
  public static final double kI_Up = 0;
  public static final double kD_Up = 0; // Re-tunear, unidades de REV distintas a Phoenix

  public static final double kS_Up = 3.5; // V
  public static final double kV_Up = 2; // V por grado/s
  public static final double kA_Up = 1.0 / 360.0; // V por grado/s^2
  public static final double kG_Up = 1; // V para sostener el intake horizontal (kG del Kraken)

  // ---------------- Slot 1 (DOWN) ----------------
  public static final double kP_Down = 15.0 / 12.0 / 360.0; // ~0.0035
  public static final double kI_Down = 0; // Kraken tenia 0.05, agregar solo si hay error estable
  public static final double kD_Down = 0;

  public static final double kS_Down = 0.9;
  public static final double kV_Down = 4.1 / 360.0;
  public static final double kA_Down = 0;
  public static final double kG_Down = 1; // El Kraken no tenia kG en este slot

  // ---------------- Gravedad ----------------
  // Lecturas del encoder en las dos posiciones de referencia. La gravedad se calcula en codigo
  // (el kCos de REV asume 0 = horizontal y no acepta offset).
  public static final double kVerticalPosition = 0; // guardado, sin torque de gravedad
  public static final double kHorizontalPosition = 40; // acostado al piso, torque maximo
}
