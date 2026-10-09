// package frc.robot.modules.individuals.intake;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.configuration.constants.modules.IntakeConstants;

// public class IntakeSpark implements IntakeIO{

//     private final SparkMax rollsSpark;
//     private final SparkMax angulatorSpark;
//     private final RelativeEncoder angulatorEncoder;
//     private final SparkClosedLoopController rollsController;
//     private final SparkClosedLoopController angulatorController;

//     // Valores "vivos": arrancan con las constantes y, en modo tuning, se editan desde el Dashboard.
//     private double kP = IntakeConstants.kAngulatorP;
//     private double kI = IntakeConstants.kAngulatorI;
//     private double kD = IntakeConstants.kAngulatorD;
//     private double kG = IntakeConstants.kAngulatorG;
//     private double kS = IntakeConstants.kAngulatorS;

//     public IntakeSpark() {

//         this.rollsSpark = new SparkMax (IntakeConstants.rollsSparkID, MotorType.kBrushless);
//         this.angulatorSpark = new SparkMax (IntakeConstants.angulatorSparkID, MotorType.kBrushless);

//         this.angulatorEncoder = angulatorSpark.getEncoder();

//         this.rollsController = rollsSpark.getClosedLoopController();
//         this.angulatorController = angulatorSpark.getClosedLoopController();

//         var angulatorConfig = new SparkMaxConfig();

//         angulatorConfig.closedLoop
//             .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
//             .outputRange(IntakeConstants.kAngulatorMinOutput, IntakeConstants.kAngulatorMaxOutput, ClosedLoopSlot.kSlot0);

//         angulatorConfig
//             .idleMode(IdleMode.kBrake)
//             .inverted(IntakeConstants.kAngulatorInverted)
//             .smartCurrentLimit(IntakeConstants.kAngulatorCurrentLimit)
//             .voltageCompensation(IntakeConstants.kMaxVolts);

//         angulatorConfig.softLimit
//             .reverseSoftLimit(IntakeConstants.kStowAngleDeg - IntakeConstants.kSoftLimitMarginDeg)
//             .reverseSoftLimitEnabled(true)
//             .forwardSoftLimit(IntakeConstants.kDeployAngleDeg + IntakeConstants.kSoftLimitMarginDeg)
//             .forwardSoftLimitEnabled(true);

//         angulatorConfig.encoder
//             .positionConversionFactor(IntakeConstants.kDegreesPerMotorRotation)
//             .velocityConversionFactor(IntakeConstants.kDegreesPerMotorRotation / 60.0);

//         var rollsConfig = new SparkMaxConfig();
//         rollsConfig.encoder.velocityConversionFactor(1.0/IntakeConstants.intakeRatio);//RPM

//         angulatorSpark.configure(angulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         rollsSpark.configure(rollsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         if (IntakeConstants.kAngulatorTuningMode) {
//             SmartDashboard.putNumber("Intake/kP", kP);
//             SmartDashboard.putNumber("Intake/kI", kI);
//             SmartDashboard.putNumber("Intake/kD", kD);
//             SmartDashboard.putNumber("Intake/kG", kG);
//             SmartDashboard.putNumber("Intake/kS", kS);
//         }
//     }

//     private void pollTuning() {
//         if (!IntakeConstants.kAngulatorTuningMode) return;

//         double p = SmartDashboard.getNumber("Intake/kP", kP);
//         double i = SmartDashboard.getNumber("Intake/kI", kI);
//         double d = SmartDashboard.getNumber("Intake/kD", kD);
//         kG = SmartDashboard.getNumber("Intake/kG", kG);
//         kS = SmartDashboard.getNumber("Intake/kS", kS);

//         if (p != kP || i != kI || d != kD) {
//             kP = p; kI = i; kD = d;
//             var cfg = new SparkMaxConfig();
//             cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
//             angulatorSpark.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//         }
//     }

//     private void commandPosition(double targetDeg) {
//         double target = MathUtil.clamp(
//             targetDeg,
//             IntakeConstants.kStowAngleDeg - IntakeConstants.kSoftLimitMarginDeg,
//             IntakeConstants.kDeployAngleDeg + IntakeConstants.kSoftLimitMarginDeg);

//         double error = target - angulatorEncoder.getPosition();
//         double ffVolts =
//             kG * Math.cos(Math.toRadians(target + IntakeConstants.kAngulatorCosOffsetDeg))
//             + kS * Math.signum(error);

//         angulatorController.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
//     }

//     @Override
//     public void updateInputs(IntakeInputs inputs){
//         pollTuning();

//         inputs.rollsAppliedVolts = rollsSpark.getAppliedOutput() * rollsSpark.getBusVoltage();
//         inputs.rollsRPS = rollsSpark.getEncoder().getVelocity();

//         inputs.angulatorAppliedVolts = angulatorSpark.getAppliedOutput() * angulatorSpark.getBusVoltage();
//         inputs.angulatorTargetAngle = angulatorController.getSetpoint();
//         inputs.angulatorPosition = getAngulatorPosition();
//     }

//     @Override
//     public void setRollsVoltage(double volts) {
//         rollsSpark.setVoltage(volts);
//     }

//     @Override
//     public void setRollsRPS(double rps) {
//         rollsController.setSetpoint(rps, ControlType.kVelocity); //kVelocity para interpretar como obj de velocidad, no voltaje
//     }

//     @Override
//     public void stopRolls() {
//         rollsSpark.set(0);
//     }

//     @Override
//     public void setAngulatorPosition(double position, IntakeMODE mode) {
//         commandPosition(position); // el modo no cambia ganancias todavia (un solo slot)
//     }

//     @Override
//     public void setAngulatorVoltage(double volts) {
//         angulatorSpark.setVoltage(volts);
//     }

//     @Override
//     public double getAngulatorPosition() {
//         return angulatorEncoder.getPosition();
//     }

//     @Override
//     public void resetAngulator() {
//         angulatorEncoder.setPosition(IntakeConstants.kStowAngleDeg);
//     }

//     @Override
//     public void stopAngulator() {
//         // Mantiene la posicion actual (con feedforward para que la gravedad no la vaya bajando).
//         commandPosition(angulatorEncoder.getPosition());
//     }

//     @Override
//     public void stopAll() {
//         stopRolls();
//         stopAngulator();
//     }
// }