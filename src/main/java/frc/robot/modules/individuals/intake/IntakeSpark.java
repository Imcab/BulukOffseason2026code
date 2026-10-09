package frc.robot.modules.individuals.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.configuration.constants.modules.IntakeConstants;

public class IntakeSpark implements IntakeIO {

    private final SparkMax angulator;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    public IntakeSpark() {
        angulator = new SparkMax(IntakeConstants.Angulator_MOTOR_CAN_ID, MotorType.kBrushless);
        m_encoder = angulator.getEncoder();
        m_controller = angulator.getClosedLoopController();

        motorConfig();
    }

    public void motorConfig() {

        var config = new SparkMaxConfig();

        var profiles = config.closedLoop;

        angulator.setCANTimeout(250);

        try {

        profiles
            .pid(IntakeConstants.kP_Up, IntakeConstants.kI_Up, IntakeConstants.kD_Up, ClosedLoopSlot.kSlot0)
            .pid(IntakeConstants.kP_Down, IntakeConstants.kI_Down, IntakeConstants.kD_Down, ClosedLoopSlot.kSlot1);

        profiles.feedForward
            .kS(IntakeConstants.kS_Up, ClosedLoopSlot.kSlot0)
            .kV(IntakeConstants.kV_Up, ClosedLoopSlot.kSlot0)
            .kA(IntakeConstants.kA_Up, ClosedLoopSlot.kSlot0);

        profiles.feedForward
            .kS(IntakeConstants.kS_Down, ClosedLoopSlot.kSlot1)
            .kV(IntakeConstants.kV_Down, ClosedLoopSlot.kSlot1)
            .kA(IntakeConstants.kA_Down, ClosedLoopSlot.kSlot1);

        profiles.maxMotion
            .cruiseVelocity(IntakeConstants.kCruiseVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(IntakeConstants.kMaxAcc, ClosedLoopSlot.kSlot0)
            .allowedProfileError(IntakeConstants.kAllowedErrorDegrees, ClosedLoopSlot.kSlot0)
            .cruiseVelocity(IntakeConstants.kCruiseVelocity, ClosedLoopSlot.kSlot1)
            .maxAcceleration(IntakeConstants.kMaxAcc, ClosedLoopSlot.kSlot1)
            .allowedProfileError(IntakeConstants.kAllowedErrorDegrees, ClosedLoopSlot.kSlot1);

        config
            .idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.kMotorInverted)
            .smartCurrentLimit(IntakeConstants.kCurrentLimit)
            .voltageCompensation(IntakeConstants.kMaxVolts);

        config
            .softLimit
            .forwardSoftLimit(IntakeConstants.kForwardLimitDegrees)
            .forwardSoftLimitEnabled(IntakeConstants.kSoftLimitsEnabled)
            .reverseSoftLimit(IntakeConstants.kReverseLimitDegrees)
            .reverseSoftLimitEnabled(IntakeConstants.kSoftLimitsEnabled);

        // Posicion en grados del mecanismo, velocidad en grados/s
        config
            .encoder
            .positionConversionFactor(IntakeConstants.kPositionFactor)
            .velocityConversionFactor(IntakeConstants.kVelocityFactor);

        angulator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder relativo: el intake debe arrancar guardado (vertical) = 0
        m_encoder.setPosition(0);

        } finally {
        angulator.setCANTimeout(0);
        }
    }

    public enum intakeMODE {
        kUP,
        kDOWN
    }

    @Override
    public void setPosition(double angle, intakeMODE mode) {
        switch (mode) {
        case kUP:
            m_controller.setSetpoint(
                angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                gravityFF(IntakeConstants.kG_Up), ArbFFUnits.kVoltage);
            break;

        case kDOWN:
            m_controller.setSetpoint(
                angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1,
                gravityFF(IntakeConstants.kG_Down), ArbFFUnits.kVoltage);
            break;
        }
    }

    /**
     * Volts para compensar la gravedad en la posicion actual. Mapea la lectura del encoder a
     * angulo real contra el piso: kVerticalPosition = 90 grados, kHorizontalPosition = 0 grados.
     * Negativo porque la gravedad empuja hacia el piso (posicion positiva) y hay que sostenerlo.
     */
    private double gravityFF(double kG) {
        double fraction =
            (m_encoder.getPosition() - IntakeConstants.kVerticalPosition)
                / (IntakeConstants.kHorizontalPosition - IntakeConstants.kVerticalPosition);
        double angleFromHorizontal = Math.toRadians(90.0 * (1.0 - fraction));
        return -kG * Math.cos(angleFromHorizontal);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {

        inputs.position = m_encoder.getPosition();
        inputs.velocity = m_encoder.getVelocity();

        inputs.profileSetpoint = m_controller.getMAXMotionSetpointPosition();
        inputs.profileVelocity = m_controller.getMAXMotionSetpointVelocity();

        inputs.appliedVolts = angulator.getAppliedOutput() * angulator.getBusVoltage();

        inputs.current = angulator.getOutputCurrent();
    }

    @Override
    public void resetPosition() {
        m_encoder.setPosition(0);
    }

    @Override
    public void applyOutput(double voltage) {
        angulator.setVoltage(voltage);
    }

    @Override
    public void stopAll() {
        angulator.stopMotor();
    }
}
