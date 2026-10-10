package frc.robot.modules.individuals.Dumper;


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

import frc.robot.configuration.constants.modules.DumperConstants;


public class DumperSpark implements DumperIO {

    private final SparkMax DumperAngulator;
    private final RelativeEncoder encoderDumper;
    private final SparkClosedLoopController controllerDumper;

    public DumperSpark(){
        DumperAngulator = new SparkMax(DumperConstants.DumperAngulator_ID, MotorType.kBrushless);
        encoderDumper = DumperAngulator.getEncoder();
        controllerDumper = DumperAngulator.getClosedLoopController();
        
        motorConfig();
    }

    public void motorConfig(){
        var config = new SparkMaxConfig();

        var profiles = config.closedLoop;

        DumperAngulator.setCANTimeout(250);

        try {

        profiles
            .pid(DumperConstants.kP_Up, DumperConstants.kI_Up, DumperConstants.kD_Up, ClosedLoopSlot.kSlot0)
            .pid(DumperConstants.kP_Down, DumperConstants.kI_Down, DumperConstants.kD_Down, ClosedLoopSlot.kSlot1);

        profiles.feedForward
            .kS(DumperConstants.kS_Up, ClosedLoopSlot.kSlot0)
            .kV(DumperConstants.kV_Up, ClosedLoopSlot.kSlot0)
            .kA(DumperConstants.kA_Up, ClosedLoopSlot.kSlot0);

        profiles.feedForward
            .kS(DumperConstants.kS_Down, ClosedLoopSlot.kSlot1)
            .kV(DumperConstants.kV_Down, ClosedLoopSlot.kSlot1)
            .kA(DumperConstants.kA_Down, ClosedLoopSlot.kSlot1);

        profiles.maxMotion
            .cruiseVelocity(DumperConstants.kCruiseVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(DumperConstants.kMaxAcc, ClosedLoopSlot.kSlot0)
            .allowedProfileError(DumperConstants.kAllowedErrorDegrees, ClosedLoopSlot.kSlot0)
            .cruiseVelocity(DumperConstants.kCruiseVelocity, ClosedLoopSlot.kSlot1)
            .maxAcceleration(DumperConstants.kMaxAcc, ClosedLoopSlot.kSlot1)
            .allowedProfileError(DumperConstants.kAllowedErrorDegrees, ClosedLoopSlot.kSlot1);

        config
            .idleMode(IdleMode.kBrake)
            .inverted(DumperConstants.kMotorInverted)
            .smartCurrentLimit(DumperConstants.kCurrentLimit)
            .voltageCompensation(DumperConstants.kMaxVolts);

        config
            .softLimit
            .forwardSoftLimit(DumperConstants.kForwardLimitDegrees)
            .forwardSoftLimitEnabled(DumperConstants.kSoftLimitsEnabled)
            .reverseSoftLimit(DumperConstants.kReverseLimitDegrees)
            .reverseSoftLimitEnabled(DumperConstants.kSoftLimitsEnabled);

        // Posicion en grados del mecanismo, velocidad en grados/s
        config
            .encoder
            .positionConversionFactor(DumperConstants.kPositionFactor)
            .velocityConversionFactor(DumperConstants.kVelocityFactor);

        DumperAngulator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder relativo: el dumper debe arrancar guardado (vertical) = 0
        encoderDumper.setPosition(0);

        } finally {
        DumperAngulator.setCANTimeout(0);
        }

    }

    public enum DumperMODE {
        kUP,
        kDOWN
    }

    @Override
    public void setPosition(double Angle, DumperMODE mode) {
        switch (mode) {
        case kUP:
            controllerDumper.setSetpoint(
                Angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                gravityFF(DumperConstants.kG_Up), ArbFFUnits.kVoltage);
            break;

        case kDOWN:
            controllerDumper.setSetpoint(
                Angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1,
                gravityFF(DumperConstants.kG_Down), ArbFFUnits.kVoltage);
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
            (encoderDumper.getPosition() - DumperConstants.kVerticalPosition)
                / (DumperConstants.kHorizontalPosition - DumperConstants.kVerticalPosition);
        double angleFromHorizontal = Math.toRadians(90.0 * (1.0 - fraction));
        return -kG * Math.cos(angleFromHorizontal);
    }

    @Override
    public void updateInputs(DumperInputs inputs) {

        inputs.position = encoderDumper.getPosition();
        inputs.velocity = encoderDumper.getVelocity();

        inputs.profileSetpoint = controllerDumper.getMAXMotionSetpointPosition();
        inputs.profileVelocity = controllerDumper.getMAXMotionSetpointVelocity();

        inputs.appliedVolts = DumperAngulator.getAppliedOutput() * DumperAngulator.getBusVoltage();

        inputs.current = DumperAngulator.getOutputCurrent();
    }

    @Override
    public void resetPosition() {
        encoderDumper.setPosition(0);
    }

    @Override
    public void applyOutput(double voltage) {
        DumperAngulator.setVoltage(voltage);
    }

    @Override
    public void stopAll() {
        DumperAngulator.stopMotor();
    }

    
}