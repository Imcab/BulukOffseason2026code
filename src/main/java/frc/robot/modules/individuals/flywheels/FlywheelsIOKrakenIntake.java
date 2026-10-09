package frc.robot.modules.individuals.flywheels;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.configuration.constants.modules.FlywheelsConstants.IntakeWheelsConstants;

public class FlywheelsIOKrakenIntake implements FlywheelsIO {

    private final TalonFX intakeFlyWheels;
    private final TalonFXConfigurator FlyWheelsConfigurator;

    public FlywheelsIOKrakenIntake() {
        intakeFlyWheels = new TalonFX(IntakeWheelsConstants.IntakeWheels_ID, CANBus.roboRIO());
        FlyWheelsConfigurator = intakeFlyWheels.getConfigurator();

        configMotor();
    }

    public void configMotor() {
        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = IntakeWheelsConstants.invertedValue;
        motorConfigs.NeutralMode = NeutralModeValue.Coast;

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimitEnable = IntakeWheelsConstants.StatorCurrentLimitEnable;
        limitConfigs.StatorCurrentLimit = IntakeWheelsConstants.StatorCurrentLimit;

        limitConfigs.SupplyCurrentLimitEnable = IntakeWheelsConstants.SupplyCurrentLimitEnable;
        limitConfigs.SupplyCurrentLimit = IntakeWheelsConstants.SupplyCurrentLimit;

        intakeFlyWheels.getConfigurator().apply(limitConfigs);

        FlyWheelsConfigurator.apply(motorConfigs);
    }

    @Override
    public void applyOutput(double volts) {
        intakeFlyWheels.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed) {
        intakeFlyWheels.set(speed);
    }

    @Override
    public void updateInputs(FlyWheelsInputs inputs) {
        inputs.appliedVolts = intakeFlyWheels.getMotorVoltage().getValueAsDouble();
        inputs.velocityRPM = intakeFlyWheels.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTargetRPM(double rpm) {
        // nothing
    }

}