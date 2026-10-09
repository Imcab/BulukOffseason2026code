package frc.robot.modules.individuals.flywheels;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.configuration.constants.modules.FlywheelsConstants.shooterWheelsConstants;

public class FlywheelsIOKrakenShooter implements FlywheelsIO {

    private final TalonFX leaderShooter, followerShooter;
    private final TalonFXConfiguration leaderConfig, followerConfig;
    private final TalonFXConfigurator leaderConfigurator, followerConfigurator;

    private VelocityVoltage velocityRequest;
    private double velocityTarget;

    public FlywheelsIOKrakenShooter(){
        leaderShooter = new TalonFX(shooterWheelsConstants.shooterLeaderID, CANBus.roboRIO());
        followerShooter = new TalonFX(shooterWheelsConstants.shooterFollowerID, CANBus.roboRIO());

        leaderConfig = new TalonFXConfiguration();
        followerConfig = new TalonFXConfiguration();

        leaderConfigurator = leaderShooter.getConfigurator();
        followerConfigurator = followerShooter.getConfigurator();

        velocityRequest = new VelocityVoltage(0);

        followerShooter.setControl(new Follower(shooterWheelsConstants.shooterLeaderID, MotorAlignmentValue.Opposed));
        configMotor();
    }

    public void configMotor(){
        var limitConfigs = leaderConfig.CurrentLimits;

        limitConfigs.SupplyCurrentLimitEnable = shooterWheelsConstants.SupplyCurrentLimitEnable;
        limitConfigs.SupplyCurrentLimit = shooterWheelsConstants.SupplyCurrentLimit;

        limitConfigs.StatorCurrentLimit = shooterWheelsConstants.StatorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = shooterWheelsConstants.StatorCurrentLimitEnable;

        var slot0Configs = leaderConfig.Slot0;

        slot0Configs.kS = shooterWheelsConstants.kS;
        slot0Configs.kV = shooterWheelsConstants.kV;
        slot0Configs.kP = shooterWheelsConstants.kP;
        slot0Configs.kI = shooterWheelsConstants.kI;
        slot0Configs.kD = shooterWheelsConstants.kD;

        leaderConfigurator.apply(leaderConfig);
        followerConfigurator.apply(followerConfig);

        leaderConfigurator.apply(limitConfigs);
        followerConfigurator.apply(limitConfigs);
    }
    
    @Override 
    public void updateInputs(FlyWheelsInputs inputs){
        inputs.velocityRPM = leaderShooter.getVelocity().getValueAsDouble() * 60.0;

        inputs.appliedVolts = leaderShooter.getMotorVoltage().getValueAsDouble();
        inputs.targetRPM = this.velocityTarget;

        inputs.current = leaderShooter.getStatorCurrent().getValueAsDouble();
    }

    @Override 
    public void setTargetRPM(double RPM){
        this.velocityTarget = RPM;
        leaderShooter.setControl(velocityRequest.withVelocity(RPM / 60).withSlot(0));   
    }

    @Override 
    public void applyOutput(double volts){
        leaderShooter.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed){
        leaderShooter.set(speed);
    }
}