package frc.robot.modules.individuals.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.configuration.constants.modules.IndexerConstants;

public class IndexerIOSpark implements IndexerIO {
    SparkMax rollMotor;
    SparkMax indexMotor;

    SparkMaxConfig rollConfig;
    SparkMaxConfig indexConfig;

    SparkClosedLoopController rollController;
    SparkClosedLoopController indexController;

    public IndexerIOSpark() {
        rollMotor = new SparkMax(IndexerConstants.rollerID, MotorType.kBrushless);
        indexMotor = new SparkMax(IndexerConstants.indexID, MotorType.kBrushless);

        rollConfig = new SparkMaxConfig();
        indexConfig = new SparkMaxConfig();

        rollController = rollMotor.getClosedLoopController();
        indexController = indexMotor.getClosedLoopController();

        rollConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .encoder.positionConversionFactor(1/IndexerConstants.indexRatio);
        // PID de velocidad de los rollers (el FF va en setRollers como arbFeedforward en volts)
        rollConfig.closedLoop.pid(
            IndexerConstants.kRollerP, IndexerConstants.kRollerI, IndexerConstants.kRollerD, ClosedLoopSlot.kSlot0);
        // Lazo de corriente (slot 1) para ControlType.kCurrent
        rollConfig.closedLoop.pid(
            IndexerConstants.kCurrentP, IndexerConstants.kCurrentI, IndexerConstants.kCurrentD, ClosedLoopSlot.kSlot1);
        indexConfig.closedLoop.pid(
            IndexerConstants.kIndexP, IndexerConstants.kIndexI, IndexerConstants.kIndexD, ClosedLoopSlot.kSlot0);
        indexConfig.closedLoop.pid(
            IndexerConstants.kCurrentP, IndexerConstants.kCurrentI, IndexerConstants.kCurrentD, ClosedLoopSlot.kSlot1);
        indexConfig
            .idleMode(IdleMode.kCoast)
            .encoder.positionConversionFactor(1/IndexerConstants.indexRatio); 
        

        rollMotor.configure(rollConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    @Override
	public void updateInputs(IndexerInputs inputs) {
        inputs.rollerVolts = rollMotor.getAppliedOutput() * rollMotor.getBusVoltage();
        inputs.rollerRPM = rollMotor.getEncoder().getVelocity();
        
        inputs.indexVolts = indexMotor.getAppliedOutput() * indexMotor.getBusVoltage();
        inputs.indexRPM = indexMotor.getEncoder().getVelocity();

        inputs.rollerAmps = rollMotor.getOutputCurrent();
        inputs.indexAmps = indexMotor.getOutputCurrent();
	}

    @Override
    public void applyRollers(double volts) {
        rollMotor.setVoltage(volts);
    }
    
    @Override
    public void applyIndex(double volts) {
        indexMotor.setVoltage(volts);
    }
    
    @Override
    public void setRollers(double RPM) {
        double ffVolts = IndexerConstants.kRollerS * Math.signum(RPM) + IndexerConstants.kRollerV * RPM;
        rollController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
    }
        
    @Override
    public void setIndex(double RPM) {
        double ffVolts = IndexerConstants.kIndexS * Math.signum(RPM) + IndexerConstants.kIndexV * RPM;
        indexController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
    }
    

    @Override
    public void setRollersCurrent(double amps) {
        rollController.setSetpoint(amps, ControlType.kCurrent, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setIndexCurrent(double amps) {
        indexController.setSetpoint(amps, ControlType.kCurrent, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setRollersSpeed(double speed) {
        rollMotor.set(speed);
    }

    @Override
    public void setIndexSpeed(double speed) {
        indexMotor.set(speed);
    }

    @Override
    public void stopRollers() {
        rollMotor.stopMotor();
    }

    @Override
    public void stopIndex() {
        indexMotor.stopMotor();
    }
    
    @Override
    public void stopAll() {
        indexMotor.stopMotor();
        rollMotor.stopMotor();
    }
    

}