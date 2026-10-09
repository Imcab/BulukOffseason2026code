package frc.robot.requests;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.modules.individuals.indexer.IndexerIO;
import frc.robot.modules.individuals.indexer.IndexerIO.IndexerInputs;


@RequestFactory
public interface IndexerRequest extends Request<IndexerInputs, IndexerIO> {

    public static ModuleColorCode IDLE =
        ModuleColorCode.solid("IDLE", Severity.OK, Color.kBlueViolet, "Indexer is idle");
    public static ModuleColorCode INDEXING_VOLTS=
        ModuleColorCode.solid("INDEXING_VOLTS", Severity.OK, Color.kSteelBlue, "Working index");
    public static ModuleColorCode INDEXING_RPM=
        ModuleColorCode.solid("INDEXING_RPM", Severity.OK, Color.kSteelBlue, "Working index");
    public static ModuleColorCode ROLLING_VOLTS=
        ModuleColorCode.solid("ROLLING_VOLTS", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode ROLLING_RPM=
        ModuleColorCode.solid("ROLLING_RPM", Severity.OK, Color.kAquamarine, "Working rollers");
    public static ModuleColorCode PROCESSING=
        ModuleColorCode.solid("PROCESSING", Severity.OK, Color.kAquamarine, "Rollers and Indexer working");



     @CreateCommand(name = "idleIndexer")
     public static class idleIndexer implements IndexerRequest{
        @Override
        public ActionStatus apply(IndexerInputs data, IndexerIO actor) {
            actor.stopAll();
            return ActionStatus.of(IDLE, "Indexer is idle");
        }
    }

    @CreateCommand(name = "setRollers")
    public static class setRollers implements IndexerRequest{
        public double m_volts = 0;
        public double RPM = 0;
        public setRollers withVolts(double volts){
            m_volts = volts;
            return this;
        }
        public setRollers withRPM(double RPM){
            this.RPM = RPM;
            return this;
        }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            if(m_volts != 0){
                actor.applyRollers(m_volts);
                return ActionStatus.of(ROLLING_VOLTS, "Only rollers voltage");
            } else if(RPM != 0){
                actor.setRollers(RPM);
                return ActionStatus.of(ROLLING_RPM, "Only rollers RPM");
            } else {
                actor.stopRollers();
                return ActionStatus.of(IDLE, "Indexer is idle");
            }
        }
    }

    @CreateCommand(name = "setIndex")
    public static class setIndex implements IndexerRequest{
        public double m_volts = 0;
        public double RPM = 0;
        public setIndex withVolts(double volts){
            m_volts = volts;
            return this;
            }
        public setIndex withRPM(double RPM){
            this.RPM = RPM;
            return this;
            }
        @Override
        public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
            if(m_volts != 0){
                actor.applyIndex(m_volts);
                return ActionStatus.of(INDEXING_VOLTS, "Only index voltage");
            } else if(RPM != 0){
                actor.setIndex(RPM);
                return ActionStatus.of(INDEXING_RPM, "Only index RPM");
            } else {
                actor.stopIndex();
                return ActionStatus.of(IDLE, "Indexer is idle");
                }
            }
        }
        @CreateCommand(name = "processing")
        public static class Processing implements IndexerRequest{
            public double m_rollerVolts = 0;
            public double m_indexVolts = 0;
            public Processing withRollers(double volts){
                m_rollerVolts = volts;
                return this;
            }
            public Processing withIndex(double volts){
                m_indexVolts = volts;
                return this;
            }
            @Override
            public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
                actor.applyRollers(m_rollerVolts);
                actor.applyIndex(m_indexVolts);
                return ActionStatus.of(PROCESSING, "Rollers and Index working");
            }
        }

        @CreateCommand(name = "processingSpeed")
        public static class ProcessingSpeed implements IndexerRequest{
            public double m_rollerSpeed = 0;
            public double m_indexSpeed = 0;
            public ProcessingSpeed withRollers(double speed){
                m_rollerSpeed = speed;
                return this;
            }
            public ProcessingSpeed withIndex(double speed){
                m_indexSpeed = speed;
                return this;
            }
            @Override
            public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
                actor.setRollersSpeed(m_rollerSpeed);
                actor.setIndexSpeed(m_indexSpeed);
                return ActionStatus.of(PROCESSING, "Rollers and Index working");
            }
        }
        @CreateCommand(name = "processingRPM")
        public static class ProcessingRPM implements IndexerRequest{
            public double m_rollerRPM = 0;
            public double m_indexRPM = 0;
            public ProcessingRPM withRollers(double rpm){
                m_rollerRPM = rpm;
                return this;
            }
            public ProcessingRPM withIndex(double rpm){
                m_indexRPM = rpm;
                return this;
            }
            @Override
            public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
                actor.setRollers(m_rollerRPM);
                actor.setIndex(m_indexRPM);
                return ActionStatus.of(PROCESSING, "Rollers and Index at RPM");
            }
        }
        @CreateCommand(name = "processingAmps")
        public static class ProcessingAmps implements IndexerRequest{
            public double m_rollerAmps = 0;
            public double m_indexAmps = 0;
            public ProcessingAmps withRollers(double amps){
                m_rollerAmps = amps;
                return this;
            }
            public ProcessingAmps withIndex(double amps){
                m_indexAmps = amps;
                return this;
            }
            @Override
            public ActionStatus apply(IndexerInputs inputs, IndexerIO actor) {
                actor.setRollersCurrent(m_rollerAmps);
                actor.setIndexCurrent(m_indexAmps);
                return ActionStatus.of(PROCESSING, "Rollers and Index at current");
            }
        }
}
