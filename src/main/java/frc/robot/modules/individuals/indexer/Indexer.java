package frc.robot.modules.individuals.indexer;

import java.util.function.Supplier;

import com.stzteam.features.dictionary.Dictionary.CommonTables;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.individuals.indexer.IndexerIO.IndexerInputs;
import frc.robot.requests.IndexerCommands;
import frc.robot.requests.IndexerRequest;

public class Indexer extends ModularSubsystem<IndexerInputs, IndexerIO> implements IndexerCommands{
    public Indexer(IndexerIO io) {
        super(
            SubsystemBuilder.<IndexerInputs, IndexerIO>setup()
                .key(KeyManager.INDEXER_KEY)
                .hardware(io, new IndexerInputs())
                .request(new IndexerRequest.idleIndexer())
                .telemetry(new IndexerTelemetry())
        );
        setDefaultCommand(runRequest(() -> new IndexerRequest.idleIndexer()));
    }


    @Override
    public void absolutePeriodic(IndexerInputs inputs) {}


public static class IndexerTelemetry extends Telemetry<IndexerInputs>{
    private static final String VELOCITY_INDEX_KEY = CommonTables.SPEED_KEY + KeyManager.INDEXER_KEY;
    private static final String VELOCITY_ROLLERS_KEY = CommonTables.SPEED_KEY + KeyManager.INDEXER_KEY;

    private static final String VOLTAGE_INDEX_KEY = CommonTables.VOLTAGE_KEY + KeyManager.INDEXER_KEY;
    private static final String VOLTAGE_ROLL_KEY = CommonTables.VOLTAGE_KEY + KeyManager.INDEXER_KEY;

    private static final String AMPS_INDEX_KEY = "IndexAmps";
    private static final String AMPS_ROLL_KEY = "RollerAmps";


        @Override
        public void telemeterize(IndexerInputs data) {

            NetworkIO.set(KeyManager.INDEXER_KEY, VELOCITY_INDEX_KEY, data.indexRPM);
            NetworkIO.set(KeyManager.INDEXER_KEY, VELOCITY_ROLLERS_KEY, data.rollerRPM);

            NetworkIO.set(KeyManager.INDEXER_KEY, VOLTAGE_INDEX_KEY, data.indexVolts);
            NetworkIO.set(KeyManager.INDEXER_KEY, VOLTAGE_ROLL_KEY, data.rollerVolts);

            NetworkIO.set(KeyManager.INDEXER_KEY, AMPS_INDEX_KEY, data.indexAmps);
            NetworkIO.set(KeyManager.INDEXER_KEY, AMPS_ROLL_KEY, data.rollerAmps);
        }
    }

    public Command setControl(Supplier<IndexerRequest> request) {
        return runRequest(request);

}



}
