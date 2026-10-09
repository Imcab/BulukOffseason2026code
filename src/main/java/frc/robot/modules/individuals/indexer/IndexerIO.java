package frc.robot.modules.individuals.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

@Fallback
public interface IndexerIO extends IO<IndexerIO.IndexerInputs> {

    public static class IndexerInputs extends Data<IndexerInputs> {
        @Unit(value = "Volts", group = "Indexer")
        public double rollerVolts = 0;

        @Unit(value = "RPM", group = "Indexer")
        public double rollerRPM = 0;

        @Unit(value = "Volts", group = "Indexer")
        public double indexVolts = 0;

        @Unit(value = "RPM", group = "Indexer")
        public double indexRPM = 0;

        // Corriente (A) de cada motor, para verificar la prueba por amperaje
        public double rollerAmps = 0;
        public double indexAmps = 0;

        @Override
        public IndexerInputs snapshot() {
            IndexerInputs clone = new IndexerInputs();
            clone.rollerVolts = this.rollerVolts;
            clone.rollerRPM = this.rollerRPM;
            clone.indexVolts = this.indexVolts;
            clone.indexRPM = this.indexRPM;
            clone.rollerAmps = this.rollerAmps;
            clone.indexAmps = this.indexAmps;
            return clone;
        }
    }

    public void applyRollers(@Unit(value = "Volts", group = "Indexer")double volts);
    /**
     * Sets the voltage of the rollers motor.
     * @param volts The voltage to apply to the rollers motor.
    */

    public void applyIndex(@Unit(value = "Volts", group = "Indexer")double volts);
    /**
     * Sets the voltage of the index motor.
     * @param volts The voltage to apply to the indexer motor.
     */

    public void setRollers(@Unit(value = "RPM", group = "Indexer")double RPM);
    /**
     * Sets the rollers to a given RPM with a closed loop controller.
     * @param RPM The desired RPM.
     */

    public void setIndex(@Unit(value = "RPM", group = "Indexer")double RPM);
    /**
     * Sets the index to a given RPM with a closed loop controller.
     * @param RPM The desired RPM.
     */

    /**
     * Sets the rollers in duty cycle (-1.0 to 1.0), no closed loop.
     */
    public default void setRollersSpeed(double speed) {
        applyRollers(speed * 12.0);
    }

    /**
     * Sets the index in duty cycle (-1.0 to 1.0), no closed loop.
     */
    public default void setIndexSpeed(double speed) {
        applyIndex(speed * 12.0);
    }

    /**
     * Control por corriente (amperes) de los rollers. Por defecto no hace nada (sim);
     * IndexerIOSpark lo implementa con ControlType.kCurrent.
     */
    public default void setRollersCurrent(double amps) {}

    /** Control por corriente (amperes) del index. Por defecto no hace nada (sim). */
    public default void setIndexCurrent(double amps) {}

    public void stopRollers();
    /**
     * Stops the rollers motor.
     */
    public void stopIndex();
    /**
     * Stops the index motor.
     */

    public void stopAll();
    /**
     * Stops the the indexer system (both roller and index)
     */

}