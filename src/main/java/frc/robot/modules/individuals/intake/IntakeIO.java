package frc.robot.modules.individuals.intake;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

@Fallback
public interface IntakeIO extends IO<IntakeIO.IntakeInputs> {

    public class IntakeInputs extends Data<IntakeInputs> {

        @Unit(value = "RPS", group = "Intake")
        public double rollsRPS = 0.0;

        @Unit(value = "Volts", group = "Intake")
        public double rollsAppliedVolts = 0.0;

        @Unit(value = "Volts", group = "Intake")
        public double angulatorAppliedVolts = 0.0;

        @Unit(value = "Degrees", group = "Intake")
        public double angulatorTargetAngle = 0.0;

        @Unit(value = "Degrees", group = "Intake")
        public double angulatorPosition = 0.0;

    }

    public enum IntakeMODE{
        kBACK,
        kFRONT
    }

    public void setRollsVoltage(@Unit (value = "Volts" , group = "Intake")double volts);

    public void setRollsRPS(@Unit (value = "RPS" , group = "Intake")double rps);

    public void stopRolls();

    public void setAngulatorPosition(@Unit(value = "Degrees", group = "Intake")double position, IntakeMODE mode);

    public void setAngulatorVoltage(@Unit(value = "Volts", group = "Intake") double volts);

    public double getAngulatorPosition();

    public void resetAngulator();

    public void stopAngulator();
    
    public void stopAll();

}