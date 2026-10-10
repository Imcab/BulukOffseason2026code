package frc.robot.modules.individuals.Dumper;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

import frc.robot.modules.individuals.Dumper.DumperSpark.DumperMODE;

@Fallback

public interface DumperIO extends IO<DumperIO.DumperInputs>{

      public static class DumperInputs extends Data<DumperInputs> {

    @Unit(value = "Degrees", group = "Dumper")
    public double position = 0;

    @Unit(value = "Degrees", group = "Dumper")
    public double targetAngle = 0;

    @Unit(value = "Degrees", group = "Dumper")
    public double profileSetpoint = 0;

    @Unit(value = "DegreesPerSecond", group = "Dumper")
    public double profileVelocity = 0;

    @Unit(value = "DegreesPerSecond", group = "Dumper")
    public double velocity = 0;

    @Unit(value = "Volts", group = "Dumper")
    public double appliedVolts = 0;

    public double current = 0;
  }

  public void setPosition(@Unit(value = "Degrees", group = "Dumper") double Angle, DumperMODE mode);

  public void applyOutput(@Unit(value = "Volts", group = "Dumper") double volts);

  public void resetPosition();

  public void stopAll();
    
}
