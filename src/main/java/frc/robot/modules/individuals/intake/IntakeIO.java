package frc.robot.modules.individuals.intake;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

import frc.robot.modules.individuals.intake.IntakeSpark.intakeMODE;

@Fallback
public interface IntakeIO extends IO<IntakeIO.IntakeInputs> {

  public static class IntakeInputs extends Data<IntakeInputs> {

    @Unit(value = "Degrees", group = "Intake")
    public double position = 0;

    @Unit(value = "Degrees", group = "Intake")
    public double targetAngle = 0;

    @Unit(value = "Degrees", group = "Intake")
    public double profileSetpoint = 0;

    @Unit(value = "DegreesPerSecond", group = "Intake")
    public double profileVelocity = 0;

    @Unit(value = "DegreesPerSecond", group = "Intake")
    public double velocity = 0;

    @Unit(value = "Volts", group = "Intake")
    public double appliedVolts = 0;

    public double current = 0;
  }

  public void setPosition(@Unit(value = "Degrees", group = "Intake") double Angle, intakeMODE mode);

  public void applyOutput(@Unit(value = "Volts", group = "Intake") double volts);

  public void resetPosition();

  public void stopAll();
}