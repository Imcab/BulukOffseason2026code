package frc.robot.modules.individuals.flywheels;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

@Fallback
public interface FlywheelsIO extends IO<FlywheelsIO.FlyWheelsInputs> {
    
    public static class FlyWheelsInputs extends Data<FlyWheelsInputs> {

    @Unit(value = "Volts", group = "FlyWheel")
    public double appliedVolts = 0;

    @Unit(value = "RPM", group = "FlyWheel")
    public double targetRPM = 0;

    @Unit(value = "RPM", group = "FlyWheel")
    public double velocityRPM = 0;

    @Unit(value = "Amps", group = "FlyWheel")
    public double current = 0;
  }

  public void applyOutput(@Unit(value = "Volts", group = "FlyWheel") double volts);

  public void setSpeed(@Unit(value = "DutyCycle", group = "FlyWheel") double speed);

  public void setTargetRPM(@Unit(value = "RPM", group = "FlyWheel") double rpm);

  public TalonFX getMotor();

}