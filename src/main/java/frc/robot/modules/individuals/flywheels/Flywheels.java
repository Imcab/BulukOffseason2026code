package frc.robot.modules.individuals.flywheels;

import java.util.function.Supplier;

import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.modules.individuals.flywheels.FlywheelsIO.FlyWheelsInputs;
import frc.robot.requests.FlywheelsCommands;
import frc.robot.requests.FlywheelsRequest;
import frc.robot.requests.FlywheelsRequestFactory;

public class Flywheels extends ModularSubsystem<FlyWheelsInputs, FlywheelsIO>
    implements FlywheelsCommands {

  public static final ModuleColorCode IDLE =
      ModuleColorCode.solid("IDLE", Severity.OK, Color.kDarkGreen, "Flywheel en reposo");
  public static final ModuleColorCode ON_TARGET =
      ModuleColorCode.solid("ON_TARGET", Severity.OK, Color.kFirstBlue, "En objetivo: %.2f RPM");
  public static final ModuleColorCode MOVING_TO_RPM =
      ModuleColorCode.solid(
          "MOVING_TO_RPM", Severity.WARNING, Color.kYellow, "Moviendo a %.2f RPM");
  public static final ModuleColorCode MANUAL_CONTROL =
      ModuleColorCode.solid(
          "MANUAL_CONTROL", Severity.WARNING, Color.kBrown, "Control manual: %.2fV");

  public Flywheels(FlywheelsIO io, String key) {
    super(
        SubsystemBuilder.<FlyWheelsInputs, FlywheelsIO>setup()
            .key(key)
            .hardware(io, new FlyWheelsInputs())
            .request(FlywheelsRequestFactory.idle())
            .telemetry(new FlywheelsTelemetry(key)));

    this.setDefaultCommand(runRequest(() -> FlywheelsRequestFactory.idle()));
  }

  public boolean isAtTarget(double toleranceRPM) {
    return MathUtil.isNear(inputs.targetRPM, inputs.velocityRPM, toleranceRPM);
  }

  /** Compara contra un RPM explicito (no depende de que el request ya haya escrito targetRPM). */
  public boolean isAtRPM(double rpm, double toleranceRPM) {
    return MathUtil.isNear(rpm, inputs.velocityRPM, toleranceRPM);
  }

  @Override
  public Command setControl(Supplier<FlywheelsRequest> request) {
    return runRequest(request);
  }

  public static class FlywheelsTelemetry extends Telemetry<FlyWheelsInputs> {

    private final String key;

    public FlywheelsTelemetry(String key) {
      this.key = key;
    }

    @Override
    public void telemeterize(FlyWheelsInputs data) {
      NetworkIO.set(key, "VelocityRPM", data.velocityRPM);
      NetworkIO.set(key, "TargetRPM", data.targetRPM);
      NetworkIO.set(key, "AppliedVolts", data.appliedVolts);
      NetworkIO.set(key, "Current", data.current);

      // Tuning
      NetworkIO.set(key, "Tuning/ErrorRPM", data.targetRPM - data.velocityRPM);
    }
  }

  @Override
  public void absolutePeriodic(FlyWheelsInputs inputs) {}

  @Override
  public void simulationPeriodic() {}
}
