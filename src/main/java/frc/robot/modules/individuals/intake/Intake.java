package frc.robot.modules.individuals.intake;

import java.util.function.Supplier;

import com.stzteam.features.dictionary.Dictionary.CommonTables;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.individuals.intake.IntakeIO.IntakeInputs;
import frc.robot.requests.IntakeCommands;
import frc.robot.requests.IntakeRequest;
import frc.robot.requests.IntakeRequestFactory;

public class Intake extends ModularSubsystem<IntakeInputs, IntakeIO> implements IntakeCommands {

  public static final ModuleColorCode IDLE =
      ModuleColorCode.solid("IDLE", Severity.OK, Color.kDarkGreen, "Intake en reposo");
  public static final ModuleColorCode ON_TARGET =
      ModuleColorCode.solid("ON_TARGET", Severity.OK, Color.kFirstBlue, "Intake en objetivo");
  public static final ModuleColorCode MOVING_TO_ANGLE =
      ModuleColorCode.solid(
          "MOVING_TO_ANGLE", Severity.WARNING, Color.kYellow, "Intake moviéndose a %.2f grados");
  public static final ModuleColorCode MANUAL_OVERRIDE =
      ModuleColorCode.solid(
          "MANUAL_OVERRIDE", Severity.WARNING, Color.kPurple, "Intake en control manual");
  public static final ModuleColorCode RESET =
      ModuleColorCode.solid("RESET", Severity.OK, Color.kDarkSalmon, "Intake reiniciado");
  public static final ModuleColorCode OUT_OF_RANGE =
      ModuleColorCode.solid("OUT_OF_RANGE", Severity.ERROR, Color.kOrange, "Intake fuera de rango");

  public Intake(IntakeIO io) {

    super(
        SubsystemBuilder.<IntakeInputs, IntakeIO>setup()
            .key(KeyManager.INTAKE_KEY)
            .hardware(io, new IntakeInputs())
            .request(IntakeRequestFactory.idle())
            .telemetry(new IntakeTelemetry()));

    this.setDefaultCommand(runRequest(() -> IntakeRequestFactory.idle()));
  }



  public boolean isAtTarget(double toleranceDegrees) {
    return MathUtil.isNear(inputs.targetAngle, inputs.position, toleranceDegrees);
  }

  @Override
  public void absolutePeriodic(IntakeInputs inputs) {}

  @Override
  public Command setControl(Supplier<IntakeRequest> request) {
    return runRequest(request);
  }

  public static class IntakeTelemetry extends Telemetry<IntakeInputs> {

    private static final String APPLIED_VOLTS_KEY = CommonTables.APPLIED_KEY + "volts";

    @Override
    public void telemeterize(IntakeInputs data) {

      NetworkIO.set(KeyManager.INTAKE_KEY, CommonTables.DEGREES_KEY, data.position);
      NetworkIO.set(KeyManager.INTAKE_KEY, CommonTables.TARGET_KEY, data.targetAngle);
      NetworkIO.set(KeyManager.INTAKE_KEY, CommonTables.TIMESTAMP_KEY, data.timestamp);
      NetworkIO.set(KeyManager.INTAKE_KEY, APPLIED_VOLTS_KEY, data.appliedVolts);

      NetworkIO.set(KeyManager.INTAKE_KEY, "Current", data.current);

      // Tuning
      NetworkIO.set(KeyManager.INTAKE_KEY, "Tuning/ProfileSetpoint", data.profileSetpoint);
      NetworkIO.set(KeyManager.INTAKE_KEY, "Tuning/ProfileVelocity", data.profileVelocity);
      NetworkIO.set(KeyManager.INTAKE_KEY, "Tuning/Velocity", data.velocity);
      NetworkIO.set(KeyManager.INTAKE_KEY, "Tuning/Error", data.targetAngle - data.position);
    }
  }

  @Override
  public void simulationPeriodic() {}
}
