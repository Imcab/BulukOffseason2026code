package frc.robot.requests;

import com.stzteam.features.dictionary.Dictionary.StatusCodes;
import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import frc.robot.modules.individuals.Dumper.DumperIO;
import frc.robot.modules.individuals.Dumper.DumperIO.DumperInputs;
import frc.robot.modules.individuals.Dumper.DumperSpark.DumperMODE;
import frc.robot.modules.individuals.intake.Intake;


@RequestFactory
public interface DumperRequest extends Request<DumperInputs, DumperIO> {

    @CreateCommand(name = "stop")
  public static class Idle implements DumperRequest {
    @Override
    public ActionStatus apply(DumperInputs data, DumperIO actor) {
      actor.stopAll();
      return ActionStatus.of(Intake.IDLE, "Idle");
    }
  }

  @CreateCommand(name = "seed")
  public static class resetPosition implements DumperRequest {
    @Override
    public ActionStatus apply(DumperInputs data, DumperIO actor) {
      actor.resetPosition();
      return ActionStatus.of(Intake.RESET, "Reseted");
    }
  }

  @CreateCommand(name = "toAngle")
  public static class setAngle implements DumperRequest {
    private double angle;
    private double tolerance = 1.0; // Grados de tolerancia por defecto
    private DumperMODE mode = DumperMODE.kUP;

    public setAngle(double initialAngle) {
      this.angle = initialAngle;
    }

    public setAngle withAngle(double angle) {
      this.angle = angle;
      return this;
    }

    public setAngle withMode(DumperMODE mode) {
      this.mode = mode;
      return this;
    }

    public setAngle Tolerance(double tolerance) {
      this.tolerance = tolerance;
      return this;
    }

    @Override
    public ActionStatus apply(DumperInputs parameters, DumperIO actor) {
      parameters.targetAngle = angle;
      actor.setPosition(angle, mode);

      boolean isAtTarget = MathUtil.isNear(angle, parameters.position, tolerance);

      if (isAtTarget) {
        return ActionStatus.of(Intake.ON_TARGET, StatusCodes.TARGETREACHED_STATUS);
      } else {
        // El template de MOVING_TO_ANGLE usa %.2f, hay que pasarle el numero, no un String
        return ActionStatus.of(Intake.MOVING_TO_ANGLE, angle);
      }
    }
  }

  @CreateCommand(name = "voltageCommand")
  public static class moveVoltage implements DumperRequest {
    private double voltage;

    public moveVoltage withVolts(double volts) {
      this.voltage = volts;
      return this;
    }

    @Override
    public ActionStatus apply(DumperInputs parameters, DumperIO actor) {
      actor.applyOutput(voltage);
      return ActionStatus.of(
          Intake.MANUAL_OVERRIDE, StatusCodes.MANUAL_STATUS + StatusCodes.voltsOf(voltage));
    }
  }
}