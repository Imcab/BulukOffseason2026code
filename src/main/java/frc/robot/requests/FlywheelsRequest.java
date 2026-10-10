package frc.robot.requests;

import com.stzteam.features.dictionary.Dictionary.StatusCodes;
import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import frc.robot.configuration.constants.modules.FlywheelsConstants;
import frc.robot.modules.individuals.flywheels.Flywheels;
import frc.robot.modules.individuals.flywheels.FlywheelsIO;
import frc.robot.modules.individuals.flywheels.FlywheelsIO.FlyWheelsInputs;
import java.util.function.DoubleSupplier;

// Los ModuleColorCode con %.2f se formatean con String.format: hay que pasarles numeros, no Strings
@RequestFactory
public interface FlywheelsRequest extends Request<FlyWheelsInputs, FlywheelsIO> {

   public static class IdleIntake implements FlywheelsRequest {

    @Override
    public ActionStatus apply(FlyWheelsInputs parameters, FlywheelsIO actor) {
      actor.applyOutput(0);
      return ActionStatus.of(Flywheels.IDLE, StatusCodes.IDLE_STATUS);
    }
  }

  public static class IdleOutake implements FlywheelsRequest {

    @Override
    public ActionStatus apply(FlyWheelsInputs parameters, FlywheelsIO actor) {
      actor.applyOutput(FlywheelsConstants.shooterWheelsConstants.idleVoltage);
      return ActionStatus.of(Flywheels.IDLE, StatusCodes.IDLE_STATUS);
    }
  }

  @CreateCommand(name = "stop")
  public static class Idle implements FlywheelsRequest {
    @Override
    public ActionStatus apply(FlyWheelsInputs data, FlywheelsIO actor) {
      data.targetRPM = 0;
      actor.applyOutput(0);
      return ActionStatus.of(Flywheels.IDLE);
    }
  }

  @CreateCommand(name = "spinAtVoltage")
  public static class moveVoltage implements FlywheelsRequest {
    private DoubleSupplier volts = () -> 0;

    public moveVoltage withVolts(double volts) {
      this.volts = () -> volts;
      return this;
    }

    public moveVoltage withVolts(DoubleSupplier volts) {
      this.volts = volts;
      return this;
    }

    @Override
    public ActionStatus apply(FlyWheelsInputs data, FlywheelsIO actor) {
      double target = volts.getAsDouble();
      data.targetRPM = 0;
      actor.applyOutput(target);
      return ActionStatus.of(Flywheels.MANUAL_CONTROL, target);
    }
  }

  @CreateCommand(name = "toRPM")
  public static class setRPM implements FlywheelsRequest {
    private DoubleSupplier rpm = () -> 0;
    private double tolerance = 50;

    public setRPM toRPM(double rpm) {
      this.rpm = () -> rpm;
      return this;
    }

    public setRPM toRPM(DoubleSupplier rpm) {
      this.rpm = rpm;
      return this;
    }

    public setRPM withTolerance(double tolerance) {
      this.tolerance = tolerance;
      return this;
    }

    @Override
    public ActionStatus apply(FlyWheelsInputs data, FlywheelsIO actor) {
      double target = rpm.getAsDouble();
      data.targetRPM = target;
      actor.setTargetRPM(target);

      if (MathUtil.isNear(target, data.velocityRPM, tolerance)) {
        return ActionStatus.of(Flywheels.ON_TARGET, target);
      }
      return ActionStatus.of(Flywheels.MOVING_TO_RPM, target);
    }
  }
}
