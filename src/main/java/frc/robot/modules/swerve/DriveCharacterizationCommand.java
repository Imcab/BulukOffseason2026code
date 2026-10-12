package frc.robot.modules.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.constants.swerve.TunerConstants;

/**
 * kS / kV del drive SIN espacio y SIN SignalLogger: el robot gira en su lugar (modulos tangentes,
 * SysIdSwerveRotation = VoltageOut al drive), barre voltajes en ambos sentidos, mide las rps
 * estables del ROTOR (promedio de los 4 drives) y ajusta |V| = kS + kV * |rps|.
 * Las ruedas ruedan sobre la alfombra con el peso del robot, asi que kS incluye esa friccion.
 *
 * <p>De paso estima el radio efectivo de la llanta con el giroscopio:
 * r = w_robot * r_modulo / (2*pi * rps_rueda).
 *
 * <p>Resultado en SmartDashboard "DriveChar/*" y en Preferences (DriveChar_kS, DriveChar_kV).
 * Usa kV y kS en TunerConstants.driveGains (unidades: V y V/rps del rotor).
 */
public class DriveCharacterizationCommand extends Command {
  private static final double[] VOLTS = {1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0};
  private static final double SETTLE_S = 1.5; // espera a que la velocidad se estabilice
  private static final double SAMPLE_S = 0.5; // promedia este tiempo
  private static final double MAX_ROTOR_RPS = 80; // corte de seguridad (~8.5 V)

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.SysIdSwerveRotation req = new SwerveRequest.SysIdSwerveRotation();
  private final StatusSignal<AngularVelocity>[] vel;
  private final StatusSignal<AngularVelocity> yawRate;
  private final double moduleRadius;
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double gearRatio;
  private final double[] stepVolts = new double[VOLTS.length * 2];
  private final double[] stepRps = new double[VOLTS.length * 2];
  private final double[] stepYaw = new double[VOLTS.length * 2];
  private final Timer timer = new Timer();
  private int step, n;
  private double sumRps, sumYaw;
  private boolean aborted;

  @SuppressWarnings("unchecked")
  public DriveCharacterizationCommand(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    vel = new StatusSignal[4];
    for (int i = 0; i < 4; i++) {
      vel[i] = drivetrain.getModule(i).getDriveMotor().getVelocity();
    }
    yawRate = drivetrain.getPigeon2().getAngularVelocityZWorld();
    moduleRadius = drivetrain.getModuleLocations()[0].getNorm();
    gearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
    for (int i = 0; i < VOLTS.length; i++) {
      stepVolts[i] = VOLTS[i];
      stepVolts[VOLTS.length + i] = -VOLTS[i];
    }
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    step = 0;
    n = 0;
    sumRps = 0;
    sumYaw = 0;
    aborted = false;
    timer.restart();
    SmartDashboard.putString("DriveChar/estado", "corriendo (el robot gira en su lugar)");
  }

  @Override
  public void execute() {
    BaseStatusSignal.refreshAll(vel[0], vel[1], vel[2], vel[3], yawRate);
    double rps = 0;
    for (var v : vel) rps += Math.abs(v.getValueAsDouble()) / 4.0; // rps del rotor
    if (rps > MAX_ROTOR_RPS) {
      aborted = true;
      return;
    }

    // SysIdSwerveRotation aplica V = RotationalRate * r / kSpeedAt12Volts * 12 a cada drive.
    double volts = stepVolts[step];
    drivetrain.setControl(req.withRotationalRate(volts * maxSpeed / (12.0 * moduleRadius)));

    double t = timer.get();
    if (t > SETTLE_S) {
      sumRps += rps;
      sumYaw += Math.abs(Math.toRadians(yawRate.getValueAsDouble()));
      n++;
    }
    if (t > SETTLE_S + SAMPLE_S) {
      stepRps[step] = sumRps / n;
      stepYaw[step] = sumYaw / n;
      SmartDashboard.putNumber("DriveChar/rps_" + volts + "V", stepRps[step]);
      sumRps = 0;
      sumYaw = 0;
      n = 0;
      step++;
      timer.restart();
    }
  }

  @Override
  public boolean isFinished() {
    return aborted || step >= stepVolts.length;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withRotationalRate(0));
    if (aborted || interrupted) {
      SmartDashboard.putString(
          "DriveChar/estado", aborted ? "ABORTADO: demasiadas rps" : "interrumpido");
      return;
    }
    // Regresion |V| = kS + kV * |rps| con los dos sentidos juntos (solo puntos que si giraron).
    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    int m = 0;
    double sumR = 0;
    int mr = 0;
    for (int k = 0; k < stepVolts.length; k++) {
      if (stepRps[k] < 1.0) continue;
      double x = stepRps[k], y = Math.abs(stepVolts[k]);
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
      syy += y * y;
      m++;
      double wheelRps = stepRps[k] / gearRatio;
      if (stepYaw[k] > 1.0) { // > 1 rad/s para que el giroscopio sea confiable
        sumR += stepYaw[k] * moduleRadius / (2 * Math.PI * wheelRps);
        mr++;
      }
    }
    if (m < 3) {
      SmartDashboard.putString("DriveChar/estado", "FALLO: menos de 3 puntos validos");
      return;
    }
    double kV = (m * sxy - sx * sy) / (m * sxx - sx * sx);
    double kS = (sy - kV * sx) / m;
    double r = (m * sxy - sx * sy) / Math.sqrt((m * sxx - sx * sx) * (m * syy - sy * sy));

    SmartDashboard.putNumber("DriveChar/kS", kS);
    SmartDashboard.putNumber("DriveChar/kV", kV);
    SmartDashboard.putNumber("DriveChar/R2", r * r);
    double wheelR = TunerConstants.FrontLeft.WheelRadius;
    SmartDashboard.putNumber("DriveChar/kV_teorico", 12.0 / (maxSpeed / (2 * Math.PI * wheelR) * gearRatio));
    if (mr > 0) {
      SmartDashboard.putNumber("DriveChar/radioRueda_m", sumR / mr);
    }
    Preferences.setDouble("DriveChar_kS", kS);
    Preferences.setDouble("DriveChar_kV", kV);
    SmartDashboard.putString(
        "DriveChar/estado", r * r > 0.99 ? "OK" : "OK pero R2 bajo: revisa alfombra/bateria");
  }
}
