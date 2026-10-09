package frc.robot.configuration.bindings;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Barre voltajes, mide RPS estables y ajusta V = kS + kV * rps. Publica en NT. */
public class DumperTestCommand extends Command {
  private static final double[] VOLTS = {2, 3, 4, 5, 6, 7, 8, 9};
  private static final double SETTLE_S = 2.5;   // espera a que se estabilice
  private static final double SAMPLE_S = 0.7;   // promedia este tiempo
  private static final double MAX_RPS = 90;     // corte de seguridad (~5400 RPM)

  private final TalonFX motor;
  private final StatusSignal<AngularVelocity> vel;
  private final VoltageOut req = new VoltageOut(0);
  private final Timer timer = new Timer();
  private final double[] rps = new double[VOLTS.length];
  private int i, n;
  private double sum;
  private boolean abortado;

  public DumperTestCommand(TalonFX motor, Subsystem... reqs) {
    this.motor = motor;
    this.vel = motor.getVelocity();
    this.vel.setUpdateFrequency(100);
    addRequirements(reqs);
  }

  @Override public void initialize() {
    i = 0; n = 0; sum = 0; abortado = false;
    timer.restart();
    SmartDashboard.putString("Char/estado", "corriendo");
  }

  @Override public void execute() {
    vel.refresh();
    double v = vel.getValueAsDouble();            // RPS del rotor
    if (Math.abs(v) > MAX_RPS) { abortado = true; return; }

    motor.setControl(req.withOutput(VOLTS[i]));
    double t = timer.get();
    if (t > SETTLE_S) { sum += v; n++; }
    if (t > SETTLE_S + SAMPLE_S) {
      rps[i] = sum / n;
      SmartDashboard.putNumber("Char/rps_" + (int) VOLTS[i] + "V", rps[i]);
      sum = 0; n = 0; i++;
      timer.restart();
    }
  }

  @Override public boolean isFinished() { return abortado || i >= VOLTS.length; }

  @Override public void end(boolean interrupted) {
    motor.setControl(req.withOutput(0));
    if (abortado || interrupted) {
      SmartDashboard.putString("Char/estado", abortado ? "ABORTADO: demasiadas RPS" : "interrumpido");
      return;
    }
    // Regresión lineal V = kS + kV * rps (solo puntos que sí giraron)
    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0; int m = 0;
    for (int k = 0; k < VOLTS.length; k++) {
      if (rps[k] < 1.0) continue;
      double x = rps[k], y = VOLTS[k];
      sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y; m++;
    }
    if (m < 3) { SmartDashboard.putString("Char/estado", "FALLO: menos de 3 puntos validos"); return; }
    double kV = (m * sxy - sx * sy) / (m * sxx - sx * sx);
    double kS = (sy - kV * sx) / m;
    double r = (m * sxy - sx * sy)
        / Math.sqrt((m * sxx - sx * sx) * (m * syy - sy * sy));

    SmartDashboard.putNumber("Char/kS", kS);
    SmartDashboard.putNumber("Char/kV", kV);
    SmartDashboard.putNumber("Char/R2", r * r);
    Preferences.setDouble("Char_kS", kS);
    Preferences.setDouble("Char_kV", kV);
    SmartDashboard.putString("Char/estado",
        r * r > 0.99 ? "OK" : "OK pero R2 bajo: revisa fricción/puntos");
  }
}