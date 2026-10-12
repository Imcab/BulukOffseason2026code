// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

/**
 * Detecta cuando la vision corrige la pose: compara la pose fusionada con la que predice la
 * odometria (pose anterior + velocidad medida * dt). La odometria sola explica el movimiento; lo
 * que no explica es una correccion de la Limelight.
 *
 * <p>Las diferencias se suman con olvido (ventana de DECAY_S): una correccion de vision llega en unos
 * pocos ciclos y se detecta; un desfase lento (redondeo de la prediccion, ruido, sesgos chicos) se
 * olvida antes de acumular y no dispara replaneos de mas.
 */
public class PoseJumpDetector {

  /** Constante de tiempo del olvido: solo cuentan las correcciones de los ultimos ~0.25 s. */
  public static final double DECAY_S = 0.25;

  private final Supplier<SwerveDriveState> state;
  private Pose2d lastPose;
  private double lastTime;
  private Translation2d accumulated = Translation2d.kZero;

  public PoseJumpDetector(Supplier<SwerveDriveState> state) {
    this.state = state;
  }

  public void reset() {
    lastPose = null;
    accumulated = Translation2d.kZero;
  }

  /** Correccion reciente (m): suma con olvido desde el ultimo reset. Llamar cada ciclo. */
  public double update() {
    SwerveDriveState s = state.get();
    Pose2d pose = s.Pose;
    double now = s.Timestamp;
    if (lastPose != null && now > lastTime) {
      double dt = now - lastTime;
      // Speeds es robot-relative: pasar a campo con el heading.
      Translation2d v =
          new Translation2d(s.Speeds.vxMetersPerSecond, s.Speeds.vyMetersPerSecond)
              .rotateBy(pose.getRotation());
      Translation2d predicted = lastPose.getTranslation().plus(v.times(dt));
      double keep = Math.exp(-dt / DECAY_S);
      accumulated = accumulated.times(keep).plus(pose.getTranslation().minus(predicted));
    }
    lastPose = pose;
    lastTime = now;
    return accumulated.getNorm();
  }
}
