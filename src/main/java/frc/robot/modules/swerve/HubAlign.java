// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static frc.robot.configuration.constants.swerve.HubAlignConstants.*;

import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * Lleva el robot a la posicion de tiro: SHOOT_DISTANCE_M frente a la cara del hub de la propia
 * alianza (tags 25/26 azul, 9/10 rojo), con la parte de atras del robot hacia el hub.
 *
 * <ol>
 *   <li>Revisa que haya alianza y vision reciente (la pose viene de la Limelight fusionada).
 *   <li>Pathfinding con PoseFinder (PathPlanner + navgrid, esquiva hub y torre). Se salta si ya
 *       esta cerca.
 *   <li>Ajuste fino con {@link DriveToPose} hasta quedar dentro de tolerancia.
 * </ol>
 *
 * <p>Pensado para whileTrue: al soltar el boton se cancela y el robot regresa al control manual.
 * Telemetria en SmartDashboard "HubAlign/*" (estado, error y un Field2d con el objetivo).
 *
 * <p>Interruptor "HubAlign/Permitir solo odometria" (default apagado): si se enciende, sin vision
 * reciente se alinea igual con la pose de la odometria (la ultima que fijo la Limelight + ruedas).
 * Util en un campo de pruebas con pocos tags; en competencia dejarlo apagado.
 */
public final class HubAlign {

  private static final String KEY = "HubAlign/";
  private static final String ALLOW_ODOMETRY_KEY = KEY + "Permitir solo odometria";
  private static Field2d field;

  private HubAlign() {}

  /**
   * @param secondsSinceVision segundos desde la ultima medicion de vision aceptada.
   */
  public static Command command(CommandSwerveDrivetrain drivetrain, DoubleSupplier secondsSinceVision) {
    AprilTagFieldLayout layout = PoseFusion.defaultFieldLayout();
    SmartDashboard.setDefaultBoolean(ALLOW_ODOMETRY_KEY, false);
    return Commands.defer(() -> build(drivetrain, layout, secondsSinceVision), Set.of(drivetrain))
        .withName("AlinearHub");
  }

  /** Pose de tiro de la alianza actual, o vacio si todavia no hay alianza. */
  public static Optional<Pose2d> currentTarget() {
    return DriverStation.getAlliance()
        .map(a -> HubTargets.shootingPose(PoseFusion.defaultFieldLayout(), a));
  }

  /** true si el robot esta en la posicion de tiro (para condicionar el disparo). */
  public static boolean isAtShootingPose(CommandSwerveDrivetrain drivetrain) {
    return currentTarget()
        .map(t -> DriveToPose.isWithinTolerance(drivetrain.getState().Pose, t))
        .orElse(false);
  }

  private static Command build(
      CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout layout, DoubleSupplier secondsSinceVision) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return status("sin alianza: conecta el Driver Station / FMS");
    }
    double visionAge = secondsSinceVision.getAsDouble();
    String age = Double.isInfinite(visionAge) ? "nunca" : String.format("hace %.1f s", visionAge);
    SmartDashboard.putNumber(KEY + "vision_s", Double.isInfinite(visionAge) ? -1 : visionAge);
    boolean staleVision = REQUIRE_RECENT_VISION && visionAge > MAX_VISION_AGE_S;
    boolean allowOdometry = SmartDashboard.getBoolean(ALLOW_ODOMETRY_KEY, false);
    if (staleVision && !allowOdometry) {
      return status("sin vision reciente (" + age + "): no se mueve");
    }
    String mode = staleVision ? " [SOLO ODOMETRIA, vision " + age + "]" : "";

    Pose2d target = HubTargets.shootingPose(layout, alliance.get());
    Pose2d start = drivetrain.getState().Pose;
    boolean close =
        start.getTranslation().getDistance(target.getTranslation()) < SKIP_PATHFIND_DISTANCE_M
            && Math.abs(target.getRotation().minus(start.getRotation()).getDegrees())
                < SKIP_PATHFIND_ANGLE_DEG;

    Field2d f = field();
    f.getObject("objetivo").setPose(target);
    SmartDashboard.putString(
        KEY + "objetivo",
        String.format(
            "%s (%.3f, %.3f) %.1f deg",
            alliance.get(), target.getX(), target.getY(), target.getRotation().getDegrees()));

    Command path =
        close
            ? Commands.none()
            : drivetrain.getPoseFinder().toPose(target, PATHFIND_CONSTRAINTS);

    Command steps =
        Commands.sequence(
            status((close ? "cerca: solo ajuste fino" : "pathfinding") + mode),
            path,
            status("ajuste fino" + mode),
            new DriveToPose(drivetrain, target).withTimeout(FINE_TIMEOUT_S),
            Commands.runOnce(
                () ->
                    SmartDashboard.putString(
                        KEY + "estado",
                        (DriveToPose.isWithinTolerance(drivetrain.getState().Pose, target)
                                ? "alineado"
                                : "timeout: fuera de tolerancia")
                            + mode)));

    // Telemetria en paralelo (sin requirements) mientras corren los pasos.
    return Commands.deadline(steps, Commands.run(() -> publish(drivetrain, target)))
        .finallyDo(
            interrupted -> {
              publish(drivetrain, target);
              if (interrupted) {
                SmartDashboard.putString(KEY + "estado", "cancelado");
              }
            });
  }

  private static Command status(String text) {
    return Commands.runOnce(() -> SmartDashboard.putString(KEY + "estado", text));
  }

  private static void publish(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    Pose2d pose = drivetrain.getState().Pose;
    field().setRobotPose(pose);
    SmartDashboard.putNumber(
        KEY + "error_cm", 100 * pose.getTranslation().getDistance(target.getTranslation()));
    SmartDashboard.putNumber(
        KEY + "error_deg", target.getRotation().minus(pose.getRotation()).getDegrees());
    SmartDashboard.putBoolean(KEY + "alineado", DriveToPose.isWithinTolerance(pose, target));
  }

  private static Field2d field() {
    if (field == null) {
      field = new Field2d();
      SmartDashboard.putData(KEY + "Campo", field);
    }
    return field;
  }
}
