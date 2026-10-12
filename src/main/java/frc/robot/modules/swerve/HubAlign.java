// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static frc.robot.configuration.constants.swerve.HubAlignConstants.*;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Lleva el robot a la posicion de tiro: SHOOT_DISTANCE_M frente a la cara del hub de la propia
 * alianza (tags 25/26 azul, 9/10 rojo), con la parte de atras del robot hacia el hub.
 *
 * <p>Vision y odometria trabajan juntas todo el tiempo: el robot navega con la pose fusionada del
 * drivetrain. Sin tags a la vista avanza con odometria; cuando la Limelight ve un tag la pose se
 * corrige, y si la correccion es grande (REPLAN_JUMP_M) se replanea la ruta desde la pose corregida.
 *
 * <ol>
 *   <li>Revisa que haya alianza y que la vision haya visto un tag alguna vez desde que prendio.
 *   <li>Pathfinding con PoseFinder (PathPlanner + navgrid, rodea hub y torre), replaneando si la
 *       vision corrige la pose. Se salta si ya esta cerca.
 *   <li>Ajuste fino con {@link DriveToPose} sobre la pose viva (si ve un tag, corrige; si no,
 *       termina con odometria).
 * </ol>
 *
 * <p>Pensado para whileTrue: al soltar el boton se cancela y el robot regresa al control manual.
 * Telemetria en SmartDashboard "HubAlign/*" (estado, error, replaneos y un Field2d).
 */
public final class HubAlign {

  private static final String KEY = "HubAlign/";
  private static Field2d field;
  private static SendableChooser<HubShotSpot> spotChooser;

  private HubAlign() {}

  /**
   * @param secondsSinceVision segundos desde la ultima medicion de vision aceptada (infinito si
   *     nunca).
   */
  public static Command command(CommandSwerveDrivetrain drivetrain, DoubleSupplier secondsSinceVision) {
    return command(drivetrain, secondsSinceVision, () -> HubShotSpot.CENTRO);
  }

  /** @param spot variante de posicion; se lee al presionar el boton. */
  public static Command command(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier secondsSinceVision, Supplier<HubShotSpot> spot) {
    AprilTagFieldLayout layout = PoseFusion.defaultFieldLayout();
    return Commands.defer(() -> build(drivetrain, layout, secondsSinceVision, spot.get()), Set.of(drivetrain))
        .withName("AlinearHub");
  }

  /**
   * Botones en el dashboard para probar sin control (HubAlign/Probar/*): uno por variante. En
   * simulacion agrega HubAlign/Sim/* para poner el robot en poses de prueba. Los comandos que
   * mueven el robot solo corren con el robot habilitado (en simulacion: Teleoperated en el DS).
   */
  public static void publishTestButtons(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier secondsSinceVision) {
    for (HubShotSpot s : HubShotSpot.values()) {
      SmartDashboard.putData(
          KEY + "Probar/" + s.label,
          command(drivetrain, secondsSinceVision, () -> s).withName("Alinear " + s.label));
    }
    if (RobotBase.isSimulation()) {
      // Poses de inicio de las pruebas automaticas, en coordenadas de la alianza azul; en rojo se
      // rotan 180 grados alrededor del centro del campo.
      double[][] starts = {
        {2.8, 2.0, 30}, {6.8, 4.0, 0}, {3.2, 1.2, 0}, {6.8, 1.5, 90}, {6.5, 6.5, 200}
      };
      for (double[] st : starts) {
        String name = String.format("Robot a (%.1f, %.1f, %.0f)", st[0], st[1], st[2]);
        SmartDashboard.putData(
            KEY + "Sim/" + name,
            Commands.runOnce(
                    () -> {
                      Pose2d p = new Pose2d(st[0], st[1], Rotation2d.fromDegrees(st[2]));
                      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                        p = new Pose2d(
                            PoseFusion.defaultFieldLayout().getFieldLength() - p.getX(),
                            PoseFusion.defaultFieldLayout().getFieldWidth() - p.getY(),
                            p.getRotation().plus(Rotation2d.k180deg));
                      }
                      drivetrain.resetPose(p);
                    })
                .ignoringDisable(true)
                .withName(name));
      }
    }
    publishSpots();
  }

  /** Dibuja todas las posiciones de tiro de la alianza actual en HubAlign/Campo. */
  public static void publishSpots() {
    DriverStation.getAlliance()
        .ifPresent(
            a -> {
              java.util.List<Pose2d> spots = new java.util.ArrayList<>();
              for (HubShotSpot s : HubShotSpot.values()) {
                if (s.isFixed()) {
                  spots.add(HubTargets.shootingPose(PoseFusion.defaultFieldLayout(), a, s.angleDeg));
                }
              }
              field().getObject("posiciones").setPoses(spots);
            });
  }

  /** Selector "HubAlign/Posicion" en el dashboard (default: Centro). */
  public static SendableChooser<HubShotSpot> spotChooser() {
    if (spotChooser == null) {
      spotChooser = new SendableChooser<>();
      spotChooser.setDefaultOption(HubShotSpot.CENTRO.label, HubShotSpot.CENTRO);
      for (HubShotSpot s : HubShotSpot.values()) {
        if (s != HubShotSpot.CENTRO) {
          spotChooser.addOption(s.label, s);
        }
      }
      SmartDashboard.putData(KEY + "Posicion", spotChooser);
    }
    return spotChooser;
  }

  /** Pose de tiro de la alianza actual, o vacio si todavia no hay alianza. */
  public static Optional<Pose2d> currentTarget() {
    return DriverStation.getAlliance()
        .map(a -> HubTargets.shootingPose(PoseFusion.defaultFieldLayout(), a));
  }

  /** true si el robot esta en cualquiera de las posiciones de tiro (para condicionar el disparo). */
  public static boolean isAtShootingPose(CommandSwerveDrivetrain drivetrain) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    Pose2d pose = drivetrain.getState().Pose;
    for (HubShotSpot s : HubShotSpot.values()) {
      if (s.isFixed()
          && DriveToPose.isWithinTolerance(
              pose, HubTargets.shootingPose(PoseFusion.defaultFieldLayout(), alliance.get(), s.angleDeg))) {
        return true;
      }
    }
    return false;
  }

  private static Command build(
      CommandSwerveDrivetrain drivetrain,
      AprilTagFieldLayout layout,
      DoubleSupplier secondsSinceVision,
      HubShotSpot spot) {
    SmartDashboard.putNumber(KEY + "replaneos", 0);
    SmartDashboard.putString(KEY + "posicion", spot.label);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return status("sin alianza: conecta el Driver Station / FMS");
    }
    double visionAge = secondsSinceVision.getAsDouble();
    if (REQUIRE_VISION_SINCE_BOOT && Double.isInfinite(visionAge)) {
      return status("la vision nunca ha visto un tag: la pose no es confiable, no se mueve");
    }

    Pose2d start = drivetrain.getState().Pose;
    HubShotSpot resolved =
        spot.isFixed() ? spot : HubTargets.nearestSpot(layout, alliance.get(), start).orElseThrow();
    Pose2d target = HubTargets.shootingPose(layout, alliance.get(), resolved.angleDeg);
    SmartDashboard.putString(
        KEY + "posicion", spot.isFixed() ? resolved.label : resolved.label + " (la mas cercana)");
    boolean close =
        start.getTranslation().getDistance(target.getTranslation()) < SKIP_PATHFIND_DISTANCE_M
            && Math.abs(target.getRotation().minus(start.getRotation()).getDegrees())
                < SKIP_PATHFIND_ANGLE_DEG;

    field().getObject("objetivo").setPose(target);
    SmartDashboard.putString(
        KEY + "objetivo",
        String.format(
            "%s (%.3f, %.3f) %.1f deg",
            alliance.get(), target.getX(), target.getY(), target.getRotation().getDegrees()));

    Command steps =
        Commands.sequence(
            status(close ? "cerca: solo ajuste fino" : "pathfinding"),
            close ? Commands.none() : pathfindWithReplanning(drivetrain, target),
            status("ajuste fino"),
            new DriveToPose(drivetrain, target).withTimeout(FINE_TIMEOUT_S),
            Commands.runOnce(
                () ->
                    SmartDashboard.putString(
                        KEY + "estado",
                        DriveToPose.isWithinTolerance(drivetrain.getState().Pose, target)
                            ? "alineado"
                            : "timeout: fuera de tolerancia")));

    // Telemetria en paralelo (sin requirements) mientras corren los pasos.
    return Commands.deadline(
            steps, Commands.run(() -> publish(drivetrain, target, secondsSinceVision)))
        .finallyDo(
            interrupted -> {
              publish(drivetrain, target, secondsSinceVision);
              if (interrupted) {
                SmartDashboard.putString(KEY + "estado", "cancelado");
              }
            });
  }

  /**
   * Pathfinding que se replanea cuando la vision corrige la pose mas de REPLAN_JUMP_M y todavia
   * falta mas de REPLAN_MIN_DISTANCE_M (mas cerca, el ajuste fino absorbe la correccion).
   */
  private static Command pathfindWithReplanning(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    PoseJumpDetector jumps = new PoseJumpDetector(drivetrain::getState);
    java.util.List<Pair<Translation2d, Translation2d>> hubs = inflatedHubs(PoseFusion.defaultFieldLayout());
    boolean[] arrived = {false};
    int[] replans = {0};

    Command onePlan =
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  jumps.reset();
                  // Crea el pathfinder si ningun comando de pathfinding lo ha hecho todavia (si no,
                  // setDynamicObstacles truena con NullPointerException en el primer uso).
                  Pathfinding.ensureInitialized();
                  Pathfinding.setDynamicObstacles(hubs, drivetrain.getState().Pose.getTranslation());
                }),
            drivetrain
                .getPoseFinder()
                .toPose(target, PATHFIND_CONSTRAINTS)
                .finallyDo(interrupted -> arrived[0] = !interrupted)
                .until(
                    () -> {
                      double corrected = jumps.update();
                      double remaining =
                          drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
                      boolean replan =
                          corrected > REPLAN_JUMP_M
                              && remaining > REPLAN_MIN_DISTANCE_M
                              && replans[0] < MAX_REPLANS;
                      if (replan) {
                        replans[0]++;
                        SmartDashboard.putNumber(KEY + "replaneos", replans[0]);
                        SmartDashboard.putString(
                            KEY + "estado",
                            String.format("la vision corrigio %.2f m: replaneando", corrected));
                      }
                      return replan;
                    }));

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              arrived[0] = false;
              replans[0] = 0;
            }),
        onePlan.repeatedly().until(() -> arrived[0]));
  }

  /** Los dos hubs (rectangulo de sus tags) inflados por media huella del robot mas el margen. */
  static java.util.List<Pair<Translation2d, Translation2d>> inflatedHubs(AprilTagFieldLayout layout) {
    double grow = ROBOT_HALF_SIZE_M + HUB_MARGIN_M;
    java.util.List<Pair<Translation2d, Translation2d>> boxes = new java.util.ArrayList<>();
    for (int[] ids : new int[][] {BLUE_HUB_TAGS, RED_HUB_TAGS}) {
      double x0 = Double.POSITIVE_INFINITY, y0 = x0, x1 = Double.NEGATIVE_INFINITY, y1 = x1;
      for (int id : ids) {
        var p = layout.getTagPose(id).orElseThrow().toPose2d();
        x0 = Math.min(x0, p.getX());
        x1 = Math.max(x1, p.getX());
        y0 = Math.min(y0, p.getY());
        y1 = Math.max(y1, p.getY());
      }
      boxes.add(Pair.of(new Translation2d(x0 - grow, y0 - grow), new Translation2d(x1 + grow, y1 + grow)));
    }
    return boxes;
  }

  private static Command status(String text) {
    return Commands.runOnce(() -> SmartDashboard.putString(KEY + "estado", text));
  }

  private static void publish(
      CommandSwerveDrivetrain drivetrain, Pose2d target, DoubleSupplier secondsSinceVision) {
    Pose2d pose = drivetrain.getState().Pose;
    field().setRobotPose(pose);
    publishSpots();
    double age = secondsSinceVision.getAsDouble();
    SmartDashboard.putNumber(KEY + "vision_s", Double.isInfinite(age) ? -1 : age);
    SmartDashboard.putString(
        KEY + "pose_con", age > STALE_VISION_WARN_S ? "odometria (sin tags a la vista)" : "vision + odometria");
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
