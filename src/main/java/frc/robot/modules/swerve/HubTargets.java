// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static frc.robot.configuration.constants.swerve.HubAlignConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Pose de tiro frente al hub, calculada desde los AprilTags del layout (coordenadas de campo con
 * origen azul, las mismas que usan el estimador, PathPlanner y la Limelight).
 *
 * <pre>
 *   normal n  = direccion a la que miran los tags de la cara frontal (se aleja del hub)
 *   tangente  = n girada +90 grados
 *   cara      = centro del hub proyectado sobre el plano de la cara
 *   objetivo  = cara + n * SHOOT_DISTANCE_M + tangente * offset lateral
 *   heading   = n (la parte de atras del robot apunta al hub) + HEADING_OFFSET_DEG
 *
 *   Variantes angulares (HubShotSpot): mismo radio desde el centro del hub, girado el angulo
 *   desde n; el heading apunta lejos del centro del hub.
 * </pre>
 *
 * <p>Como sale de los tags de cada alianza, rojo y azul quedan simetricos sin voltear nada a mano.
 */
public final class HubTargets {

  private HubTargets() {}

  /** Posicion centrada (frente a la cara del hub). */
  public static Pose2d shootingPose(AprilTagFieldLayout layout, Alliance alliance) {
    return shootingPose(layout, alliance, 0.0);
  }

  /**
   * Posicion de tiro sobre el arco alrededor del hub, a la misma distancia del centro del hub que
   * la centrada, girada angleDeg desde la normal de la cara (positivo = izquierda del driver). El
   * heading apunta lejos del centro del hub: la parte de atras del robot queda hacia el hub.
   */
  public static Pose2d shootingPose(AprilTagFieldLayout layout, Alliance alliance, double angleDeg) {
    boolean red = alliance == Alliance.Red;
    int[] front = red ? RED_FRONT_TAGS : BLUE_FRONT_TAGS;
    int[] hub = red ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    Pose2d tagA = tag2d(layout, front[0]);
    Pose2d tagB = tag2d(layout, front[1]);
    Rotation2d normal = tagA.getRotation();
    Translation2d n = new Translation2d(1.0, normal);
    Translation2d t = new Translation2d(1.0, normal.plus(Rotation2d.kCCW_90deg));

    Translation2d faceMid = tagA.getTranslation().plus(tagB.getTranslation()).div(2.0);
    Translation2d hubCenter = hubCenter(layout, hub, n, t);

    // Distancia del centro del hub a su cara frontal, mas la distancia de tiro.
    double radius = dot(faceMid.minus(hubCenter), n) + SHOOT_DISTANCE_M;

    // Mirando desde su estacion, la izquierda del driver es la normal girada en sentido horario.
    Rotation2d direction = normal.minus(Rotation2d.fromDegrees(angleDeg));
    Translation2d onArc = hubCenter.plus(new Translation2d(radius, direction));

    // El offset lateral se mide hacia el primer tag de la lista (25 en azul, 9 en rojo).
    Translation2d facePoint = hubCenter.plus(n.times(dot(faceMid.minus(hubCenter), n)));
    double towardFirstTag = Math.signum(dot(tagA.getTranslation().minus(facePoint), t));
    if (towardFirstTag == 0) {
      towardFirstTag = 1.0;
    }

    Translation2d target = onArc.plus(t.times(towardFirstTag * LATERAL_OFFSET_M));
    return new Pose2d(target, direction.plus(Rotation2d.fromDegrees(HEADING_OFFSET_DEG)));
  }

  /** La pose de la variante pedida; para MAS_CERCANA, la variante fija mas cercana a {@code from}. */
  public static Pose2d shootingPose(
      AprilTagFieldLayout layout, Alliance alliance, HubShotSpot spot, Pose2d from) {
    if (spot.isFixed()) {
      return shootingPose(layout, alliance, spot.angleDeg);
    }
    return nearestSpot(layout, alliance, from).map(s -> shootingPose(layout, alliance, s.angleDeg)).orElseThrow();
  }

  /** La variante fija cuya pose esta mas cerca (en posicion) de {@code from}. */
  public static java.util.Optional<HubShotSpot> nearestSpot(
      AprilTagFieldLayout layout, Alliance alliance, Pose2d from) {
    HubShotSpot best = null;
    double bestDist = Double.POSITIVE_INFINITY;
    for (HubShotSpot s : HubShotSpot.values()) {
      if (!s.isFixed()) {
        continue;
      }
      double d = shootingPose(layout, alliance, s.angleDeg).getTranslation().getDistance(from.getTranslation());
      if (d < bestDist) {
        bestDist = d;
        best = s;
      }
    }
    return java.util.Optional.ofNullable(best);
  }

  /**
   * Centro geometrico del hub: punto medio entre caras opuestas (frontal/trasera a lo largo de n,
   * laterales a lo largo de t). No es el promedio de los tags: en cada cara un tag esta centrado y
   * el otro desplazado, y en las caras laterales esos desplazamientos no se cancelan.
   */
  static Translation2d hubCenter(AprilTagFieldLayout layout, int[] hub, Translation2d n, Translation2d t) {
    return n.times(midPlane(layout, hub, n)).plus(t.times(midPlane(layout, hub, t)));
  }

  /** Posicion (a lo largo de axis) del plano medio entre las caras que miran a +axis y a -axis. */
  private static double midPlane(AprilTagFieldLayout layout, int[] hub, Translation2d axis) {
    double plus = 0, minus = 0;
    int np = 0, nm = 0;
    for (int id : hub) {
      Pose2d tag = tag2d(layout, id);
      double facing = dot(new Translation2d(1.0, tag.getRotation()), axis);
      double offset = dot(tag.getTranslation(), axis);
      if (facing > 0.9) {
        plus += offset;
        np++;
      } else if (facing < -0.9) {
        minus += offset;
        nm++;
      }
    }
    if (np == 0 || nm == 0) {
      throw new IllegalStateException("El hub necesita tags en caras opuestas para calcular su centro");
    }
    return (plus / np + minus / nm) / 2.0;
  }

  private static Pose2d tag2d(AprilTagFieldLayout layout, int id) {
    return layout
        .getTagPose(id)
        .orElseThrow(() -> new IllegalStateException("El layout no tiene el tag " + id))
        .toPose2d();
  }

  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }
}
