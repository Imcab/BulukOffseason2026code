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
 * </pre>
 *
 * <p>Como sale de los tags de cada alianza, rojo y azul quedan simetricos sin voltear nada a mano.
 */
public final class HubTargets {

  private HubTargets() {}

  public static Pose2d shootingPose(AprilTagFieldLayout layout, Alliance alliance) {
    boolean red = alliance == Alliance.Red;
    int[] front = red ? RED_FRONT_TAGS : BLUE_FRONT_TAGS;
    int[] hub = red ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    Pose2d tagA = tag2d(layout, front[0]);
    Pose2d tagB = tag2d(layout, front[1]);
    Rotation2d normal = tagA.getRotation();
    Translation2d n = new Translation2d(1.0, normal);
    Translation2d t = new Translation2d(1.0, normal.plus(Rotation2d.kCCW_90deg));

    Translation2d faceMid = tagA.getTranslation().plus(tagB.getTranslation()).div(2.0);
    Translation2d hubCenter = Translation2d.kZero;
    for (int id : hub) {
      hubCenter = hubCenter.plus(tag2d(layout, id).getTranslation());
    }
    hubCenter = hubCenter.div(hub.length);

    // Centro del hub llevado al plano de la cara (a lo largo de la normal).
    Translation2d facePoint = hubCenter.plus(n.times(dot(faceMid.minus(hubCenter), n)));

    // El offset lateral se mide hacia el primer tag de la lista (25 en azul, 9 en rojo).
    double towardFirstTag = Math.signum(dot(tagA.getTranslation().minus(facePoint), t));
    if (towardFirstTag == 0) {
      towardFirstTag = 1.0;
    }

    Translation2d target =
        facePoint.plus(n.times(SHOOT_DISTANCE_M)).plus(t.times(towardFirstTag * LATERAL_OFFSET_M));
    return new Pose2d(target, normal.plus(Rotation2d.fromDegrees(HEADING_OFFSET_DEG)));
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
