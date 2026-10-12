// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.configuration.constants.swerve.HubAlignConstants;
import org.junit.jupiter.api.Test;

/** Geometria de la posicion de tiro: sin simulacion, solo el layout de AprilTags 2026. */
class HubTargetsTest {

  private static final AprilTagFieldLayout LAYOUT = PoseFusion.defaultFieldLayout();
  private static final double EPS = 1e-3;

  @Test
  void blueTargetIsInFrontOfTags25And26FacingAway() {
    Pose2d t = HubTargets.shootingPose(LAYOUT, Alliance.Blue);
    Pose2d tag26 = LAYOUT.getTagPose(26).orElseThrow().toPose2d();
    System.out.printf("[test] azul: (%.4f, %.4f) %.1f deg%n", t.getX(), t.getY(), t.getRotation().getDegrees());
    // 2.54 m frente a la cara (los tags 25/26 miran hacia -X, al muro azul)
    assertEquals(tag26.getX() - HubAlignConstants.SHOOT_DISTANCE_M, t.getX(), EPS);
    // centrado en el eje del hub (el tag 26 esta justo en el centro del hub)
    assertEquals(tag26.getY(), t.getY(), EPS);
    // mirando hacia atras: 180 grados en coordenadas de campo
    assertEquals(180.0, Math.abs(t.getRotation().getDegrees()), 0.01);
  }

  @Test
  void redTargetIsTheRotationOfBlue() {
    Pose2d blue = HubTargets.shootingPose(LAYOUT, Alliance.Blue);
    Pose2d red = HubTargets.shootingPose(LAYOUT, Alliance.Red);
    System.out.printf("[test] rojo: (%.4f, %.4f) %.1f deg%n", red.getX(), red.getY(), red.getRotation().getDegrees());
    double lx = LAYOUT.getFieldLength();
    double ly = LAYOUT.getFieldWidth();
    // El campo 2026 es simetrico por rotacion de 180 grados alrededor del centro.
    assertEquals(lx - blue.getX(), red.getX(), 0.01);
    assertEquals(ly - blue.getY(), red.getY(), 0.01);
    assertEquals(0.0, red.getRotation().getDegrees(), 0.01);
  }

  @Test
  void redTargetIsInFrontOfTags9And10() {
    Pose2d red = HubTargets.shootingPose(LAYOUT, Alliance.Red);
    Pose2d tag10 = LAYOUT.getTagPose(10).orElseThrow().toPose2d();
    assertEquals(tag10.getX() + HubAlignConstants.SHOOT_DISTANCE_M, red.getX(), EPS);
    assertEquals(tag10.getY(), red.getY(), EPS);
  }

  @Test
  void distanceToFaceIsExactlyTheShootingDistance() {
    for (Alliance a : Alliance.values()) {
      Pose2d t = HubTargets.shootingPose(LAYOUT, a);
      int faceTag = a == Alliance.Red ? 9 : 25;
      Pose2d tag = LAYOUT.getTagPose(faceTag).orElseThrow().toPose2d();
      Translation2d n = new Translation2d(1.0, tag.getRotation());
      Translation2d d = t.getTranslation().minus(tag.getTranslation());
      double alongNormal = d.getX() * n.getX() + d.getY() * n.getY();
      assertEquals(HubAlignConstants.SHOOT_DISTANCE_M, alongNormal, EPS, a.toString());
    }
  }

  /** Centro del hub medido directo del layout: punto medio entre caras opuestas. */
  private static Translation2d hubCenter(Alliance a) {
    // Azul: frontal 26 (x 4.02) / trasera 20 (x 5.23); laterales 18 (y 3.43) / 21 (y 4.64).
    // Rojo: frontal 10 / trasera 4; laterales 5 / 11.
    int[] ids = a == Alliance.Red ? new int[] {10, 4, 5, 11} : new int[] {26, 20, 18, 21};
    double x = (tagXY(ids[0]).getX() + tagXY(ids[1]).getX()) / 2;
    double y = (tagXY(ids[2]).getY() + tagXY(ids[3]).getY()) / 2;
    return new Translation2d(x, y);
  }

  private static Translation2d tagXY(int id) {
    return LAYOUT.getTagPose(id).orElseThrow().toPose2d().getTranslation();
  }

  @Test
  void allSpotsAreAtTheSameDistanceFromTheHubAndFaceAwayFromIt() {
    for (Alliance a : Alliance.values()) {
      Translation2d c = hubCenter(a);
      double r0 = HubTargets.shootingPose(LAYOUT, a).getTranslation().getDistance(c);
      for (HubShotSpot spot : HubShotSpot.values()) {
        if (!spot.isFixed()) continue;
        Pose2d p = HubTargets.shootingPose(LAYOUT, a, spot.angleDeg);
        System.out.printf("[test] %s %-13s (%.3f, %.3f) %.1f deg%n", a, spot.label, p.getX(), p.getY(), p.getRotation().getDegrees());
        assertEquals(r0, p.getTranslation().getDistance(c), EPS, a + " " + spot);
        // De espaldas al hub: el heading apunta del centro del hub hacia el robot.
        double away = Math.toDegrees(Math.atan2(p.getY() - c.getY(), p.getX() - c.getX()));
        assertEquals(0.0, p.getRotation().minus(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(away)).getDegrees(), 0.01);
      }
      assertEquals(3.1437, r0, 0.001); // 0.6037 (centro del hub a la cara) + 2.54
    }
  }

  @Test
  void leftAndRightAreAsTheDriverSeesThem() {
    // Azul: el driver mira hacia +X, su izquierda es +Y. Rojo: mira hacia -X, su izquierda es -Y.
    double blueCenterY = HubTargets.shootingPose(LAYOUT, Alliance.Blue).getY();
    assertTrue(HubTargets.shootingPose(LAYOUT, Alliance.Blue, HubShotSpot.IZQUIERDA_30.angleDeg).getY() > blueCenterY);
    assertTrue(HubTargets.shootingPose(LAYOUT, Alliance.Blue, HubShotSpot.DERECHA_30.angleDeg).getY() < blueCenterY);
    double redCenterY = HubTargets.shootingPose(LAYOUT, Alliance.Red).getY();
    assertTrue(HubTargets.shootingPose(LAYOUT, Alliance.Red, HubShotSpot.IZQUIERDA_30.angleDeg).getY() < redCenterY);
    assertTrue(HubTargets.shootingPose(LAYOUT, Alliance.Red, HubShotSpot.DERECHA_30.angleDeg).getY() > redCenterY);
  }

  @Test
  void redSpotsAreTheRotationOfBlueSpots() {
    for (HubShotSpot spot : HubShotSpot.values()) {
      if (!spot.isFixed()) continue;
      Pose2d b = HubTargets.shootingPose(LAYOUT, Alliance.Blue, spot.angleDeg);
      Pose2d r = HubTargets.shootingPose(LAYOUT, Alliance.Red, spot.angleDeg);
      assertEquals(LAYOUT.getFieldLength() - b.getX(), r.getX(), 0.01, spot.toString());
      assertEquals(LAYOUT.getFieldWidth() - b.getY(), r.getY(), 0.01, spot.toString());
      assertEquals(0.0, r.getRotation().minus(b.getRotation().plus(edu.wpi.first.math.geometry.Rotation2d.k180deg)).getDegrees(), 0.01);
    }
  }

  @Test
  void nearestPicksTheClosestSpot() {
    for (Alliance a : Alliance.values()) {
      for (HubShotSpot spot : HubShotSpot.values()) {
        if (!spot.isFixed()) continue;
        Pose2d p = HubTargets.shootingPose(LAYOUT, a, spot.angleDeg);
        Pose2d near = new Pose2d(p.getTranslation().plus(new Translation2d(0.2, -0.1)), p.getRotation());
        assertEquals(spot, HubTargets.nearestSpot(LAYOUT, a, near).orElseThrow(), a + " " + spot);
        assertEquals(p, HubTargets.shootingPose(LAYOUT, a, HubShotSpot.MAS_CERCANA, near));
      }
    }
  }
}
