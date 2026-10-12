// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
}
