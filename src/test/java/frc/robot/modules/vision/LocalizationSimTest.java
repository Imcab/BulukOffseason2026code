// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.vision;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.configuration.constants.swerve.TunerConstants;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Prueba de punta a punta en simulacion: Limelight simulada -> NetworkTables -> PoseFusion ->
 * estimador del swerve de CTRE. La camara ve el mundo desde una pose verdadera fija y al estimador
 * se le mete un error a proposito para ver que la fusion lo corrige.
 */
class LocalizationSimTest {

  private static final double LOOP_SECONDS = 0.02;

  private static CommandSwerveDrivetrain drivetrain;
  private static Localization localization;
  private static Pose2d truth;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    truth = poseFacingNearestTag();
    drivetrain = TunerConstants.createDrivetrain();
    localization = new Localization(drivetrain, () -> truth);
  }

  @AfterAll
  static void tearDown() {
    SimulatedLimelight.closeAll();
    drivetrain.close();
  }

  /** Robot a 2.5 m de un tag del lado azul, mirandolo de frente (la camara apunta hacia adelante). */
  private static Pose2d poseFacingNearestTag() {
    AprilTag tag =
        PoseFusion.defaultFieldLayout().getTags().stream()
            .filter(t -> t.pose.getX() < 6.0)
            .filter(t -> Math.abs(t.pose.getRotation().getZ()) < 0.1) // mira hacia +X
            .findFirst()
            .orElseThrow();
    Translation2d tagXY = tag.pose.toPose2d().getTranslation();
    System.out.println("[test] tag " + tag.ID + " en " + tagXY);
    return new Pose2d(tagXY.plus(new Translation2d(2.5, 0.0)), Rotation2d.k180deg);
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** Corre el ciclo de la fusion y regresa cuantas mediciones se aceptaron. */
  private static int run(double seconds) throws InterruptedException {
    int accepted = 0;
    for (int i = 0; i < seconds / LOOP_SECONDS; i++) {
      localization.simulationPeriodic();
      NetworkTableInstance.getDefault().flushLocal();
      localization.periodic();
      accepted += localization.getAcceptedCount();
      Thread.sleep((long) (LOOP_SECONDS * 1000));
    }
    return accepted;
  }

  private static void report(String phase, int accepted) {
    Pose2d est = drivetrain.getState().Pose;
    System.out.printf(
        "[test] %s: %d aceptadas, error xy %.3f m, error heading %.2f deg%n",
        phase,
        accepted,
        est.getTranslation().getDistance(truth.getTranslation()),
        est.getRotation().minus(truth.getRotation()).getDegrees());
  }

  @Test
  void megaTag1WhileDisabledThenMegaTag2WhileEnabled() throws InterruptedException {
    // Fase 1: deshabilitado, heading 30 grados mal. MegaTag1 debe corregirlo.
    setEnabled(false);
    drivetrain.resetPose(
        new Pose2d(truth.getTranslation(), truth.getRotation().plus(Rotation2d.fromDegrees(30))));
    Thread.sleep(100);
    report("inicio fase 1", 0);
    int accepted1 = run(3.0);
    report("fin fase 1 (MegaTag1)", accepted1);

    Pose2d est = drivetrain.getState().Pose;
    assertTrue(accepted1 > 0, "MegaTag1 no acepto ninguna medicion");
    assertTrue(
        Math.abs(est.getRotation().minus(truth.getRotation()).getDegrees()) < 3.0,
        "MegaTag1 no corrigio el heading");

    // Fase 2: habilitado, heading bien pero 0.5 m corrido. MegaTag2 debe corregir x/y.
    setEnabled(true);
    drivetrain.resetPose(
        new Pose2d(truth.getTranslation().plus(new Translation2d(0.0, 0.5)), truth.getRotation()));
    Thread.sleep(100);
    report("inicio fase 2", 0);
    int accepted2 = run(3.0);
    report("fin fase 2 (MegaTag2)", accepted2);

    est = drivetrain.getState().Pose;
    assertTrue(accepted2 > 0, "MegaTag2 no acepto ninguna medicion");
    assertTrue(
        est.getTranslation().getDistance(truth.getTranslation()) < 0.10,
        "MegaTag2 no corrigio x/y");
    assertTrue(
        Math.abs(est.getRotation().minus(truth.getRotation()).getDegrees()) < 1.0,
        "MegaTag2 movio el heading");
  }
}
