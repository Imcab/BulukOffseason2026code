// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.constants.swerve.TunerConstants;
import frc.robot.modules.vision.Localization;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * HubAlign de punta a punta en simulacion: swerve de CTRE simulado + Limelight simulada (fusion
 * MegaTag2) + PoseFinder/pathfinding de PathPlanner con el navgrid real + ajuste fino. Corre en
 * tiempo real (el simulador de CTRE avanza con el reloj), asi que tarda unos segundos por caso.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class HubAlignSimTest {

  private static final double LOOP_S = 0.02;
  private static final double MAX_RUN_S = 12.0;

  private static CommandSwerveDrivetrain drivetrain;
  private static Localization localization;
  private static final java.util.List<Pose2d> trace = new java.util.ArrayList<>();

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    drivetrain = TunerConstants.createDrivetrain();
    localization = new Localization(drivetrain); // la camara simulada ve el mundo desde la pose estimada
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    SimulatedLimelight.closeAll();
    drivetrain.close();
  }

  private static void setAlliance(AllianceStationID id) {
    DriverStationSim.setAllianceStationId(id);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void loop(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / LOOP_S; i++) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      trace.add(drivetrain.getState().Pose);
      Thread.sleep((long) (LOOP_S * 1000));
    }
  }

  private static double[] moduleDistances() {
    var positions = drivetrain.getState().ModulePositions;
    double[] d = new double[positions.length];
    for (int i = 0; i < d.length; i++) {
      d[i] = positions[i].distanceMeters;
    }
    return d;
  }

  /** Corre el comando hasta que termine (o MAX_RUN_S). Devuelve los segundos que tardo. */
  private static double runUntilDone(Command cmd) throws InterruptedException {
    CommandScheduler.getInstance().schedule(cmd);
    long t0 = System.nanoTime();
    while (CommandScheduler.getInstance().isScheduled(cmd)
        && (System.nanoTime() - t0) / 1e9 < MAX_RUN_S) {
      loop(LOOP_S);
    }
    return (System.nanoTime() - t0) / 1e9;
  }

  private static void alignFrom(AllianceStationID station, Pose2d start) throws InterruptedException {
    setAlliance(station);
    drivetrain.resetPose(start);
    loop(1.0); // deja que la vision acepte mediciones (MegaTag2, robot habilitado)

    Pose2d target = HubAlign.currentTarget().orElseThrow();
    trace.clear();
    double seconds =
        runUntilDone(HubAlign.command(drivetrain, localization::secondsSinceLastVision));
    loop(0.3);
    Pose2d end = drivetrain.getState().Pose;
    double errCm = 100 * end.getTranslation().getDistance(target.getTranslation());
    double errDeg = Math.abs(target.getRotation().minus(end.getRotation()).getDegrees());
    System.out.printf(
        "[test] %s desde %s -> %s en %.2f s | objetivo %s | error %.1f cm, %.2f deg | estado: %s%n",
        station, start, end, seconds, target, errCm, errDeg,
        SmartDashboard.getString("HubAlign/estado", "?"));

    assertTrue(seconds < MAX_RUN_S, "HubAlign no termino");
    assertTrue(errCm < 5.0, "error de posicion " + errCm + " cm");
    assertTrue(errDeg < 3.0, "error de heading " + errDeg + " deg");
    assertTrue(HubAlign.isAtShootingPose(drivetrain) || errCm < 5.0);
  }

  @Test
  @Order(1)
  void blueFromTheSide() throws InterruptedException {
    alignFrom(AllianceStationID.Blue1, new Pose2d(2.8, 2.0, Rotation2d.fromDegrees(30)));
  }

  @Test
  @Order(2)
  void redFromTheSide() throws InterruptedException {
    // Misma situacion que en azul, rotada 180 grados alrededor del centro del campo.
    alignFrom(AllianceStationID.Red1, new Pose2d(16.541 - 2.8, 8.069 - 2.0, Rotation2d.fromDegrees(210)));
  }

  @Test
  @Order(3)
  void blueFromAroundTheHub() throws InterruptedException {
    // Del otro lado del hub: el pathfinding tiene que rodearlo (no atravesarlo).
    alignFrom(AllianceStationID.Blue1, new Pose2d(6.8, 4.0, Rotation2d.fromDegrees(0)));
    // Hub azul fisico: x 4.02-5.23, y 3.43-4.64. Con media huella del robot (0.45 m) el centro no
    // debe entrar a x 3.57-5.68, y 2.98-5.09.
    double minClear = Double.POSITIVE_INFINITY;
    for (Pose2d p : trace) {
      double dx = Math.max(Math.max(4.0219 - p.getX(), p.getX() - 5.2292), 0);
      double dy = Math.max(Math.max(3.4312 - p.getY(), p.getY() - 4.6380), 0);
      minClear = Math.min(minClear, Math.hypot(dx, dy));
    }
    System.out.printf("[test] distancia minima del centro del robot al hub: %.2f m (%d poses)%n", minClear, trace.size());
    assertTrue(minClear > 0.45, "el robot paso por encima del hub: " + minClear + " m");
  }

  @Test
  @Order(4)
  void doesNotMoveIfVisionNeverSawATag() throws InterruptedException {
    setAlliance(AllianceStationID.Blue1);
    drivetrain.resetPose(new Pose2d(2.8, 2.0, Rotation2d.fromDegrees(30)));
    loop(0.5);
    // Se mide con las ruedas (cuanto rodo cada modulo), no con la pose estimada: despues de un
    // resetPose pueden llegar mediciones viejas de la Limelight simulada que mueven el estimador
    // sin que el robot se mueva.
    double[] before = moduleDistances();
    runUntilDone(HubAlign.command(drivetrain, () -> Double.POSITIVE_INFINITY));
    loop(0.5);
    double[] after = moduleDistances();
    double rolled = 0;
    for (int i = 0; i < before.length; i++) {
      rolled = Math.max(rolled, Math.abs(after[i] - before[i]));
    }
    System.out.printf("[test] vision nunca: estado '%s', las ruedas rodaron %.3f m%n",
        SmartDashboard.getString("HubAlign/estado", "?"), rolled);
    assertTrue(rolled < 0.02, "se movio sin pose confiable: " + rolled + " m");
    assertTrue(SmartDashboard.getString("HubAlign/estado", "").startsWith("la vision nunca"));
  }

  @Test
  @Order(5)
  void keepsGoingWithOdometryWhenTagsAreNotVisible() throws InterruptedException {
    // La vision vio un tag hace 30 s y ya no ve nada: avanza con la odometria.
    setAlliance(AllianceStationID.Blue1);
    Pose2d start = new Pose2d(6.8, 1.5, Rotation2d.fromDegrees(90));
    drivetrain.resetPose(start);
    loop(0.3);
    Pose2d target = HubAlign.currentTarget().orElseThrow();
    double seconds = runUntilDone(HubAlign.command(drivetrain, () -> 30.0));
    loop(0.3);
    Pose2d end = drivetrain.getState().Pose;
    double errCm = 100 * end.getTranslation().getDistance(target.getTranslation());
    System.out.printf("[test] solo odometria: %.2f s, error %.1f cm, estado '%s', pose con '%s'%n",
        seconds, errCm, SmartDashboard.getString("HubAlign/estado", "?"),
        SmartDashboard.getString("HubAlign/pose_con", "?"));
    assertTrue(errCm < 5.0, "no se alineo con odometria: " + errCm + " cm");
    assertTrue(SmartDashboard.getString("HubAlign/pose_con", "").startsWith("odometria"));
  }
}
