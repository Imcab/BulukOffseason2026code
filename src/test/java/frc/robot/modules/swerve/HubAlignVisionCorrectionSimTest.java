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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.constants.swerve.TunerConstants;
import frc.robot.modules.vision.Localization;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Vision + odometria combinadas: el robot arranca creyendo estar 0.72 m de donde realmente esta y
 * sin ver ningun tag. La Limelight simulada ve el mundo desde la pose REAL (odometria de las ruedas
 * desde la pose real de inicio, sin vision). En el camino empieza a ver tags, la pose se corrige y
 * la ruta se replanea. Pasa solo si el robot REAL termina en la posicion de tiro.
 */
class HubAlignVisionCorrectionSimTest {

  private static final double LOOP_S = 0.02;

  private static CommandSwerveDrivetrain drivetrain;
  private static Localization localization;
  private static SwerveDriveOdometry truth;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    drivetrain = TunerConstants.createDrivetrain();
    HubAlignReplay.installPathLogger();
    localization = new Localization(drivetrain, HubAlignVisionCorrectionSimTest::truePose);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @AfterAll
  static void tearDown() {
    HubAlignReplay.write("HubAlignVisionCorrectionSimTest");
    CommandScheduler.getInstance().cancelAll();
    SimulatedLimelight.closeAll();
    drivetrain.close();
  }

  /** Pose real: odometria de las ruedas (perfecta en simulacion) desde la pose real de inicio. */
  private static Pose2d truePose() {
    if (truth == null) {
      return drivetrain.getState().Pose;
    }
    var s = drivetrain.getState();
    // El heading no tiene error (MegaTag2 no lo toca y el giroscopio es perfecto en simulacion).
    return truth.update(s.Pose.getRotation(), s.ModulePositions);
  }

  private static void loop(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / LOOP_S; i++) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      if (truth != null) {
        HubAlignReplay.sample(drivetrain.getState().Pose, truePose());
      }
      Thread.sleep((long) (LOOP_S * 1000));
    }
  }

  @Test
  void visionCorrectsThePoseAndThePathWhileDriving() throws InterruptedException {
    // Real: (2.8, 2.0) mirando hacia la pared de abajo (270 grados): la camara no ve ningun tag.
    Pose2d realStart = new Pose2d(2.8, 2.0, Rotation2d.fromDegrees(270));
    // Lo que cree el robot: 0.72 m corrido (odometria acumulada mal antes de presionar el boton).
    Pose2d believed = new Pose2d(realStart.getTranslation().plus(new Translation2d(0.6, -0.4)), realStart.getRotation());

    drivetrain.resetPose(believed);
    Thread.sleep(100);
    truth =
        new SwerveDriveOdometry(
            drivetrain.getKinematics(), realStart.getRotation(), drivetrain.getState().ModulePositions, realStart);
    loop(0.3);
    double startErr = drivetrain.getState().Pose.getTranslation().getDistance(truePose().getTranslation());
    System.out.printf("[test] al presionar: error de pose %.2f m, vision hace %.1f s%n",
        startErr, localization.secondsSinceLastVision());
    assertTrue(startErr > 0.5, "la vision ya habia corregido antes de empezar: el caso no prueba nada");

    Pose2d target = HubAlign.currentTarget().orElseThrow();
    // "Vio un tag hace 30 s": la pose es la de la odometria (mal) hasta que vuelva a ver uno.
    HubAlignReplay.begin("Azul, la vision corrige 0.72 m en el camino", edu.wpi.first.wpilibj.DriverStation.Alliance.Blue,
        "Centro", realStart, target);
    Command cmd = HubAlign.command(drivetrain, () -> Math.min(30.0, localization.secondsSinceLastVision()));
    CommandScheduler.getInstance().schedule(cmd);
    long t0 = System.nanoTime();
    while (CommandScheduler.getInstance().isScheduled(cmd) && (System.nanoTime() - t0) / 1e9 < 15) {
      loop(LOOP_S);
    }
    double seconds = (System.nanoTime() - t0) / 1e9;
    loop(0.3);

    Pose2d real = truePose();
    Pose2d est = drivetrain.getState().Pose;
    double realErrCm = 100 * real.getTranslation().getDistance(target.getTranslation());
    double estErrCm = 100 * est.getTranslation().getDistance(real.getTranslation());
    double replans = SmartDashboard.getNumber("HubAlign/replaneos", -1);
    System.out.printf(
        "[test] %.2f s | robot REAL a %.1f cm del objetivo | estimador vs real %.1f cm | replaneos %.0f | estado '%s'%n",
        seconds, realErrCm, estErrCm, replans, SmartDashboard.getString("HubAlign/estado", "?"));

    HubAlignReplay.end(seconds < 15 && estErrCm < 8.0 && realErrCm < 8.0, seconds, realErrCm, 0,
        "El robot cree estar 0.72 m corrido (linea punteada) y no ve tags. En el camino la Limelight ve tags, corrige la pose y se replanea; el error es el del robot REAL.");
    assertTrue(seconds < 15, "no termino");
    assertTrue(estErrCm < 8.0, "la vision no corrigio la pose: " + estErrCm + " cm");
    assertTrue(realErrCm < 8.0, "el robot real no llego a la posicion de tiro: " + realErrCm + " cm");
    assertTrue(replans >= 1, "la vision corrigio 0.72 m pero no se replaneo la ruta");
  }
}
