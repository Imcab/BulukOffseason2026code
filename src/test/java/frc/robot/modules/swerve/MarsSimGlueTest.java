package frc.robot.modules.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.stzteam.forgemini.io.NetworkIO;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.configuration.constants.swerve.TunerConstants;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * El glue de MARS con la bandera puesta: sin MARS corriendo cae a la simulacion de CTRE, y con
 * un "bridge" que publica latido y encoders, la odometria sale de esos encoders.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class MarsSimGlueTest {

  private static CommandSwerveDrivetrain drivetrain;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    System.setProperty("mars.sim", "true"); // lo mismo que -Pmarssim
    drivetrain = TunerConstants.createDrivetrain();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.close();
    System.clearProperty("mars.sim");
  }

  private static void loop(double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / 0.02; i++) {
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      Thread.sleep(20);
    }
  }

  @Test
  @Order(1)
  void withoutMarsRunningTheRobotStillDrivesWithCtre() throws InterruptedException {
    assertTrue(MarsSimGlue.habilitado());
    assertTrue(NetworkIO.get("MARS/Glue/diag", "creado", false), "el glue no se construyo");
    drivetrain.setControl(SwerveRequestFactory.simpleDriveRequest().withVelocityX(1.0));
    loop(1.5);
    double x = drivetrain.getState().Pose.getX();
    assertTrue(x > 0.5, "sin MARS el robot debia moverse con la sim de CTRE, x = " + x);
    assertTrue(NetworkIO.get("MARS/Glue/diag", "ticks", 0.0) > 100, "el hilo del glue no corre");
  }

  @Test
  @Order(2)
  void withABridgePublishingTheOdometryComesFromItsEncoders() throws InterruptedException {
    drivetrain.setControl(SwerveRequestFactory.simpleDriveRequest().withVelocityX(0.0));
    loop(0.5);
    // "Bridge" falso: latido a 250 Hz y la rueda de cada modulo en una posicion fija.
    final double driveMotorRad = 40.0; // radianes del eje del motor de traccion
    final boolean[] run = {true};
    Thread bridge =
        new Thread(
            () -> {
              double beat = 0;
              while (run[0]) {
                NetworkIO.set("MARS/Glue/in", "heartbeat", ++beat);
                for (String m : new String[] {"fl", "fr", "bl", "br"}) {
                  NetworkIO.set("MARS/Glue/in", m + "_drive/position", driveMotorRad);
                  NetworkIO.set("MARS/Glue/in", m + "_drive/velocity", 0.0);
                  NetworkIO.set("MARS/Glue/in", m + "_steer/position", 0.0);
                  NetworkIO.set("MARS/Glue/in", m + "_steer/velocity", 0.0);
                }
                try {
                  Thread.sleep(4);
                } catch (InterruptedException e) {
                  return;
                }
              }
            });
    bridge.start();
    try {
      loop(1.0);
      assertTrue(NetworkIO.get("MARS/Glue/diag", "conectado", false), "el glue no detecto el latido");
      double expected =
          driveMotorRad
              / (2 * Math.PI)
              / TunerConstants.FrontLeft.DriveMotorGearRatio
              * 2
              * Math.PI
              * TunerConstants.FrontLeft.WheelRadius;
      double got = drivetrain.getState().ModulePositions[0].distanceMeters;
      assertEquals(expected, got, 0.01, "la distancia del modulo debia salir del encoder de Gazebo");
    } finally {
      run[0] = false;
      bridge.join();
    }
    // Sin latido, el glue vuelve solo a la sim de CTRE.
    loop(1.0);
    assertTrue(!NetworkIO.get("MARS/Glue/diag", "conectado", true), "el glue no solto la simulacion");
  }
}
