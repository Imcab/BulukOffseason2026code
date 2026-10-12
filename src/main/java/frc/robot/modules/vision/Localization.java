// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.stzteam.features.robotlocalization.fusion.LimelightPoseSource;
import com.stzteam.features.robotlocalization.fusion.LocalizationInputs;
import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import com.stzteam.features.robotlocalization.fusion.PoseSourceConfig;
import com.stzteam.features.robotlocalization.fusion.StdDevStrategies;
import com.stzteam.features.robotlocalization.vision.sim.LimelightSimConfig;
import com.stzteam.features.robotlocalization.vision.sim.SimulatedLimelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

import java.util.function.Supplier;

import static frc.robot.configuration.constants.vision.VisionConstants.*;

/**
 * Fusiona la Limelight (MegaTag2) con la odometria del swerve usando PoseFusion de
 * RobotLocalization. PoseFusion filtra las mediciones (entorno, timestamp, cancha, ambiguedad,
 * consenso y Mahalanobis) y las que sobreviven llegan a addVisionMeasurement con desviaciones
 * estandar calculadas cada ciclo segun distancia, numero de tags y velocidad del chasis.
 *
 * <p>MegaTag2 confia ciegamente en el heading que le damos, asi que si el heading esta mal la pose
 * sale mal. Por eso, con el robot deshabilitado se usa MegaTag1, que resuelve la pose (heading
 * incluido) solo con lo que ve, y corrige el heading del estimador antes del match. Ya habilitado,
 * solo corre MegaTag2. Nunca corren juntas: con el heading mal se contradirian y el filtro de
 * consenso tiraria las dos.
 *
 * <p>Telemetria en NetworkTables bajo {@link KeyManager#LIMELIGHT_KEY}/MegaTag1 y /MegaTag2: por que
 * se rechazo cada medicion y cuantas se aceptaron.
 */
public class Localization extends SubsystemBase {

  private final CommandSwerveDrivetrain drivetrain;
  // Dos fusiones porque PoseFusion indexa las fuentes por nombre y las dos leen la misma tabla.
  private final PoseFusion megaTag1Fusion;
  private final PoseFusion megaTag2Fusion;
  private final LimelightPoseSource megaTag1;
  private final LimelightPoseSource megaTag2;

  // En el robot real el constructor no crea nada y periodic() regresa de inmediato.
  private final SimulatedLimelight simLimelight;
  private final Supplier<Pose2d> simTruthPose;

  public Localization(CommandSwerveDrivetrain drivetrain) {
    this(drivetrain, () -> drivetrain.getState().Pose);
  }

  /**
   * @param simTruthPose pose real del robot para la camara simulada. Con la pose del estimador (el
   *     default) la vision siempre coincide con la odometria y no corrige nada; los tests le pasan
   *     una pose verdadera distinta para ver que la fusion si corrige.
   */
  public Localization(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> simTruthPose) {
    this.drivetrain = drivetrain;
    this.simTruthPose = simTruthPose;

    LocalizationInputs inputs =
        new LocalizationInputs(
            // MegaTag2 necesita el heading de campo; la fuente se lo escribe a la Limelight solita.
            () -> drivetrain.getState().Pose.getRotation(),
            () -> RadiansPerSecond.of(drivetrain.getState().Speeds.omegaRadiansPerSecond),
            drivetrain::getChassisSpeeds,
            // Pigeon2 reporta en G, que es lo que espera el filtro de aceleracion.
            () ->
                VecBuilder.fill(
                    drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(),
                    drivetrain.getPigeon2().getAccelerationY().getValueAsDouble()),
            () -> drivetrain.getState().Pose);

    this.megaTag2 =
        LimelightPoseSource.megaTag2(LIMELIGHT_NAME)
            .withFieldMargin(FIELD_MARGIN_METERS)
            .withStdDevsStrategy(
                // Siempre acotar: una sigma desbocada rompe la inversion de la matriz del Kalman.
                // El heading de MegaTag2 es el mismo que le damos, asi que su sigma se fija enorme.
                StdDevStrategies.clamped(
                    StdDevStrategies.dynamicStateDegradation(
                        StdDevStrategies.quadraticContinuous(
                            BASE_STD_XY, LimelightPoseSource.IGNORE_HEADING_STD_DEV),
                        drivetrain::getChassisSpeeds,
                        MAX_SAFE_LINEAR_MPS,
                        MAX_SAFE_OMEGA_RADS),
                    MIN_STD_XY,
                    MAX_STD_XY,
                    LimelightPoseSource.IGNORE_HEADING_STD_DEV,
                    LimelightPoseSource.IGNORE_HEADING_STD_DEV));

    this.megaTag1 =
        new LimelightPoseSource(
                LIMELIGHT_NAME,
                LimelightPoseSource.Mode.MEGATAG_1,
                // Sin Mahalanobis: su trabajo es justo corregir un heading que puede estar muy lejos
                // de lo que cree el estimador, y ese filtro lo tiraria por "demasiado distinto".
                PoseSourceConfig.forAprilTags()
                    .withMahalanobisGate(Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0))
            .withFieldMargin(FIELD_MARGIN_METERS)
            .withStdDevsStrategy(
                StdDevStrategies.clamped(
                    StdDevStrategies.quadraticContinuous(MT1_BASE_STD_XY, MT1_BASE_STD_THETA),
                    MIN_STD_XY,
                    MAX_STD_XY,
                    MT1_MIN_STD_THETA,
                    MT1_MAX_STD_THETA));

    this.megaTag1Fusion =
        new PoseFusion(drivetrain::addVisionMeasurement, inputs)
            .setTelemetryTable(KeyManager.LIMELIGHT_KEY + "/MegaTag1")
            .registerSource(megaTag1);

    this.megaTag2Fusion =
        new PoseFusion(drivetrain::addVisionMeasurement, inputs)
            .setTelemetryTable(KeyManager.LIMELIGHT_KEY + "/MegaTag2")
            .registerSource(megaTag2);

    this.simLimelight =
        new SimulatedLimelight(LIMELIGHT_NAME, ROBOT_TO_LIMELIGHT, LimelightSimConfig.limelight3G());

    if (RobotBase.isReal()) {
      pushCameraPose(ROBOT_TO_LIMELIGHT);
    }
  }

  /**
   * Escribe la pose de la camara a la Limelight para que coincida con la constante. Limelight usa
   * pitch positivo hacia arriba; WPILib, hacia abajo.
   */
  private static void pushCameraPose(Transform3d robotToCamera) {
    LimelightHelpers.setCameraPose_RobotSpace(
        LIMELIGHT_NAME,
        robotToCamera.getX(),
        robotToCamera.getY(),
        robotToCamera.getZ(),
        Units.radiansToDegrees(robotToCamera.getRotation().getX()),
        -Units.radiansToDegrees(robotToCamera.getRotation().getY()),
        Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
  }

  /** @return cuantas mediciones de vision se aceptaron en el ultimo ciclo. */
  public int getAcceptedCount() {
    return megaTag1Fusion.getAcceptedCount() + megaTag2Fusion.getAcceptedCount();
  }

  @Override
  public void periodic() {
    boolean disabled = DriverStation.isDisabled();
    megaTag1.setEnabled(disabled);
    megaTag2.setEnabled(!disabled);

    megaTag1Fusion.update();
    megaTag2Fusion.update();
  }

  @Override
  public void simulationPeriodic() {
    simLimelight.periodic(simTruthPose.get());
  }
}
