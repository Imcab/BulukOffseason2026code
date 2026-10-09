// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.stzteam.features.dictionary.Dictionary.CommonTables;
import com.stzteam.forgemini.io.NetworkIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.configuration.KeyManager;

public class SwerveTelemetry {

  /* Robot swerve drive state */
  private static final String TABLE_NAME = KeyManager.SWERVE_KEY;

  /* Robot pose for field positioning */
  private static final String POSITIONING_KEY = "Pose";

  /* NetworkTables keys, computed once instead of every loop */
  private static final String SPEEDS_KEY = CommonTables.sPluralOf(CommonTables.SPEED_KEY);
  private static final String MODULE_STATES_KEY =
      CommonTables.sPluralOf(CommonTables.MODULE_KEY + CommonTables.STATE_KEY);
  private static final String MODULE_TARGETS_KEY =
      CommonTables.sPluralOf(CommonTables.MODULE_KEY + CommonTables.TARGET_KEY);
  private static final String MODULE_POSITIONS_KEY =
      CommonTables.sPluralOf(CommonTables.MODULE_KEY + CommonTables.POSITION_KEY);
  private static final String ODOMETRY_FREQUENCY_KEY =
      CommonTables.ODOMETRY_KEY + CommonTables.FREQUENCY_KEY;
  private static final String ROBOT_POSE_KEY = CommonTables.ROBOT_KEY + CommonTables.POSE_KEY;

  /* SignalLogger keys */
  private static final String LOG_POSE = "DriveState/Pose";
  private static final String LOG_SPEEDS = "DriveState/Speeds";
  private static final String LOG_MODULE_STATES = "DriveState/ModuleStates";
  private static final String LOG_MODULE_TARGETS = "DriveState/ModuleTargets";
  private static final String LOG_MODULE_POSITIONS = "DriveState/ModulePositions";
  private static final String LOG_ODOMETRY_PERIOD = "DriveState/OdometryPeriod";

  private final double[] m_poseArray = new double[3];

  private final boolean m_signalLoggerEnabled;

  /**
   * Construct a telemetry object.
   *
   * @param signalLoggerEnabled Whether to record a hoot log with CTRE's SignalLogger
   */
  public SwerveTelemetry(boolean signalLoggerEnabled) {
    m_signalLoggerEnabled = signalLoggerEnabled;

    if (signalLoggerEnabled) {
      /* Path must be set before starting, otherwise the logger restarts */
      SignalLogger.setPath("/media/sda1/");
      SignalLogger.start();
    } else {
      /* Phoenix 6 starts logging on its own during FMS matches, turn that off too */
      SignalLogger.enableAutoLogging(false);
      SignalLogger.stop();
    }

    /* Field2d type only needs to be published once */
    NetworkIO.set(POSITIONING_KEY, ".type", "Field2d");
  }

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    final Pose2d pose = state.Pose;

    /* Telemeterize the swerve drive state */
    NetworkIO.set(TABLE_NAME, CommonTables.POSE_KEY, pose);
    NetworkIO.set(TABLE_NAME, CommonTables.GYRO_KEY, pose.getRotation());
    NetworkIO.set(TABLE_NAME, SPEEDS_KEY, state.Speeds);
    NetworkIO.set(TABLE_NAME, MODULE_STATES_KEY, state.ModuleStates);
    NetworkIO.set(TABLE_NAME, MODULE_TARGETS_KEY, state.ModuleTargets);
    NetworkIO.set(TABLE_NAME, MODULE_POSITIONS_KEY, state.ModulePositions);
    NetworkIO.set(TABLE_NAME, CommonTables.TIMESTAMP_KEY, state.Timestamp);
    NetworkIO.set(TABLE_NAME, ODOMETRY_FREQUENCY_KEY, 1.0 / state.OdometryPeriod);

    /* Also write to log file */
    if (m_signalLoggerEnabled) {
      SignalLogger.writeStruct(LOG_POSE, Pose2d.struct, pose);
      SignalLogger.writeStruct(LOG_SPEEDS, ChassisSpeeds.struct, state.Speeds);
      SignalLogger.writeStructArray(
          LOG_MODULE_STATES, SwerveModuleState.struct, state.ModuleStates);
      SignalLogger.writeStructArray(
          LOG_MODULE_TARGETS, SwerveModuleState.struct, state.ModuleTargets);
      SignalLogger.writeStructArray(
          LOG_MODULE_POSITIONS, SwerveModulePosition.struct, state.ModulePositions);
      SignalLogger.writeDouble(LOG_ODOMETRY_PERIOD, state.OdometryPeriod, "seconds");
    }

    /* Telemeterize the pose to a Field2d */
    m_poseArray[0] = pose.getX();
    m_poseArray[1] = pose.getY();
    m_poseArray[2] = pose.getRotation().getDegrees();

    NetworkIO.set(POSITIONING_KEY, ROBOT_POSE_KEY, m_poseArray);
  }
}
