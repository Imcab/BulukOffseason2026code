// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.configuration.constants.swerve.TunerConstants;


public class SwerveRequestFactory {

  public static final double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public static final double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  public static SwerveRequest.Idle idle() {
    return new SwerveRequest.Idle();
  }

  public static SwerveRequest.SwerveDriveBrake brake() {
    return new SwerveRequest.SwerveDriveBrake();
  }

  public static SwerveRequest.PointWheelsAt point() {
    return new SwerveRequest.PointWheelsAt();
  }

  public static SwerveRequest.FieldCentric driveFieldCentric() {
    return new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(
            DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  }

  public static SwerveRequest.FieldCentric simpleDriveRequest() {
    return new SwerveRequest.FieldCentric();
  }

  public static SwerveRequest.RobotCentric driveRobotCentric() {
    return new SwerveRequest.RobotCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  }

  // Objeto para aplicar velocidades desde PathPlanner
  public static SwerveRequest.ApplyRobotSpeeds pathPlannerRequest() {
    return new SwerveRequest.ApplyRobotSpeeds();
  }

  public static SwerveRequest.SysIdSwerveTranslation translationCharacterization() {
    return new SwerveRequest.SysIdSwerveTranslation();
  }

  public static SwerveRequest.SysIdSwerveSteerGains steerCharacterization() {
    return new SwerveRequest.SysIdSwerveSteerGains();
  }

  public static SwerveRequest.SysIdSwerveRotation rotationCharacterization() {
    return new SwerveRequest.SysIdSwerveRotation();
  }
}