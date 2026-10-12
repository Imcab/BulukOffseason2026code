// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.constants.swerve;

import com.pathplanner.lib.config.PIDConstants;
import frc.robot.modules.swerve.SwerveRequestFactory;

public class SwerveConstants {

  public static final double MaxSpeed = SwerveRequestFactory.MaxSpeed;
  public static final double MaxAngularRate = SwerveRequestFactory.MaxAngularRate;
  

  public static final double crossMovementSpeed = 0.5;

  // PID de seguimiento de trayectoria (PPHolonomicDriveController).
  // 5.0 / 5.0 hasta tener resultados de tools/pid_sweep (los 10/5 del autotune en Gazebo,
  // 2026-10-01, no estan validados: ahi traslacion ya tuvo un aborto en kP=12).
  // TODO pegar aqui el kP recomendado por tools/pid_sweep y confirmarlo en el robot.
  public static final PIDConstants PathTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
  public static final PIDConstants PathRotationPID = new PIDConstants(5.0, 0.0, 0.0);
}
