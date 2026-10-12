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
  // Autotune de tools/pid_sweep (2026-10-12) con kS/kV MEDIDOS (0.1295 V, 0.1156 V/rps),
  // variante velocity_limites_propuestos: drive en DriveRequestType.Velocity + stator 70 A /
  // supply 70 A. Estable en 4 x 16384 robots aleatorizados incluso con kP / 0.65 (regla del 65%
  // del limite). Error p95 ~0.21 m. (6.5, 8.0) ya falla el margen en algunos robots.
  // Si el drive vuelve a OpenLoopVoltage hay que correr el autotune de la variante actual_openloop.
  // TODO confirmar en el robot: si oscila al final del path bajar ~20%.
  public static final PIDConstants PathTranslationPID = new PIDConstants(6.0, 0.0, 0.0);
  public static final PIDConstants PathRotationPID = new PIDConstants(7.5, 0.0, 0.0);
}
