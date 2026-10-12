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
  // Autotune de tools/pid_sweep (2026-10-12), variante velocity_limites_propuestos:
  // drive en DriveRequestType.Velocity + stator 80 A / supply 70 A. Estable en 16384 robots
  // aleatorizados incluso con kP / 0.65 (regla del 65% del limite). Error p95 ~0.21 m.
  // Si el drive vuelve a OpenLoopVoltage, usar (8.0, 10.5) de la variante actual_openloop.
  // TODO confirmar en el robot despues de medir kS/kV con DriveChar y volver a correr el autotune.
  public static final PIDConstants PathTranslationPID = new PIDConstants(6.5, 0.0, 0.0);
  public static final PIDConstants PathRotationPID = new PIDConstants(8.0, 0.0, 0.0);
}
