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
  // Valores del autotune en Gazebo (MARS, mundo offseason2026, 2026-10-01): traslacion mejora
  // hasta kP=12 pero ahi ya hubo un aborto, asi que se deja un escalon abajo; rotacion es plana
  // entre kP 2 y 6.5 y empeora en 8+. kD no salio por encima del ruido. Confirmar en el robot
  // con PathFollowingTuner (SmartDashboard -> PathTuning) y pegar aqui el resultado.
  public static final PIDConstants PathTranslationPID = new PIDConstants(10.0, 0.0, 0.0);
  public static final PIDConstants PathRotationPID = new PIDConstants(5.0, 0.0, 0.0);
}
