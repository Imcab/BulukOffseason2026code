// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.constants.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  private VisionConstants() {}

  // Nombre de la tabla de NetworkTables (hostname de la Limelight).
  // TODO confirmar con el hostname configurado en la interfaz web de la Limelight.
  public static final String LIMELIGHT_NAME = "limelight";

  // Centro del robot -> lente de la camara, en convencion WPILib (X adelante, Y izquierda, Z arriba).
  // Se le escribe a la Limelight al arrancar (setCameraPose_RobotSpace) y lo usa el simulador.
  // TODO medir en el robot real.
  public static final Transform3d ROBOT_TO_LIMELIGHT =
      new Transform3d(
          new Translation3d(0.3425, 0.07, 0.42),
          new Rotation3d(0.0, Units.degreesToRadians(10.0), 0.0));

  // Ganancias dinamicas del filtro (desviaciones estandar de la medicion de vision).
  // sigma_xy = BASE_STD_XY * distancia^2 / tags, inflada con la velocidad del chasis
  // y acotada a [MIN_STD_XY, MAX_STD_XY].
  public static final double BASE_STD_XY = 0.04;
  public static final double MIN_STD_XY = 0.02;
  public static final double MAX_STD_XY = 4.0;

  // Por encima de estas velocidades la imagen se degrada y la medicion pesa menos.
  public static final double MAX_SAFE_LINEAR_MPS = 2.0;
  public static final double MAX_SAFE_OMEGA_RADS = Math.PI;

  // MegaTag1 mientras el robot esta deshabilitado, para corregir el heading.
  // sigma = BASE * distancia^2 / tags, acotada. El heading si cuenta aqui: es lo que se busca.
  public static final double MT1_BASE_STD_XY = 0.08;
  public static final double MT1_BASE_STD_THETA = 0.16;
  public static final double MT1_MIN_STD_THETA = 0.05;
  public static final double MT1_MAX_STD_THETA = 2.0;

  // Margen fuera de la cancha dentro del cual una pose todavia cuenta como valida.
  public static final double FIELD_MARGIN_METERS = 0.5;
}
