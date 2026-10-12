// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.constants.swerve;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

/**
 * Posicion de tiro frente al hub y parametros del alineado (HubAlign).
 *
 * <p>Geometria (layout 2026 de WPILib, el mismo que usa la vision): la cara frontal del hub azul
 * tiene los tags 25 y 26 y mira hacia el muro azul (180 grados); la del rojo tiene los tags 9 y 10 y
 * mira hacia el muro rojo (0 grados). El objetivo se calcula desde esos tags, asi que sirve igual
 * para las dos alianzas.
 */
public final class HubAlignConstants {

  private HubAlignConstants() {}

  // Tags de la cara frontal del hub (la que mira hacia la propia alianza).
  public static final int[] BLUE_FRONT_TAGS = {25, 26};
  public static final int[] RED_FRONT_TAGS = {9, 10};

  // Todos los tags de cada hub: su promedio da el centro del hub.
  public static final int[] BLUE_HUB_TAGS = {18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] RED_HUB_TAGS = {2, 3, 4, 5, 8, 9, 10, 11};

  // Distancia de la cara frontal del hub al CENTRO del robot (posicion de tiro).
  public static final double SHOOT_DISTANCE_M = 2.54;

  // Desplazamiento a lo largo de la cara, medido desde el eje del hub. 0 = centrado en el hub.
  // Positivo = hacia el tag 25 (azul) / tag 9 (rojo). Los tags 25 y 26 NO estan centrados: el
  // punto medio entre ellos queda 0.178 m hacia el 25; para alinearse a ese punto medio usar 0.178.
  public static final double LATERAL_OFFSET_M = 0.0;

  // Heading final relativo a la normal de la cara (la direccion que se aleja del hub). 0 = la parte
  // de atras del robot apunta al hub: 180 grados en azul y 0 grados de campo en rojo.
  public static final double HEADING_OFFSET_DEG = 0.0;

  // Pathfinding (PathPlanner + navgrid, via PoseFinder). Algo mas suave que los autos.
  public static final PathConstraints PATHFIND_CONSTRAINTS =
      new PathConstraints(3.5, 3.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  // Si ya esta mas cerca que esto, se salta el pathfinding y solo hace el ajuste fino.
  public static final double SKIP_PATHFIND_DISTANCE_M = 0.5;
  public static final double SKIP_PATHFIND_ANGLE_DEG = 20.0;

  // Ajuste fino final (PID de pose con los kP de PathPlanner, ya validados en tools/pid_sweep).
  public static final double FINE_MAX_SPEED_MPS = 1.0;
  public static final double FINE_MAX_OMEGA_RADS = 2.5;
  public static final double POSITION_TOLERANCE_M = 0.03;
  public static final double ANGLE_TOLERANCE_DEG = 2.0;
  public static final double SETTLE_TIME_S = 0.12; // tiempo seguido dentro de tolerancia
  public static final double FINE_TIMEOUT_S = 2.0;

  // Vision + odometria combinadas: el robot siempre navega con la pose fusionada. Sin tags a la
  // vista avanza con odometria; cuando la Limelight ve un tag corrige la pose y, si la correccion
  // es grande, se replanea la ruta desde la pose corregida.
  public static final double REPLAN_JUMP_M = 0.20; // correccion de vision que dispara un replaneo
  public static final double REPLAN_MIN_DISTANCE_M = 1.0; // mas cerca, el ajuste fino la absorbe
  public static final int MAX_REPLANS = 5; // tope por si la vision brinca de un lado a otro

  // Unico freno: si la vision NUNCA ha visto un tag desde que prendio el robot, la pose no
  // significa nada (cree estar en el origen) y no se mueve.
  public static final boolean REQUIRE_VISION_SINCE_BOOT = true;
  // Solo informativo: arriba de esto el estado avisa que va con odometria.
  public static final double STALE_VISION_WARN_S = 2.0;
}
