// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

/**
 * Posiciones de tiro alrededor del hub, todas a la misma distancia del centro del hub (la de la
 * posicion centrada: SHOOT_DISTANCE_M desde la cara frontal). El angulo se mide desde la normal de
 * la cara; izquierda/derecha son como las ve el driver desde su estacion, en las dos alianzas.
 */
public enum HubShotSpot {
  CENTRO("Centro", 0.0),
  IZQUIERDA_30("Izquierda 30", 30.0),
  DERECHA_30("Derecha 30", -30.0),
  IZQUIERDA_45("Izquierda 45", 45.0),
  DERECHA_45("Derecha 45", -45.0),
  /** La variante mas cercana a donde esta el robot al presionar el boton. */
  MAS_CERCANA("Mas cercana", Double.NaN);

  public final String label;
  /** Positivo = a la izquierda del driver. NaN para MAS_CERCANA. */
  public final double angleDeg;

  HubShotSpot(String label, double angleDeg) {
    this.label = label;
    this.angleDeg = angleDeg;
  }

  public boolean isFixed() {
    return !Double.isNaN(angleDeg);
  }
}
