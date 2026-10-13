# bump3d: escenas del bump en Gazebo, en paralelo

Un mundo de Gazebo (el motor de MARS) con muchas copias del robot, cada una con su bump
enfrente, cruzando a la vez: columnas = dirección, filas = velocidad (y altura del chasis).
Complementa a `tools/bump2d`: el 2D barre miles de casos en segundos; esto lo muestra en 3D,
con la colisión del robot de MARS. No se despliega al robot.

## Uso

```powershell
.\tools\bump3d\run_bench.ps1                          # ventana 3D; arranca sola y se repite
.\tools\bump3d\run_bench.ps1 -Headless -Seconds 12    # solo la tabla
.\tools\bump3d\run_bench.ps1 -Speeds 1,2 -LiftsCm 0,4 -Directions diamante_intake,intake_atras
```

Con ventana, las escenas se repiten en bucle: cuando todas terminan, cada robot regresa a su
salida y vuelve a cruzar. Resultados (de la primera vuelta) en `build/resultados.json`, con la
traza x, z, pitch de cada escena cada 20 ms.

## Qué usa de MARS (sin modificar nada de su instalación)

- `mars-sim-server.exe` / `mars-sim-gui.exe` y el entorno conda `mars-sim`.
- `worlds/gen_offseason2026.py` para armar el robot: mallas del CAD, bumper como caja de colisión
  a su altura real (fondo a 3.5 cm), masa 58 kg.
- La rampa del bump igual a `worlds/cancha_2026.py` (16.5 cm × 1.12 m, 2.23 m de ancho).

## Cómo se mueve cada robot

No usa el código del robot: MarsLink escucha tópicos fijos, así que en un mundo solo un robot
puede recibir órdenes del código. Cada copia lleva plugins estándar de gz-sim:
la dirección fija en el ángulo de cruce y un `JointController` de velocidad por rueda con el par
topado a 70 A de stator (7.2 N·m en la rueda). `measure_bench.py` frena cada robot 0.6 s después
de cruzar.

## Diferencias con el robot real (TODO medir)

- Fricción del bumper 0.3 (Gazebo usa 1.0 si no se dice nada; con 1.0 el robot en diamante a
  3 m/s queda clavado en la esquina del bumper).
- La colisión del chasis es una caja sólida con el fondo a 3.5 cm; el CAD tiene 4.1 cm al centro.
- Sin contraelectromotriz: hasta 3 m/s manda el límite de corriente, así que no cambia el resultado.

## Glue con el código del robot

Para manejar UN robot con el código completo (autos, HubAlign) en la física de Gazebo:
`./gradlew simulateJava -Pmarssim` con MARS Simulation Studio corriendo
(ver `src/main/java/frc/robot/modules/swerve/MarsSimGlue.java`). Sin `-Pmarssim`,
la simulación es la normal de WPILib + CTRE.
