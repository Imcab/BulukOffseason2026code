# pid_sweep: kP de PathPlanner por simulación

Herramienta para elegir los kP de traslación y rotación del `PPHolonomicDriveController` del
robot dumper (STZ Robotics / MARS #3472). Simula miles de robots en paralelo con parámetros
físicos aleatorizados, busca automáticamente el mejor par de kP que siga siendo estable en
**todo** el rango (no solo en el mejor caso), lo valida con cientos de miles de corridas y genera
una repetición visual.

Todo es Python + torch. Corre en Windows, Linux o Mac, en CPU o GPU NVIDIA. No necesita Isaac
Sim, el proyecto del robot, pathplannerlib ni wpimath.

> **Los kP que salgan de aquí son un punto de partida.** El modelo es una simplificación. Hay que
> confirmarlos en el robot real: empieza con el valor recomendado, corre los autos y sube o baja
> con cuidado.

## Qué hay

```
params.yaml              datos físicos, rangos aleatorizados, variantes, grid, autotune y estrés
trajectories/*.json      8 trayectorias SINTÉTICAS ya muestreadas (t, x, y, theta, vx, vy, omega)
core/
  model.py               módulo swerve: Kraken X60 FOC, límites de corriente, fricción, steer,
                         llanta (círculo de fricción), reparto de carga por altura/offset del CM
  controller.py          PPHolonomicDriveController (FF + P en x, y, theta) y latencia
  sim.py                 simulador por lotes: N robots, cada uno con su trayectoria, en un solo lote
  autotune.py            autotuning de kP (CEM) + validación masiva + escenarios de estrés
  sensitivity.py         margen de ganancia según cada parámetro físico (qué medir primero)
  replay.py              repetición visual en HTML (cuadrícula de entornos tipo "headless")
  sweep_torch.py         barrido en grid completo (mapa de estabilidad)
  report.py              summary.md y gráficas del grid
  trajgen.py             generador de las trayectorias sintéticas
tests/                   52 pruebas (física, controlador, lotes, CPU=GPU, autotune, replay)
referencia/              resultados del 2026-10-12 (RTX 4060): autotune.md, sensibilidad,
                         grid/ (mapa de estabilidad) y replay.html
```

Cada robot simulado:
1. Arranca desplazado hasta 15 cm / 5° del inicio del path.
2. Sigue la trayectoria a 50 Hz con 20–60 ms de latencia.
3. Sus módulos producen fuerza según el motor, el límite de corriente y la tracción.
4. Al terminar manda velocidad 0, igual que `FollowPathCommand`.

La prueba **`mantener_pose`** (robot desplazado 15 cm / 5°, solo corrige el PID) es la que
detecta inestabilidad. El resto de las trayectorias miden el seguimiento.

**Variantes** (cada una se analiza por separado):

| variante | qué representa |
|---|---|
| `actual_openloop` | el robot HOY: `ApplyRobotSpeeds` en `OpenLoopVoltage`, stator 120 A |
| `velocity` | si se cambia a `DriveRequestType.Velocity` con kV = 0.124 |
| `velocity_limites_propuestos` | Velocity + stator 80 A / supply 70 A (propuesta, no aplicada) |

## Instalación

Python 3.10+.

```bash
cd tools/pid_sweep
python -m venv .venv
# Windows: .venv\Scripts\activate      Linux/Mac: source .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cu128   # GPU NVIDIA
# o, solo CPU:  pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install pyyaml numpy matplotlib pytest
python -c "import torch; print(torch.cuda.is_available())"   # True = usará la GPU
```

## Uso

Todos los comandos se corren desde `tools/pid_sweep`. Con GPU agrega `--device cuda`; en CPU
usa `--quick` para las corridas grandes.

| paso | comando | tiempo (RTX 4060) |
|---|---|---|
| 1. Pruebas | `python -m pytest tests -q` | ~3.5 min |
| 2. Autotune | `python -m core.autotune --device cuda` | ~8 min por variante |
| 3. Sensibilidad | `python -m core.sensitivity --kp 8.0,10.5 --device cuda` | ~5 min |
| 4. Replay visual | `python -m core.replay --scenario "Autotune=8,10.5" --scenario "Gazebo=10,5" --scenario "Inestable=25,25"` | ~20 s |
| 5. Mapa en grid (opcional) | `python -m core.sweep_torch --device cuda` | ~1 min |

Notas por paso:
- **Pruebas:** tiene que decir `52 passed`. La prueba CPU = GPU se salta si no hay CUDA.
- **Autotune:** escribe `results_autotune/autotune.md`, el historial CSV y las gráficas.
- **Sensibilidad:** pon en `--kp` el resultado del autotune. Escribe
  `results_autotune/sensibilidad_<variante>.md` y una PNG.
- **Replay:** abre `results/replay.html` en el navegador. No necesita internet.
- **Mapa en grid:** escribe `results/summary.md` y el mapa de estabilidad kP_t × kP_r.

Para recalcular el reporte del grid sin volver a simular:
`python -m core.report results/sweep_results.csv`.

## Cómo funciona el autotune

Usa el método de entropía cruzada (CEM) sobre (log kP_t, log kP_r):
1. Cada generación prueba 48 candidatos. Cada uno corre con los mismos 192 robots aleatorizados
   en las 8 trayectorias más `mantener_pose`, **dos veces**: con sus kP y con kP / 0.65.
2. Un candidato **es válido** solo si no hay ni un robot inestable en ninguna de las dos corridas.
   Así queda, como mucho, al 65% del límite de inestabilidad en todo el rango aleatorizado
   (la regla del 60–70%).
3. Entre los válidos se queda con el de menor **costo p95** (percentil 95 sobre robots y
   trayectorias). El costo por corrida es: error RMS de posición + error final + lo mismo en
   heading (1° = 1 cm) + 1 m si aborta.
4. Al final re-evalúa los mejores candidatos juntos y valida al ganador:
   - con 16 384 robots nuevos (≈ 295 000 corridas),
   - contra el código actual (5, 5) y el autotune de Gazebo (10, 5),
   - en escenarios **fuera** del rango (estrés): latencia 80 ms, COF 0.7, 75% de velocidad,
     +8 kg, CM a 0.30 m, error inicial 30 cm / 10°.

Los pesos, tamaños y escenarios están en `autotune:` dentro de `params.yaml`.

## Cómo leer los resultados

- **`autotune.md`**:
  - La tabla *Resultado* da el kP elegido y el valor redondeado para pegar en
    `SwerveConstants.PathTranslationPID` / `PathRotationPID` (usa la variante de tu código; hoy
    es `actual_openloop`).
  - *Inestables (kP/0.65)* tiene que ser 0: eso significa que se cumple el margen.
  - *Estrés* muestra qué tan rápido se degrada fuera del rango. Muchos abortos con COF bajo o
    con poca velocidad indican que los **constraints del path** (4.5 m/s, 4 m/s²) piden más de
    lo que el robot puede dar. Subir el kP no arregla eso.
- **`sensibilidad_<variante>.md`**: el parámetro que más mueve el margen de ganancia es el que
  conviene medir primero en el robot.
- **`replay.html`**:
  - Tiene dos vistas: "Un entorno por robot" (cuadrícula, como Isaac en headless) y
    "Superpuestos" (todas las variaciones de una trayectoria encimadas).
  - Los colores: azul = siguió el path, ámbar = abortó, rojo = inestable.
  - Pasa el mouse sobre un robot para ver su latencia, COF, velocidad, error final, etc.
  - Todos los escenarios usan los mismos robots.
- **`summary.md`** (grid): el mapa de estabilidad completo, útil para ver la forma de la frontera.

## Supuestos y limitaciones

- Cuerpo rígido plano, sin suspensión. La transferencia de carga es cuasi-estática.
- La pose medida es la real: sin ruido y sin deriva de odometría por derrape. La latencia es un
  retraso puro del comando.
- Sin modelo de batería. La caída de voltaje solo entra vía `speed_factor`.
- El steer usa un límite de velocidad y un primer orden con valores supuestos.
- La llanta usa un modelo lineal de ángulo de deslizamiento con círculo de fricción. No hay datos
  del Spike Grip.
- La fricción del módulo se supone igual al kS estimado (0.408 V). La sensibilidad muestra qué
  pasa si es distinta.
- Se ignoran los feedforwards de fuerza de PathPlanner, igual que el código actual.
- Las trayectorias son sintéticas, porque el proyecto no tiene paths de PathPlanner. Cuando
  existan, muestréalas al mismo formato JSON en `trajectories/`.
- kI = kD = 0.

**Datos pendientes de medir en el robot** (actualiza `params.yaml` y vuelve a correr el autotune):
latencia del lazo, kS/kV (SysId), velocidad máxima real, COF del Spike Grip, altura del CM y
parámetros del steer.
