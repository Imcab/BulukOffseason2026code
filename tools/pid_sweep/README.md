# pid_sweep: kP de PathPlanner por simulación

Paquete autocontenido para elegir los kP de traslación y rotación del `PPHolonomicDriveController`
del robot dumper (STZ Robotics / MARS #3472). Simula miles de robots en paralelo, cada uno con un
par (kP_t, kP_r) distinto y parámetros físicos aleatorizados. Luego recomienda un kP que sea
estable en **todo** el rango aleatorizado, no solo en el mejor caso.

> **Los kP que salgan de aquí son un punto de partida.** El modelo es una simplificación. Hay que
> confirmarlos en el robot real: empieza con el valor recomendado, corre los autos y sube o baja
> con cuidado.

## (a) Qué hace

```
params.yaml            datos físicos, rangos de aleatorización, variantes y grid de kP
trajectories/*.json    8 trayectorias SINTÉTICAS ya muestreadas (t, x, y, theta, vx, vy, omega)
core/                  torch puro, sin Isaac:
  model.py             módulo swerve: motor Kraken X60 FOC, límites de corriente, steer,
                       llanta (círculo de fricción), reparto de carga por altura/offset del CM
  controller.py        PPHolonomicDriveController (FF de la trayectoria + P en x, y, theta) y latencia
  sim.py               corre una trayectoria en N entornos y calcula métricas
  sweep.py, report.py  barrido, CSV, summary.md y gráficas
  sweep_torch.py       el sweep completo SIN Isaac (CPU o GPU)
  trajgen.py           generador de las trayectorias sintéticas
isaac/run_sweep.py     la misma simulación, pero Isaac Lab integra el cuerpo rígido. NO PROBADO.
tests/                 pruebas de core/ (corren en CPU, sin Isaac)
scripts/generate_trajectories.py   regenera trajectories/ si cambian los constraints
referencia_torch_cpu/  resultado del sweep torch en CPU (Windows), para comparar
```

Cada robot simulado:
1. Arranca desplazado hasta 15 cm / 5° del inicio del path.
2. Sigue la trayectoria con el controlador de PathPlanner a 50 Hz.
3. El comando llega a los módulos con 20–60 ms de latencia.
4. Los módulos producen fuerza según el motor, el límite de corriente y la tracción.
5. Al terminar el path manda velocidad 0, igual que `FollowPathCommand`.

Además, cada robot corre la prueba **`mantener_pose`**. Es una trayectoria quieta en la que el
robot empieza desplazado 15 cm / 5° y solo corrige el PID. Ahí cualquier oscilación viene del lazo
y no del path, así que es la prueba que define el límite de estabilidad.

**Variantes** (se barren las tres con los mismos robots aleatorizados):

| variante | qué representa |
|---|---|
| `actual_openloop` | el robot HOY: `ApplyRobotSpeeds` en `OpenLoopVoltage`, stator 120 A (kSlipCurrent) |
| `velocity` | si se cambia a `DriveRequestType.Velocity` con kV = 0.124 |
| `velocity_limites_propuestos` | Velocity + stator 80 A / supply 70 A (propuesta, no aplicada) |

## (b) Requisitos y versión asumida

- Ubuntu 22.04 (cualquier Linux con Isaac Sim funciona).
- **Isaac Lab v2.3.2** sobre **Isaac Sim 4.5 / 5.0 / 5.1**, Python 3.11. Es la última versión
  estable al escribir esto (2026-10). Isaac Lab 3.0 todavía era "early access" y cambia la API de
  fuerzas externas, así que **no** está soportado.
- Para la parte sin Isaac solo se necesita: `torch`, `pyyaml`, `numpy`, `matplotlib` y `pytest`
  (ver `requirements.txt`).

Para revisar tu versión de Isaac Lab:

```bash
cd ~/IsaacLab            # o donde lo tengas instalado
git describe --tags      # debería decir v2.3.x
./isaaclab.sh -p -c "import isaaclab, torch; print(torch.__version__, torch.cuda.is_available())"
```

## (c) Copiarlo a Ubuntu

Solo hace falta esta carpeta. No necesita el proyecto del robot, ni pathplannerlib, ni wpimath.

```bash
# desde la PC con el proyecto (Windows con Git Bash, o Linux):
scp -r tools/pid_sweep usuario@ip-ubuntu:~/pid_sweep
# o cópiala en una USB y en Ubuntu:  cp -r /media/$USER/USB/pid_sweep ~/pid_sweep
```

## (d) Cómo correrlo

Todos los comandos asumen `cd ~/pid_sweep`, salvo donde se indica otra carpeta.

### Paso 1: pruebas de core (sin Isaac)

Con el Python de Isaac Lab (ya trae torch):

```bash
cd ~/pid_sweep
~/IsaacLab/isaaclab.sh -p -m pip install pyyaml matplotlib pytest   # si faltan
~/IsaacLab/isaaclab.sh -p -m pytest tests -q
```

O en un entorno aparte:

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install pyyaml numpy matplotlib pytest
python -m pytest tests -q
```

Tiene que decir `39 passed`. Si algo falla aquí, el problema no es de Isaac: pega la salida a Claude.

### Paso 2: sweep en torch (sin Isaac), como referencia

```bash
python -m core.sweep_torch --quick --out results_torch_quick       # ~15 s, 24 robots
python -m core.sweep_torch --out results_torch                     # 3000 robots x 9 pruebas x 3 variantes
python -m core.sweep_torch --device cuda --out results_torch       # igual, en GPU
```

En CPU (laptop de 8 núcleos) el sweep completo (5000 robots) tarda ~3.5 minutos.
`referencia_torch_cpu/` trae el `summary.md` y las gráficas de esa corrida, hecha en Windows
el 2026-10-12, para comparar.

Para recalcular el reporte sin volver a simular (por ejemplo, si cambias `fraction_of_limit`):

```bash
python -m core.report results_torch/sweep_results.csv
```

### Paso 3: sweep en Isaac Lab (NO PROBADO)

Primero con pocos entornos:

```bash
cd ~/IsaacLab
./isaaclab.sh -p ~/pid_sweep/isaac/run_sweep.py --headless --quick --out ~/pid_sweep/results_isaac_quick
```

Compara `results_isaac_quick/summary.md` con `results_torch_quick/summary.md`. Deben salir
**casi iguales**, porque Isaac solo integra el cuerpo rígido. Si coinciden, corre el masivo:

```bash
./isaaclab.sh -p ~/pid_sweep/isaac/run_sweep.py --headless --out ~/pid_sweep/results_isaac
```

Para más entornos o un grid más fino, edita `sweep:` en `params.yaml`: `kp_translation`,
`kp_rotation` y `samples_per_cell`. Entornos = num_t × num_r × samples_per_cell.

## (e) Cómo leer `results/`

- **`summary.md`**: empieza aquí.
  - *Recomendación por variante*: el límite robusto de kP_t y kP_r y el kP recomendado (65% del
    límite, redondeado hacia abajo a 0.5).
  - *Métricas en la celda recomendada*: mediana y peor caso de cada métrica sobre todos los robots
    y trayectorias.
  - *Primer caso inestable*: qué robot fijó el límite. Si casi siempre es el de latencia alta,
    **medir la latencia real** es lo que más mueve la recomendación.
  - *Si no aparece inestabilidad en el rango barrido*: el summary lo dice. No extrapoles: amplía el grid.
- **`sweep_results.csv`**: una fila por robot y por trayectoria. Columnas:
  - Parámetros: `kp_t`, `kp_r`, `mass`, `moi`, `cg_height`, `cof`, `speed_factor`, `latency_s`, `init_*`.
  - Métricas: `lateral_rms_m`, `pos_rms_m`, `max_*`, `final_pos_err_m`, `final_yaw_err_deg`,
    `overshoot_m`, `overshoot_deg`, `path_time_s`, `total_time_s`, `slip_frac` y `reversals_*`.
  - Banderas (0/1): `abort`, `unstable_t`, `unstable_r`, `unstable` y `wheel_lift`.
- **`estabilidad_<variante>.png`**: mapa kP_t × kP_r con la fracción de robots inestables. La
  estrella es la recomendación.
- **`error_vs_kp_traslacion.png` / `error_vs_kp_rotacion.png`**: peor caso y mediana del error
  contra kP. Si la curva del peor caso **sube** antes del límite, la latencia o la tracción ya
  están mandando y subir kP no ayuda.

Qué significa cada bandera:
- **aborto**: el auto habría fallado (error > 0.75 m o > 45° en algún momento). Se reporta, pero no
  fija el límite: un kP muy bajo también "aborta" por atraso, y eso no es inestabilidad.
- **inestable**: el lazo diverge (> 1.5 m o > 90°) u oscila en `mantener_pose` (3 o más inversiones
  del error, con histéresis de 2 cm / 2°).

Para pasar el resultado al robot, pon los kP recomendados de la variante que corresponda a tu
código en `SwerveConstants.PathTranslationPID` y `PathRotationPID`. Hoy esa variante es
`actual_openloop`.

## (f) Problemas comunes

| síntoma | qué hacer |
|---|---|
| `ModuleNotFoundError: yaml` / `matplotlib` | `~/IsaacLab/isaaclab.sh -p -m pip install pyyaml matplotlib` |
| `ModuleNotFoundError: core` | corre los comandos desde `~/pid_sweep` (o usa la ruta absoluta del script de Isaac) |
| Isaac: `AttributeError` / `TypeError` en `permanent_wrench_composer`, `set_masses`, `set_inertias`, `write_root_pose_to_sim` | tu versión de Isaac Lab no es la 2.3.x. Pega a Claude el error completo **y** la salida de `git describe --tags` |
| Isaac: el robot no se mueve (métricas iguales para todos los kP) | posiblemente el cuerpo está "dormido" o las fuerzas no se aplican. Pega a Claude las primeras líneas del log `[isaac]` y `[sweep]` |
| Isaac y torch dan resultados muy distintos | bug en la capa de Isaac. Pega a Claude ambos `summary.md` |
| Se queda sin memoria de GPU | baja `samples_per_cell` o el número de puntos del grid |
| Tarda mucho en crear la escena | normal con miles de entornos y `replicate_physics=False`. Prueba primero con `--quick` |

**Si falla la API de Isaac**: no la arregles a ciegas. Copia el traceback completo, la versión
(`git describe --tags` en la carpeta de Isaac Lab) y el comando que corriste, y pégalo de vuelta en
la conversación con Claude. Mientras tanto, los resultados de `core.sweep_torch` son válidos:
usan el mismo modelo.

## (g) Supuestos, limitaciones y aviso

- Cuerpo rígido plano, sin suspensión. La transferencia de carga es cuasi-estática.
- La pose medida es la real: sin ruido y sin deriva de odometría por derrape. La latencia es un
  retraso puro del comando.
- Sin modelo de batería. La caída de voltaje solo entra vía `speed_factor`.
- El steer usa un límite de velocidad y un primer orden con valores supuestos.
- La llanta usa un modelo lineal de ángulo de deslizamiento con círculo de fricción. No hay datos
  del Spike Grip.
- La fricción del módulo se supone igual al kS estimado (0.408 V). Si el kS real es menor, en modo
  Velocity el robot puede "cazar" ±1° de heading con kP_r moderado.
- Se ignoran los feedforwards de fuerza de PathPlanner, igual que el código actual.
- Las trayectorias son sintéticas, porque el proyecto no tiene paths de PathPlanner. Cuando existan,
  conviene muestrearlas y agregarlas a `trajectories/` con el mismo formato JSON.
- kI = kD = 0.

**Datos pendientes de medir en el robot** (cambian la recomendación; actualiza `params.yaml`):
kS real, COF real del Spike Grip, velocidad máxima real, latencia del lazo, altura del CM y
parámetros del steer.

Los kP de este paquete son un **punto de partida**: el ajuste final se hace en el robot real.
