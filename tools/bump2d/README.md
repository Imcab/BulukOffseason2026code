# bump2d — modelo 2D del cruce del bump

Vista lateral (x, z, pitch) del robot cruzando el bump de REBUILT. Sirve para responder
"¿toca la panza?", "¿a qué velocidad cruza?" y "¿conviene cruzar recto, de lado o en diamante?".
No se despliega al robot: vive en `tools/`, fuera de `src/`.

## Archivos

- `extract_profile.py` — lee el STL del CAD (parser binario propio) y genera `robot_profile.json`:
  la silueta de abajo/arriba de todo lo que no es llanta, en tiras de 1 cm, y dónde quedan las
  ruedas a lo largo del avance para cada dirección de cruce. El STL (148 MB) no se commitea.
- `core/bump_model.py` — cuerpo rígido + contactos (llantas resorte-amortiguador con fricción
  limitada a μN y motor con lazo de velocidad y límite de stator; silueta del chasis como puntos de
  contacto). Todos los escenarios corren en paralelo con numpy.
- `run_bump.py` — barrido velocidad × dirección × altura extra del chasis; `--viewer x.html` genera
  el visor animado (`core/viewer.py` + `core/viewer_template.html`).
- `tests/` — sanidad contra cálculos a mano (piso plano, umbral de "high-centering" 7.9 cm,
  odometría +4.8 cm por cruce, diamante 1-2-1 ruedas).

## Uso

```
python -I extract_profile.py ruta/al/robot.stl --out robot_profile.json   # solo si cambia el CAD
python run_bump.py                                   # tabla
python run_bump.py --speeds 0.5,1,2 --viewer bump.html
python -m pytest tests -q
```

## Direcciones

| nombre | ruedas a lo largo del avance |
|---|---|
| `intake_adelante`, `intake_atras`, `de_lado` | 2 ejes de 2 ruedas a ±0.276 m |
| `diamante_intake`, `diamante_atras` (45°) | 1 rueda a +0.39 m, 2 al centro, 1 a −0.39 m |

## Valores supuestos (TODO medir)

- Altura del centro de masa 0.20 m e inercia de pitch 5 kg·m² (`Robot`).
- Fricción del bumper contra el bump 0.3 y bumper rígido (el real es tela + espuma y se deforma).
- La silueta usa el intake como está en el STL.
