# Error de seguimiento del tobillo vs kp (simulacion, unitree_mujoco)

Ventana 4-14 s, ejemplo g1_low_level_example, kd = 1, consigna seno de 1 Hz, +-14,3 deg.

| kp tobillos | RMS (deg) | Max (deg) |
|---|---|---|
| 40  | 1,97 | 3,35 |
| 80  | 1,08 | ~2,2 |
| 120 | 0,77 | 1,60 |

El error baja casi como 1/kp (exponente ~0,85). Sin inestabilidad hasta kp = 120 en simulacion.
Las articulaciones sin cambiar de kp (codos, hombros, cintura) dan el mismo RMS en las tres pruebas.
