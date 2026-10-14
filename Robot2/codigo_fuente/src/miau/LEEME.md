# Trayectoria PH-PA-PH-P1-PH-PB-PH-P2-PH con cinemática diferencial inversa

## Qué hay en este paquete de cambios
Copia estas carpetas/archivos dentro de tu paquete `miau` (respetando las rutas).
No se toca `nodo.cpp`, `vti2.cpp`, el URDF ni `controllers.yaml`.

| Archivo | Estado | Para qué sirve |
|---|---|---|
| `src/nodo_jacobiano.cpp` | NUEVO | Nodo ROS 2: J⁻¹ + Euler, homing, trayectoria y CSV |
| `include/miau/diff_ik_planner.hpp` | NUEVO | Planificador (C++ puro): q̇ = J⁻¹(q)·v, Euler, límite 1 rad/s |
| `config/trayectoria.yaml` | NUEVO | **Puntos, secuencia y velocidades (lo que vas a editar)** |
| `launch/trayectoria.launch.py` | NUEVO | Lanza Gazebo (tu `gazebo.launch.py`) + el nodo |
| `CMakeLists.txt` | MODIFICADO | Solo añade el ejecutable `nodo_jacobiano` y C++17 |

## Compilar y correr
```bash
cd <tu_workspace>
colcon build --packages-select miau --symlink-install
source install/setup.bash
ros2 launch miau trayectoria.launch.py
```
Si arrastras carpetas `build/ install/ log/` viejas (aparece `Miau` con mayúscula), bórralas y recompila.

## Cambiar puntos o movimientos (`config/trayectoria.yaml`)
```yaml
points:
  P3: [0.10, 0.05, 0.20]          # <- punto nuevo
sequence: ["PH","PA","PH","P3","PH"]   # <- y úsalo en la secuencia
```
Escribe siempre los números con punto decimal. Con `--symlink-install` basta con volver a lanzar.
El script de Python tiene la misma configuración arriba del archivo (`PUNTOS` y `SECUENCIA`).

## Qué hace el nodo
1. **Preflight**: simula todo con el modelo KDL y avisa (singularidad, juntas fuera de ±π, error cartesiano).
2. **Homing**: lleva las juntas desde donde estén hasta PH (≤ `home_joint_vel`, 0.5 rad/s).
3. **Trayectoria**: líneas rectas entre puntos; q̇ = J⁻¹(q)·v_e, q(k+1) = q(k) + q̇·Δt.
   Si alguna junta llegaría a más de `max_joint_vel` (1 rad/s), se reduce todo el q̇ (la trayectoria sigue igual, más lenta).

## Datos exportados (carpeta `~/miau_logs/`, con fecha y hora en el nombre)
* `*_real.csv`: lo medido en Gazebo por `/joint_states`: posición, velocidad y **torque** (effort) de J1, J2, J3, más la posición real del TCP.
* `*_plan.csv`: lo comandado: posición, velocidad y aceleración por junta, torque calculado por dinámica inversa (`_torque_rne`), punto de referencia vs FK, det(J) y factor de recorte de velocidad.

Nota: el torque de `*_real.csv` es lo que reporta el plugin de gz_ros2_control; si sale vacío/NaN o en cero, usa `_torque_rne` del `*_plan.csv`.
