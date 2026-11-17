# Robot 1: integración con la planta (brain)

Esta guía explica cómo el Robot 1 cumple el contrato de `propuesta_brain.pdf` sin cambiar la lógica de control existente (`udp_bridge`, `kinematic_control`, CSV, ESP32).

## Qué se agregó

```
src/robot_control/
├── robot_control/robot1_core.py        NUEVO  lógica del contrato (sin ROS, probada)
├── robot_control/robot1_node.py        NUEVO  nodo ROS 2 del Robot 1 (/robot_1/cmd, /robot_1/estado, ...)
├── launch/robot1_planta.launch.py      NUEVO  lanza el robot completo + robot1_node
├── test/test_robot1_core.py            NUEVO  21 pruebas (no necesitan ROS)
├── test/conftest.py                    NUEVO
├── robot_control/trajectory_publisher.py   MODIFICADO (solo se agregó código, ver abajo)
├── config/params.yaml                  MODIFICADO (1 parámetro + bloque robot1_node)
├── setup.py                            MODIFICADO (1 línea: registra robot1_node)
└── package.xml                         MODIFICADO (1 línea: depend planta_interfaces)
```

`udp_bridge.py`, `kinematic_control.py`, `kinematics.py`, `robot.launch.py`, el CSV y los paquetes `antropomorfico_sim` y `robot_interfaces` **no se tocaron**.

### Cambios en `trajectory_publisher.py` (todos aditivos)

Nada de lo que ya hacía cambia: `~/start` y `~/stop` se comportan igual y el CSV se recorre igual. Se agregó:

1. **Tópico `~/estado`** (`std_msgs/String`, confiable + transient local): publica `espera`, `corriendo`, `pausa`, `fin` o `volviendo`. Se publica al cambiar y se repite cada 1 s. Es la forma de saber cuándo terminó la trayectoria.
2. **Servicio `~/reset`** (`std_srvs/Trigger`): desde `pausa` o `fin` vuelve en línea recta al primer punto del CSV (el home) y queda en `espera`. Mientras vuelve, publica `x_d` y `ẋ_d` consistentes (sin saltos), para que `kinematic_control` no dispare su falla. Lo usa `ABORTAR`.
3. **Parámetro `vel_retorno_m_s`** (0.05 m/s por defecto): velocidad del regreso.
4. **Estado nuevo `volviendo`**: durante el regreso `~/start` responde "Está volviendo al inicio".

## Cómo implementarlo

1. Copia los archivos del zip sobre tu repo, respetando las rutas (el zip ya trae la estructura de carpetas). Los archivos modificados van completos; si prefieres aplicar solo los cambios, usa el parche `cambios_archivos_existentes.patch` desde la raíz del repo:
   ```bash
   git apply cambios_archivos_existentes.patch
   ```
2. Junta el paquete compartido en el mismo workspace. `planta_interfaces` está en el entregable del brain (`Brain/codigo_fuente/planta_ws/src/planta_interfaces`). Cópialo (o enlázalo) dentro de `Robot1/codigo_fuente/antropomorfico_ws/src/`.
3. Compila:
   ```bash
   cd Robot1/codigo_fuente/antropomorfico_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select planta_interfaces robot_interfaces robot_control
   source install/setup.bash
   ```
4. Revisa `src/robot_control/config/params.yaml`:
   - `udp_bridge.esp32_ip`: la IP actual de la ESP32 (hoy dice `10.125.133.118`).
   - Que el PC y los demás equipos usen el mismo `ROS_DOMAIN_ID` (valor por definir entre los grupos).
5. Lanza todo el Robot 1:
   ```bash
   export ROS_DOMAIN_ID=<el acordado>
   ros2 launch robot_control robot1_planta.launch.py
   ```
   `robot.launch.py` sigue funcionando para probar el robot solo, sin la planta.

`robot1_node` **no publica estado hasta ver datos de la ESP32 y el estado de `trajectory_publisher`**. Cuando el brain empieza a ver `/robot_1/estado` en 0, el robot está listo. Si el brain muestra el Robot 1 en 3 (sin comunicación), revisa el enlace con la ESP32 antes que nada.

## Qué hace con cada orden del brain

| Llega | Qué hace el Robot 1 | Qué publica en `/robot_1/estado` |
|---|---|---|
| `T1` (nueva `seq`) | Llama `~/start` | De inmediato estado 1 (`caja_id`, `seq`). Al terminar la trayectoria, 2. Cuando está en home, 0 |
| `T1` con la misma `seq` | No repite la tarea | Vuelve a publicar su estado actual |
| `T1` con `seq` menor | La ignora | Nada |
| `PAUSAR` | Llama `~/stop`: se queda donde está | Su estado actual |
| `REANUDAR` | Llama `~/start`: retoma donde quedó | Su estado actual |
| `ABORTAR` | `~/stop`, luego `~/reset`: vuelve al inicio | 0 cuando esté en home y listo |
| `/planta/estop = true` | Igual que `PAUSAR` | Su estado actual |
| `/planta/estop = false` | Sigue en pausa hasta recibir `REANUDAR` | Su estado actual |
| Sin `/brain/latido` por `t_hb_s` | Pausa (parada segura) | Su estado actual |

El estado se repite cada 1 s (sirve de latido hacia el brain). No publica 0 mientras tenga una caja en proceso.

### Cuándo reporta estado 3 (error)

- Se pierde el enlace con la ESP32 (sin `/joint_states` por más de `t_link_s`).
- `trajectory_publisher` deja de publicar su estado.
- El error de seguimiento pasa de `e_error_m` (0.12 m) durante una tarea.
- Llega `T1` con el robot fuera de home, sin enlace o con la trayectoria ocupada.
- La trayectoria no arranca en `t_arranque_s`, o no vuelve a home en `t_asentar_s`.

Al fallar, mantiene la posición (`~/stop`) y sigue en 3 hasta recibir `ABORTAR`. El brain lo manda cuando la GUI pulsa Continuar.

## Parámetros (bloque `robot1_node` de `params.yaml`)

| Parámetro | Valor | Para qué |
|---|---|---|
| `e_inicio_m` | 0.02 | Error máximo para considerar al robot en home (igual al del CC) |
| `e_error_m` | 0.12 | Error de seguimiento que se reporta como error (menor que `e_falla_m` = 0.15 del CC) |
| `t_link_s` | 1.0 | Sin `/joint_states` por más de esto = enlace caído |
| `t_traj_s` | 3.0 | Sin estado de `trajectory_publisher` = nodo caído |
| `t_hb_s` | 3.0 | Sin latido del brain = parada segura (igual que en el documento) |
| `t_arranque_s` | 2.0 | Tiempo para ver la trayectoria corriendo tras iniciar |
| `t_min_finalizo_s` | 0.5 | Mínimo publicando "finalizó" antes de pasar a "esperando" |
| `t_asentar_s` | 15.0 | Tiempo máximo para quedar en home al terminar o abortar |
| `periodo_estado_s` | 1.0 | Cada cuánto se repite el estado |

El tiempo que tarda una etapa es el del CSV dividido entre `escala_velocidad` (16.9 s con 1.0). Debe ser **menor que `t_tarea[0]` del brain** (30 s por defecto); si cambias la escala, ajusta `t_tarea` en `Brain/.../brain_params.yaml`.

## Pruebas

**Sin ROS (lógica del Robot 1):**
```bash
cd Robot1/codigo_fuente/antropomorfico_ws/src/robot_control
python3 -m pytest test -v
```

**Con ROS, robot solo** (con `robot1_planta.launch.py` corriendo; simula al brain):
```bash
ros2 topic echo /robot_1/estado &
ros2 topic pub --once /robot_1/cmd planta_interfaces/msg/RobotCmd "{robot_id: 1, caja_id: 17, tarea: 'T1', medicamento: '', seq: 41}"
# en medio de la trayectoria:
ros2 topic pub --once /robot_1/cmd planta_interfaces/msg/RobotCmd "{robot_id: 1, caja_id: 0, tarea: 'PAUSAR', medicamento: '', seq: 42}"
ros2 topic pub --once /robot_1/cmd planta_interfaces/msg/RobotCmd "{robot_id: 1, caja_id: 0, tarea: 'REANUDAR', medicamento: '', seq: 43}"
ros2 topic pub --once /robot_1/cmd planta_interfaces/msg/RobotCmd "{robot_id: 1, caja_id: 0, tarea: 'ABORTAR', medicamento: '', seq: 44}"
```
Debes ver: 1 al aceptar, 2 al terminar y 0 al quedar libre. Para ver el estado de la trayectoria: `ros2 topic echo /trajectory_publisher/estado`.

**Con el brain:** en el workspace del brain, `ros2 launch brain brain.launch.py` y un `ros2 service call /gui/iniciar_caja planta_interfaces/srv/IniciarCaja "{caja_id: 17, medicamento: 'AB'}"`. Si los otros robots aún no existen, lanza los falsos dejando solo `[2, 3, 4, 5]` en `robots_falsos_params.yaml`.

Antes de la primera prueba con el robot real, hazla con `escala_velocidad` baja (por ejemplo 0.3) y la mano cerca del paro.

## Cosas a tener en cuenta

1. **Home:** el robot debe estar en home (error menor a 2 cm) antes de recibir `T1`, igual que ya exige `kinematic_control` para armarse.
2. **Falla del CC:** si `kinematic_control` entra en falla (error > 0.15 m), queda detenido hasta que llames su servicio. Si pasa, el Robot 1 reporta 3 y `ABORTAR` no podrá terminar (el robot no avanza). Resuélvelo así antes de pulsar Continuar en la GUI:
   ```bash
   ros2 service call /kinematic_control/reset std_srvs/srv/Trigger "{}"
   ```
   y deja el robot físicamente en home.
3. **`ABORTAR` es lento si el robot está lejos del home:** vuelve a 0.05 m/s (`vel_retorno_m_s`). Desde la pila o la línea son unos 7 s. `t_asentar_s` (15 s) debe cubrirlo; si subes la distancia o bajas la velocidad, aumenta `t_asentar_s`.
4. **La garra durante `ABORTAR`:** conserva el estado que tenía (si estaba cerrada con una pieza, sigue cerrada). Hay que abrirla a mano si hace falta.
5. **Corrida única:** `T1` supone que el CSV termina en el mismo punto donde empieza (se verificó: home a home).
6. **Lo que no se pudo probar aquí:** el robot físico, la ESP32 y la red real. La lógica y los nodos se probaron con simuladores. La primera prueba real puede requerir ajustar `e_error_m` y `t_asentar_s`.
