# Brain

Información referente al Brain de la planta, encargado de orquestar y organizar a los Robots para sincronizarlos.

El brain es un nodo ROS 2 en Python (`rclpy`). Recibe de la GUI las cajas a ensamblar, asigna cada etapa a los 5 robots (uno por etapa) según los estados que estos publican, y devuelve a la GUI el avance, las alertas y el estado de la planta. Implementa la propuesta `propuesta_brain.pdf`.

## Contenido

| Ruta | Descripción |
|---|---|
| `codigo_fuente/planta_ws/src/planta_interfaces/` | Mensajes y servicio compartidos (`RobotCmd`, `RobotEstado`, `Progreso`, `Alerta`, `EstadoPlanta`, `IniciarCaja`). **Todos los grupos (robots y GUI) deben usar este paquete.** |
| `codigo_fuente/planta_ws/src/brain/brain/brain_node.py` | Nodo ROS 2 del brain |
| `codigo_fuente/planta_ws/src/brain/brain/brain_core.py` | Lógica del brain sin ROS (cajas, reintentos, fallos, parada global) |
| `codigo_fuente/planta_ws/src/brain/brain/robots_falsos.py` | Robots de prueba que cumplen el contrato, para probar sin los otros grupos |
| `codigo_fuente/planta_ws/src/brain/config/brain_params.yaml` | **Tiempos y reintentos (aquí se ajustan)** |
| `codigo_fuente/planta_ws/src/brain/test/` | Pruebas unitarias (no necesitan ROS) |

## Requisitos

- ROS 2 Jazzy sobre Ubuntu 24.04 (supuesto; el código usa solo `rclpy`, `std_msgs` y `std_srvs`). La lógica está probada con pruebas unitarias; la compilación con `colcon` y la prueba en red real quedan por verificar en tu equipo.
- Todos los equipos en la misma red del router y con el **mismo `ROS_DOMAIN_ID`** (valor por definir entre los grupos).

## Compilar

```bash
cd Brain/codigo_fuente/planta_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select planta_interfaces brain
source install/setup.bash
```

Con `--symlink-install`, editar `brain_params.yaml` no obliga a recompilar.

## Ejecutar

Brain solo (cuando los robots reales ya están conectados):

```bash
export ROS_DOMAIN_ID=<el acordado>
ros2 launch brain brain.launch.py
```

Brain con robots falsos (para probar sin los grupos reales):

```bash
ros2 launch brain prueba.launch.py
```

Si algunos robots son reales, edita `config/robots_falsos_params.yaml` y deja en `robots:` solo los que quieras simular (por ejemplo `[2, 3, 4, 5]` si el robot 1 es el real) y lanza cada nodo por separado:

```bash
ros2 run brain brain_node --ros-args --params-file src/brain/config/brain_params.yaml
ros2 run brain robots_falsos --ros-args --params-file src/brain/config/robots_falsos_params.yaml
```

## Parámetros ajustables (`brain_params.yaml`)

| Parámetro | Valor inicial | Qué hace |
|---|---|---|
| `t_ack` | 2.0 s | Espera del estado 1 (ejecutando) tras cada intento de una orden |
| `max_intentos` | 3 | Intentos de una orden antes de declarar fallo |
| `t_tarea` | 30.0 s por robot (lista de 5) | Tiempo máximo de la etapa de cada robot. Se ajusta midiendo cada robot |
| `t_offline` | 3.0 s | Sin estado de un robot por más de esto = desconectado |
| `periodo_latido` | 1.0 s | Latido `/brain/latido` (los robots caen a parada segura si pasan T_hb = 3 s sin él) |
| `periodo_estado` | 1.0 s | Publicación de `/planta/estado` |
| `periodo_tick` | 0.1 s | Revisión interna de reintentos, tiempos y despacho |

Los parámetros se leen al arrancar: para cambiar un tiempo, edita el YAML y reinicia el brain. Usa siempre decimales en los tiempos (`30.0`, no `30`).

## Interfaces (contrato con robots y GUI)

| Interfaz | Tipo | Sentido | QoS |
|---|---|---|---|
| `/robot_N/cmd` | `RobotCmd` | brain → robot N | confiable, profundidad 10 |
| `/robot_N/estado` | `RobotEstado` | robot N → brain | confiable, profundidad 10 |
| `/brain/latido` | `std_msgs/UInt32` | brain → robots (cada 1 s) | confiable, profundidad 1 |
| `/planta/estop` | `std_msgs/Bool` | GUI → brain y robots | confiable + transient local |
| `/gui/iniciar_caja` | servicio `IniciarCaja` | GUI → brain | |
| `/gui/continuar` | servicio `std_srvs/Trigger` | GUI → brain | |
| `/planta/progreso` | `Progreso` | brain → GUI | confiable, profundidad 10 |
| `/planta/alerta` | `Alerta` | brain → GUI | confiable, profundidad 10 |
| `/planta/estado` | `EstadoPlanta` (cada 1 s) | brain → GUI | confiable, profundidad 10 |

Órdenes del brain a un robot (`RobotCmd.tarea`): `T1` a `T5` (la de ese robot), `PAUSAR`, `REANUDAR`, `ABORTAR`. Estados del robot: 0 esperando, 1 ejecutando (hace de confirmación), 2 finalizó, 3 error.

## Cómo se comporta

- **Flujo:** la caja k pasa al robot i cuando el robot i-1 finalizó la caja k y el robot i está esperando. Puede haber varias cajas en la línea. Cada etapa completada publica un `Progreso` (20, 40, 60, 80, 100 %).
- **Medicamento:** `IniciarCaja` valida AA, AB o BB (acepta minúsculas). Solo el robot 2 lo recibe; los demás reciben `medicamento` vacío.
- **Rechazo de cajas:** `aceptada: false` con el motivo si la planta no está en OPERANDO, si `caja_id` es 0 o ya se usó, o si el medicamento no es válido.
- **Confirmación y reintentos:** si el robot no responde con estado 1 en `t_ack`, el brain reenvía la misma orden con el **mismo `seq`**; tras `max_intentos` declara fallo.
- **Fallos** (no confirma, no termina en `t_tarea`, estado 3, desconexión con trabajo): alerta a la GUI, planta en `FALLO` y `PAUSAR` a todos los robots. Un robot desconectado **sin** trabajo solo genera alerta.
- **Continuar** (`/gui/continuar`): salta la etapa fallida (la caja pasa al siguiente robot sin sumar 20 %, así que nunca llega a 100 %), envía `ABORTAR` al robot que falló y `REANUDAR` a los demás. Cuando una caja termina con etapas saltadas se publica una alerta con `robot_id` 0.
- **Parada global:** `/planta/estop = true` pasa la planta a `PARADA_GLOBAL` y manda `PAUSAR`. Para reanudar hay que liberar (`false`) **y luego** llamar `/gui/continuar`; si la parada sigue activa el brain rechaza el continuar.
- **Tiempo en pausa:** mientras la planta está en FALLO o PARADA_GLOBAL no corren `t_ack` ni `t_tarea`, para no declarar fallos falsos al reanudar.
- **Latido:** el brain publica `/brain/latido` siempre, también en FALLO y PARADA_GLOBAL.

### Decisiones donde el documento era ambiguo (revisar con los demás grupos)

1. `seq` es un contador global que empieza en 1 y crece con cada orden y cada comando (`PAUSAR`, `REANUDAR`, `ABORTAR`). El brain ignora estados 1/2 cuyo `seq` o `caja_id` no coincida con la orden vigente.
2. `porcentaje` = 20 × etapas completadas (una etapa saltada no suma). `etapa` es el número del robot que terminó.
3. `cajas_en_cola[i]` = cantidad de cajas esperando al robot i+1. `cajas_robots[i]` = caja que tiene el robot (0 si ninguna).
4. Un robot sin comunicación se muestra en `estados_robots` como 3 (error).
5. `max_intentos = 3` cuenta el envío original (1 envío + 2 reenvíos).
6. Tras `continuar`, un robot al que se le mandó `ABORTAR` queda disponible solo cuando publique un nuevo estado 0.
7. Si un robot ya desconectado sigue sin volver tras `continuar`, sus cajas esperan en cola hasta que reaparezca.

## Pruebas

**Unitarias (sin ROS):**

```bash
cd Brain/codigo_fuente/planta_ws/src/brain
python3 -m pytest test -v
```

Cubren flujo normal, cajas en paralelo, medicamento, reintentos, fallos, continuar con salto de etapa, desconexión, parada global, tiempo en pausa y mensajes duplicados.

**Manuales con ROS** (con `prueba.launch.py` corriendo, en otra terminal con `source install/setup.bash`):

```bash
# Iniciar una caja
ros2 service call /gui/iniciar_caja planta_interfaces/srv/IniciarCaja "{caja_id: 17, medicamento: 'AB'}"
ros2 topic echo /planta/progreso
ros2 topic echo /planta/estado

# Parada global, liberar y continuar
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local /planta/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local /planta/estop std_msgs/msg/Bool "{data: false}"
ros2 service call /gui/continuar std_srvs/srv/Trigger "{}"

# Provocar fallos con los robots falsos
ros2 param set /robots_falsos robot_mudo 3                       # el robot 3 no confirma
ros2 param set /robots_falsos robot_error 4                      # el robot 4 reporta error...
ros2 param set /robots_falsos caja_error 17                      # ...al ejecutar la caja 17
ros2 topic echo /planta/alerta
```

Para probar un robot real, simula al brain desde consola:

```bash
ros2 topic pub --once /robot_1/cmd planta_interfaces/msg/RobotCmd "{robot_id: 1, caja_id: 17, tarea: 'T1', medicamento: '', seq: 41}"
ros2 topic echo /robot_1/estado
```

## Problemas frecuentes

- **No se ven los tópicos entre equipos:** revisar que todos tengan el mismo `ROS_DOMAIN_ID` y que el router no bloquee multicast.
- **Un robot aparece siempre en 3 en `/planta/estado`:** el brain no recibe su `/robot_N/estado` (red, dominio o QoS distinto a confiable).
- **`InvalidParameterTypeException` al arrancar:** algún tiempo del YAML está como entero (`30`); escribirlo como `30.0`.
- **El robot real nunca recibe órdenes:** debe estar suscrito a `/robot_N/cmd` con QoS confiable y usar el paquete `planta_interfaces` compilado de este mismo workspace.
