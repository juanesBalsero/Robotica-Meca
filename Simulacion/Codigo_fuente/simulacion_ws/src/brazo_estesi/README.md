# SCARA RRP en ROS 2 Jazzy y Gazebo Harmonic

Paquete ROS 2 para simular un manipulador SCARA de tres grados de libertad
con configuración R-R-P. Incluye el modelo Xacro, mallas STL, integración con
`gz_ros2_control`, control de trayectoria, mundo con dos estaciones visuales
y un nodo que ejecuta una trayectoria desde CSV.

## Entorno probado

- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic 8
- `gz_ros2_control`

## Articulaciones

| Junta | Tipo | Límite de posición | Velocidad máxima |
|---|---|---:|---:|
| M1 | Revoluta | −3.1416 a 3.1416 rad | 2.0 rad/s |
| M2 | Revoluta | −2.8 a 2.8 rad | 2.0 rad/s |
| M3 | Prismática | 0 a 0.05 m | 0.05 m/s |
| M4 | Fija | Pinza visual rígida | — |

La pinza está incluida como una malla rígida. No abre, no cierra y no sujeta
objetos físicamente.

## Contenido

```text
config/controllers.yaml                 Controladores ros2_control
launch/gazebo.launch.py                 Lanzamiento completo de Gazebo
meshes/                                 Mallas del robot
scripts/ejecutar_trayectoria_csv.py     Ejecutor de trayectoria
trajectories/referencias_propuesta.csv Referencias articulares
trajectories/referencias_propuesta_antes_tcp.csv Respaldo previo a calibrar el TCP
urdf/brazo_estesi.urdf.xacro            Modelo del robot
worlds/empty.sdf                        Mundo y estaciones visuales
```

## Instalación

Instale ROS 2 Jazzy y los paquetes necesarios:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher
```

Cree un workspace y clone el repositorio dentro de `src`:

```bash
mkdir -p ~/brazo_ws/src
cd ~/brazo_ws/src
git clone URL_DEL_REPOSITORIO brazo_estesi
cd ~/brazo_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select brazo_estesi
source install/setup.bash
```

## Ejecutar la simulación

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/brazo_ws/install/setup.bash
ros2 launch brazo_estesi gazebo.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/brazo_ws/install/setup.bash
ros2 control list_controllers
ros2 run brazo_estesi ejecutar_trayectoria_csv.py
```

Los controladores `brazo_controller` y `joint_state_broadcaster` deben aparecer
en estado `active` antes de enviar la trayectoria.

## Trayectoria

El CSV contiene 1641 muestras, con periodo de 0.01 s y duración de 16.4 s.
Incluye posiciones, velocidades y aceleraciones de M1, M2 y M3, además del
número de tramo y el estado lógico de la pinza.

La trayectoria fue recalculada para que el frame `tcp_gripper`, ubicado en el
centro geométrico de la pinza, llegue al centro de cada estación. No debe
interpretarse la posición del eje M3 como la posición final de agarre. La
velocidad del primer y último punto se fija exactamente en cero, como exige
el `joint_trajectory_controller` de ROS 2 Jazzy.

Para esta escena, M3 se limita a `0.020 m`. Este valor evita que la barra
superior de la pieza en T de L4 entre en la carcasa de L3 y también mantiene
los dedos por encima de las mesas.

Las estaciones reproducen las cotas fijas del plano físico tomando a Rob3
como origen: Est1 está en `(-0.19, 0.00) m`, Est2 en `(0.18, 0.00) m`, ambas
miden `0.15 × 0.15 m` y su superficie está a `0.10 m` del suelo. Estas poses
no deben ajustarse para compensar errores del TCP o de la trayectoria.

Antes de enviar el CSV, el nodo:

1. valida columnas, tiempos y límites articulares;
2. espera `/joint_states` y el servidor de acciones;
3. aproxima el robot al primer punto durante 10 s;
4. envía la trayectoria al `joint_trajectory_controller`.

## Limitaciones

- La pinza es solamente visual.
- Las mesas son elementos visuales sin colisiones.
- No se implementó transporte físico de la caja.
- La validación geométrica se limita a una zona segura de M2 y a un descenso
  máximo de M3; no se utiliza MoveIt 2 para planificar colisiones.

## Licencia

BSD-3-Clause. Consulte [LICENSE](LICENSE).
