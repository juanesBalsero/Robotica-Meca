# SCARA R-R-P — ROS 2 Jazzy / Gazebo Harmonic

## Estado
- J1 revoluta
- J2 revoluta, limite medido +/-1.750 rad para evitar choque
- J3 prismatica, +/-84 mm
- Collision meshes incluidas
- gz_ros2_control + joint_trajectory_controller incluidos
- MATLAB genera trayectoria cartesiana recta, IK, velocidades y CSV
- ROS 2 envia el CSV a Gazebo

## IMPORTANTE
Las masas e inercias actuales son aproximadas. Las velocidades si son utiles para la entrega; el torque/esfuerzo de Gazebo debe llamarse estimacion hasta reemplazar masas e inercias por las propiedades reales de SolidWorks.

## Instalar dependencias en ROS 2 Jazzy
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

## Compilar
```bash
cd ~/scara_ws
rm -rf build install log
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/scara_ws/install/setup.bash
```

## RViz
```bash
ros2 launch scara_description display.launch.py
```

## Gazebo Harmonic + control
```bash
ros2 launch scara_description gazebo_gz.launch.py
```

Comprobar controladores:
```bash
ros2 control list_controllers
```
Deben aparecer `joint_state_broadcaster` y `arm_controller` como `active`.

## Trayectoria
1. Abrir `matlab/trayectoria_scara.m` en MATLAB.
2. El limite `q2_lim` ya esta configurado en +/-1.750 rad. Cambiarlo solo si se vuelve a medir.
3. Ejecutar. Se crea `trayectoria_scara.csv`.
4. Copiar el CSV a:
   `~/scara_ws/src/scara_description/config/trayectoria_scara.csv`
5. Con Gazebo abierto:
```bash
ros2 run scara_description send_trajectory.py ~/scara_ws/src/scara_description/config/trayectoria_scara.csv
```

## Registrar posiciones, velocidades y esfuerzo
En otra terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/scara_ws/install/setup.bash
ros2 run scara_description log_joint_states.py
```
Se guarda `~/scara_joint_states.csv`.
