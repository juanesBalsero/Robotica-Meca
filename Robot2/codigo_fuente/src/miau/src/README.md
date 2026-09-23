# Paquete ROS 2: miau

Nodos de control cinemático y generación de trayectorias para el manipulador de 3 GDL (Robot2) utilizando **Orocos KDL**.

## 📁 Contenido del Paquete

* **`nodo.cpp`**: Genera una trayectoria cartesiana recta (Home ➔ Medicamento A) mediante *waypoints* discretos y resuelve la cinemática inversa (IK) con el algoritmo LMA.
* **`vti2.cpp`**: Genera una trayectoria con **perfil trapezoidal de velocidad** (aceleración - meseta - desaceleración) sincronizando todas las articulaciones para que inicien y terminen al mismo tiempo.

## ⚙️ Compilación

Desde la raíz de tu espacio de trabajo (`codigo_fuente`):

```bash
colcon build --packages-select miau
source install/setup.bash


# Executar trayectoria por waypoints
ros2 run miau nodo

# Ejecutar trayectoria con perfil trapezoidal
ros2 run miau vti2

Tópicos
Publica: /position_controller/commands (std_msgs/msg/Float64MultiArray) — Comandos de posición angular.

Suscrito: /joint_states (sensor_msgs/msg/JointState) — Lectura de estado real de las articulaciones.

Archivos de Salida (Logs CSV)
Ambos nodos exportan datos en archivos .csv marcados con tiempo relativo transcurrido (t_sec) para su posterior graficación:

trajectory_log.csv / trajectory_real.csv: Posición, velocidad y esfuerzo real de /joint_states.

trajectory_planned.csv: Trayectoria teórica calculada por el perfil trapezoidal.