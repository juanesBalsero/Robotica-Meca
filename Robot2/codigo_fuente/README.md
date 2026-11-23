# Código Fuente — Robot 2 (`miau`)

Software de control en ROS 2 y firmware embebido en MicroPython del manipulador antropomórfico de 3 GDL (**Robot 2**).

## Estructura
- [`src/miauvf/`](src/miauvf/): Paquete ROS 2 (`miau`, C++17 y Python 3) con modelo URDF, simulación en Gazebo (`gz_ros2_control`), planificadores cinemáticos KDL (`nodo_jacobiano`, `nodo_cinematico`), GUI en Tkinter (`gui_miau.py`) y puente serial (`esp32_robot_bridge.py`).
- [`firmware_esp32/`](firmware_esp32/): Código MicroPython para las 3 placas ESP32 comunicadas por **ESP-NOW (Canal 1)** (`main_ant.py`, `main_dof1_2.py`, `main_dof3.py`).

## Compilación Rápida (ROS 2)

    colcon build --packages-select miau --symlink-install
    source install/setup.bash
