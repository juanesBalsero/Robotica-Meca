# Paquete ROS 2: `miau` (Directorio `miauvf`)

> **Nota:** El directorio se denomina `miauvf` (versión final), pero el nombre registrado en `package.xml` y `CMakeLists.txt` es **`miau`**.

## Componentes (`ament_cmake`, C++17 / Python 3)
- `urdf/Completo.urdf` y `meshes/`: Modelo cinemático/inercial (`J1`, `J2`, `J3`, `caja_obstaculo`) y mallas `.STL`.
- `src/nodo_jacobiano.cpp` y `include/miau/diff_ik_planner.hpp`: Cinemática diferencial inversa ($\dot{q} = J^{-1}(q)\,\vec{v}_e$), dinámica inversa KDL (RNE) y exportación CSV.
- `src/nodo_cinematico.cpp` y `include/miau/realimentacion.hpp`: Control punto a punto (`/comando_trayectoria`) con lazo proporcional (`kp_joint: 1.12`).
- `scripts/esp32_robot_bridge.py`: Puente serial (`/dev/ttyUSB0`) que aplica `config/hw_calibration.yaml` ($\text{rad/s} \leftrightarrow \text{RPM}$) y publica `/joint_states`.
- `scripts/gui_miau.py`: Interfaz gráfica Tkinter para operación (`PH`, `PA`, `PB`, `P1`, `P2`), pinza (`rawr`) y pruebas manuales.
- `config/`: Parámetros de controladores (`controllers.yaml`), trayectorias (`trayectoria.yaml`, `cinematico.yaml`) y hardware (`hw_calibration.yaml`).

## Compilación y Ejecución

    colcon build --packages-select miau --symlink-install
    source install/setup.bash
    ros2 launch miau robot_miau.launch.py serial_port:=/dev/ttyUSB0 use_rviz:=true

Otros lanzadores: `trayectoria.launch.py` (Gazebo + Jacobiano), `gazebo.launch.py` (simulación base) y `launch.py` (RViz2).
