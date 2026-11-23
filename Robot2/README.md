# Robot 2 — Manipulador Antropomórfico de 3 GDL (`miau`)

## 1. Descripción General
**Robot 2** es un brazo robótico antropomórfico de **3 grados de libertad rotacionales (`J1`, `J2`, `J3`)** con pinza servoaccionada (*gripper*). Dentro de la línea de producción principal, toma medicamentos desde los puntos de alimentación (`PA`, `PB`) y los deposita en la caja de empaque (`P1`, `P2`), retornando a reposo (`PH`) y evadiendo obstáculos (`caja_obstaculo`).

## 2. Arquitectura del Sistema
1. **Planificación y Control en ROS 2 (`codigo_fuente/src/miauvf/`):**
   - Modelo URDF (`Completo.urdf`) con mallas CAD (`.STL`), visualización en RViz2 y simulación en Gazebo (`gz_ros2_control` a 100 Hz).
   - Nodos en C++17 (`nodo_jacobiano`, `nodo_cinematico`) basados en KDL para cinemática diferencial inversa ($\dot{q} = J^{-1}(q)\,\vec{v}_e$), dinámica inversa (RNE) y control punto a punto.
   - Interfaz gráfica (`gui_miau.py`) con enclavamientos de seguridad, modo de pruebas manuales y telemetría de encoders en tiempo real.
2. **Red Embebida de 3 × ESP32 (`codigo_fuente/firmware_esp32/`):**
   - Puente serial (`esp32_robot_bridge.py`) que traduce velocidades ($\text{rad/s} \leftrightarrow \text{RPM}$) y calibración (`hw_calibration.yaml`).
   - Comunicación inalámbrica punto a punto mediante **ESP-NOW (Canal 1)** entre la **ESP32 Antena** (`main_ant.py`) y las placas de potencia/control de **J1-J2** (`main_dof1_2.py`) y **J3** (`main_dof3.py`).

## 3. Estructura del Directorio
- [`codigo_fuente/`](codigo_fuente/): Paquete ROS 2 (`src/miauvf/`) y firmware MicroPython (`firmware_esp32/`).
- [`documentacion/`](documentacion/): Diagramas, planos mecánicos y eléctricos, memorias de cálculo, *datasheets*, instructivo de usuario y trazas de simulación.
- `.gitignore`: Exclusión de `build/`, `install/`, `log/`, `__pycache__/` y `*.pyc`.
