# Workspace Multi-Robot — Simulación y Control en ROS 2

Workspace integral de **ROS 2** orientado al modelado URDF/Xacro, simulación física en Gazebo, visualización en RViz2 y control cinemático/de trayectorias de múltiples manipuladores robóticos (configuraciones antropomórficas y SCARA), además de un paquete maestro de despliegue conjunto (`robot_bringup`).

---

## 🧠 Funcionamiento General

El workspace articula distintos paquetes independientes para cada manipulador junto con un paquete central de orquestación:

1. **Orquestación de Escena (`robot_bringup`)**: Levanta el entorno de simulación en Gazebo (`empty_world.sdf`) e integra los robots de la celda en un único lanzamiento maestro (`bringup.launch.py`).
2. **Simulación Física y Control Articular (`ros2_control` + Gazebo)**: Cada paquete define la descripción cinemática y dinámica de su robot mediante archivos `.urdf` o `.urdf.xacro`, mallas `.STL` y controladores configurados en `ros2_controllers.yaml`.
3. **Planificación Cinemática y Seguimiento de Trayectorias**:
   - **Control en C++ (`miau`)**: Implementa nodos de cinemática diferencial inversa (`diff_ik_planner.hpp`), cálculo de Jacobiano (`nodo_jacobiano.cpp`) y lazos de realimentación (`realimentacion.hpp`).
   - **Control y Generación de Trayectorias en Python/MATLAB**: Incluye interpoladores trapezoidales (`trapezoidal_trajectory_node.py`), reproductores de trayectorias precalculadas en `.csv` y `.xlsx`, cálculo de cinemática directa/inversa (`control_cinematico.py`, `fk_publisher.py`), interfaces gráficas con sliders (`gui_sliders.py`) y scripts de generación en MATLAB (`trayectoria_scara.m`).

---

## 📁 Estructura y Contenidos del Workspace

```text
src/
├── robot_bringup/           # Paquete maestro para levantar la simulación conjunta
├── antropomorfico_sim/      # Brazo antropomórfico con pinza de dos dedos y perfil trapezoidal
├── brazo_estesi/            # Manipulador de 4 GDL con gripper y ejecución de trayectorias CSV
├── miau/                    # Manipulador con control cinemático y Jacobiano en C++ + RViz2
├── scara_boceto/            # Robot SCARA con control cinemático en Python, GUI y trayectorias Excel
└── scara_description/       # Robot SCARA con referencia CAD (.STEP), script MATLAB y soporte Gazebo Classic/Gz
```

### Detalle por Paquete

#### 1. `robot_bringup` (Orquestación General)
* `launch/bringup.launch.py`: Archivo de lanzamiento principal para iniciar el entorno completo de simulación.
* `worlds/empty_world.sdf`: Mundo base de Gazebo para la escena multi-robot.
* `test/`: Pruebas automatizadas de estilo y licencia (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`).

#### 2. `antropomorfico_sim` (Manipulador Antropomórfico)
* `urdf/antropomorfico.urdf` y `meshes/`: Modelo del brazo (`base_link`, `link_1`–`link_3`, `link_ee`) y pinza (`dedoizq.STL`, `dedoder.STL`).
* `launch/sim.launch.py`: Inicia el modelo en simulación junto con sus controladores (`config/ros2_controllers.yaml`).
* `antropomorfico_sim/trapezoidal_trajectory_node.py`: Generador de trayectorias con perfil de velocidad trapezoidal.
* `antropomorfico_sim/trayectoria_publisher_node.py`: Publicador de puntos desde `config/trayectoria_robot.csv`.
* `test/`: Pruebas de calidad de código (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`).

#### 3. `brazo_estesi` (Manipulador 4 GDL + Gripper)
* `urdf/brazo_estesi.urdf.xacro` y `meshes/`: Eslabones `L1.STL`–`L4.STL` y efector `GRIPPER.STL`.
* `launch/gazebo.launch.py`: Despliega el robot en el mundo `worlds/empty.sdf` con `config/ros2_controllers.yaml`.
* `scripts/ejecutar_trayectoria_csv.py`: Ejecuta las trayectorias almacenadas en `trajectories/referencias_propuesta.csv` y `referencias_propuesta_antes_tcp.csv`.

#### 4. `miau` (Control Cinemático y Jacobiano en C++)
* `urdf/Completo.urdf`, `urdf/Completo.csv` y `meshes/`: Descripción del manipulador (`base_link`, `L1`–`L3` y texturas).
* `include/miau/`: Cabeceras `diff_ik_planner.hpp` (cinemática inversa diferencial) y `realimentacion.hpp`.
* `src/`: Nodos compilados en C++ (`nodo.cpp`, `nodo_cinematico.cpp`, `nodo_jacobiano.cpp`, `vti2.cpp`).
* `config/`: Parámetros en `cinematico.yaml`, `trayectoria.yaml` y `ros2_controllers.yaml`.
* `launch/`: `gazebo.launch.py`, `cinematico.launch.py`, `trayectoria.launch.py` y `launch.py`.
* `rviz/config.rviz`: Configuración predefinida de visualización en RViz2.

#### 5. `scara_boceto` (SCARA con Control Cinemático y GUI)
* `urdf/scara_boceto.urdf.xacro`, `urdf/final.urdf` y `meshes/`: Componentes del SCARA (`base_link`, `rot1`, `rot2`, `prisma`, `efector` y `caja.STL`).
* `launch/`: `display.launch.py` (visualización URDF) y `gazebo.launch.py` (simulación física).
* `scara_boceto/`:
  * `control_cinematico.py`: Control cinemático del manipulador.
  * `fk_publisher.py`: Publicación de cinemática directa.
  * `gui_sliders.py`: Interfaz gráfica con deslizadores para control manual de articulaciones.
  * `trajectory_rviz.py`: Visualización y seguimiento de trayectorias en RViz2 usando `tray1.xlsx`–`tray4.xlsx`.
* `test/`: Pruebas de linter y formato (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`).

#### 6. `scara_description` (SCARA con Integración CAD y MATLAB)
* `urdf/scara.urdf.xacro` y `meshes/`: Modelo del SCARA (`base_link.stl`, `link_1.stl`–`link_3.stl`).
* `cad_reference/base_A_original.STEP`: Modelo CAD paramétrico de referencia.
* `matlab/trayectoria_scara.m`: Script de cálculo de trayectorias que exporta `config/trayectoria_scara.csv`.
* `launch/`: Soporte para RViz (`display.launch.py`), Gazebo Classic (`gazebo_classic.launch.py`) y Gazebo Sim (`gazebo_gz.launch.py`).
* `scripts/`: `send_trajectory.py` (envío de comandos) y `log_joint_states.py` (registro de estados articulares).

---

## 🛠️ Compilación del Workspace

Desde la raíz del workspace (donde se encuentra la carpeta `src/`):

```bash
# Compilar todos los paquetes del workspace
colcon build --symlink-install

# Cargar el entorno compilado en la terminal actual
source install/setup.bash
```

---

## 🚀 Cómo Correr las Simulaciones

### 1. Lanzamiento Maestro de la Celda (`robot_bringup`)
Para iniciar el entorno integrado en Gazebo:

```bash
ros2 launch robot_bringup bringup.launch.py
```

### 2. Simulación Individual por Robot

#### A. Brazo Antropomórfico (`antropomorfico_sim`)
```bash
# Terminal 1: Lanzar simulación y controladores
ros2 launch antropomorfico_sim sim.launch.py

# Terminal 2: Ejecutar trayectoria trapezoidal o desde CSV
ros2 run antropomorfico_sim trapezoidal_trajectory_node
# o bien:
ros2 run antropomorfico_sim trayectoria_publisher_node
```

#### B. Brazo Estesi (`brazo_estesi`)
```bash
# Terminal 1: Lanzar simulación en Gazebo
ros2 launch brazo_estesi gazebo.launch.py

# Terminal 2: Ejecutar trayectoria desde archivo CSV
ros2 run brazo_estesi ejecutar_trayectoria_csv.py
```

#### C. Manipulador Miau (`miau`)
```bash
# Terminal 1: Lanzar entorno Gazebo y RViz2
ros2 launch miau gazebo.launch.py

# Terminal 2: Lanzar nodos de control cinemático o trayectoria
ros2 launch miau cinematico.launch.py
# o bien:
ros2 launch miau trayectoria.launch.py
```

#### D. SCARA Boceto (`scara_boceto`)
```bash
# Visualización en RViz2
ros2 launch scara_boceto display.launch.py

# Simulación en Gazebo
ros2 launch scara_boceto gazebo.launch.py
```

#### E. SCARA Description (`scara_description`)
```bash
# Lanzar en Gazebo Sim (Ignition/Gz)
ros2 launch scara_description gazebo_gz.launch.py

# O en Gazebo Classic
ros2 launch scara_description gazebo_classic.launch.py

# Enviar trayectoria y registrar estados articulares (en terminales separadas)
ros2 run scara_description send_trajectory.py
ros2 run scara_description log_joint_states.py
```

---

## 🧪 Cómo Correr las Pruebas

Los paquetes `antropomorfico_sim`, `robot_bringup` y `scara_boceto` incluyen suites de pruebas unitarias y de verificación de estilo (`flake8`, `pep257`, `copyright`). Para ejecutarlas:

```bash
# Ejecutar todas las pruebas del workspace
colcon test

# Ver el resumen detallado de resultados
colcon test-result --verbose
```

Si deseas probar un paquete específico:

```bash
colcon test --packages-select antropomorfico_sim robot_bringup scara_boceto
colcon test-result --all
```