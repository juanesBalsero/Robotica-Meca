# Brazo Robótico Antropomórfico (`antropomorfico_ws`)

Workspace de **ROS 2** para el modelado, simulación y control de un brazo robótico antropomórfico equipado con un efector final tipo pinza (*gripper*) de dos dedos. El proyecto está dividido en dos dominios independientes: uno dedicado exclusivamente a la **simulación virtual** y otro destinado al **control y operación del robot físico en la vida real**.

## 📋 Tabla de Contenidos

1. [Arquitectura y Funcionamiento](#-arquitectura-y-funcionamiento)
2. [Contenido del Workspace](#-contenido-del-workspace)
3. [Requisitos Previos](#-requisitos-previos)
4. [Cómo Ejecutar la Simulación](#-cómo-ejecutar-la-simulación)
5. [Cómo Ejecutar el Control del Robot Real](#-cómo-ejecutar-el-control-del-robot-real)

## ⚙️ Arquitectura y Funcionamiento

El workspace organiza sus paquetes según el entorno de ejecución al que pertenecen:

### 🖥️ Entorno de Simulación (`antropomorfico_sim`)
Este paquete abarca todo lo necesario para simular el manipulador de forma virtual sin requerir hardware físico:
* **Modelado Cinemático y Visual:** Contiene la descripción URDF del robot (`antropomorfico.urdf`) y las mallas geométricas 3D en formato `.STL` (base, tres eslabones principales, soporte del efector final y dos dedos).
* **Controladores Simulados:** Integra `ros2_control` mediante `ros2_controllers.yaml` para emular el comportamiento articular.
* **Generación de Trayectorias en Simulación:** Incluye nodos exclusivos para probar movimientos en el simulador, ya sea calculando perfiles de velocidad trapezoidales en línea (`trapezoidal_trajectory_node.py`) o reproduciendo puntos predefinidos desde un archivo CSV (`trayectoria_publisher_node.py`).

### 🤖 Entorno del Robot Real (`robot_control` y `robot_interfaces`)
Estos dos paquetes conforman la arquitectura de software destinada al manipulador físico:
* **Interfaces de Comunicación (`robot_interfaces`):** Define los mensajes personalizados de ROS 2 (`TrajectoryPoint.msg`) empleados para transmitir consignas de trayectoria (posición, velocidad, aceleración/tiempo) hacia el sistema de control real.
* **Control y Planta del Manipulador (`robot_control`):** Implementa el núcleo matemático y de control (`robot1_core.py`), encapsulado en un nodo de ROS 2 (`robot1_node.py`) configurado con los parámetros físicos del robot (`params.yaml`). Además, cuenta con su propio publicador de trayectorias (`trajectory_publisher.py`) para comandar el robot físico.

## 📁 Contenido del Workspace

```text
src/
├── antropomorfico_sim/                     # [SIMULACIÓN] Modelo URDF, mallas y trayectorias virtuales
│   ├── antropomorfico_sim/
│   │   ├── __init__.py
│   │   ├── trapezoidal_trajectory_node.py  # Generador de trayectorias con perfil trapezoidal (simulación)
│   │   └── trayectoria_publisher_node.py   # Publicador de trayectorias desde CSV (simulación)
│   ├── config/
│   │   ├── ros2_controllers.yaml           # Configuración de controladores simulados (ros2_control)
│   │   └── trayectoria_robot.csv           # Puntos de trayectoria predefinidos para simulación
│   ├── launch/
│   │   └── sim.launch.py                   # Launch principal del entorno de simulación
│   ├── meshes/                             # Modelos geométricos 3D del manipulador
│   │   ├── base_link.STL                   # Base fija del robot
│   │   ├── link_1.STL                      # Eslabón 1 (cintura/hombro)
│   │   ├── link_2.STL                      # Eslabón 2 (brazo)
│   │   ├── link_3.STL                      # Eslabón 3 (antebrazo)
│   │   ├── link_ee.STL                     # Soporte del efector final
│   │   ├── dedoder.STL                     # Dedo derecho de la pinza
│   │   └── dedoizq.STL                     # Dedo izquierdo de la pinza
│   ├── urdf/
│   │   └── antropomorfico.urdf             # Descripción cinemática, inercial y visual del robot
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── robot_control/                          # [VIDA REAL] Control y operación del robot físico
│   ├── config/
│   │   └── params.yaml                     # Parámetros dinámicos, ganancias y ajustes del robot real
│   ├── launch/
│   │   └── robot1_planta.launch.py         # Launch del nodo de control/planta del robot físico
│   ├── robot_control/
│   │   ├── robot1_core.py                  # Lógica central de control y dinámica
│   │   ├── robot1_node.py                  # Nodo ROS 2 que ejecuta robot1_core
│   │   └── trajectory_publisher.py         # Publicador de consignas de trayectoria para el robot real
│   ├── package.xml
│   └── setup.py
│
└── robot_interfaces/                       # [VIDA REAL] Mensajes personalizados para el control real
    ├── msg/
    │   └── TrajectoryPoint.msg             # Definición de mensaje para puntos de trayectoria
    ├── CMakeLists.txt
    └── package.xml
```

## 🛠️ Requisitos Previos

* **Sistema Operativo:** Ubuntu 22.04 / 24.04 (o entorno compatible con ROS 2).
* **Distribución de ROS 2:** Humble Hawksbill, Iron Irwini o Jazzy Jalisco.
* **Paquetes y librerías:**
  * `ros2_control` y `ros2_controllers`
  * `robot_state_publisher` y `joint_state_publisher`
  * Simulador compatible con `sim.launch.py` (Gazebo / RViz2)
  * `numpy`

> **Nota:** Antes de ejecutar cualquiera de los comandos siguientes, asegúrate de haber cargado el entorno de tu workspace en cada terminal:
> ```bash
> source ~/antropomorfico_ws/install/setup.bash
> ```

## 🖥️ Cómo Ejecutar la Simulación

Todos los comandos de esta sección pertenecen exclusivamente al paquete `antropomorfico_sim` y se ejecutan sin conexión al robot físico.

### 1. Lanzar el entorno de simulación
Inicia la descripción URDF, el visor/simulador y los controladores definidos en `ros2_controllers.yaml`:

```bash
ros2 launch antropomorfico_sim sim.launch.py
```

### 2. Enviar trayectorias en simulación
En una **segunda terminal** (con el entorno del workspace cargado), elige una de las dos opciones disponibles para mover el robot simulado:

* **Opción A — Trayectoria con perfil de velocidad trapezoidal:**
  ```bash
  ros2 run antropomorfico_sim trapezoidal_trajectory_node
  ```

* **Opción B — Reproducción de trayectoria desde `trayectoria_robot.csv`:**
  ```bash
  ros2 run antropomorfico_sim trayectoria_publisher_node
  ```

### Verificación de la simulación
```bash
# Verificar el estado de los controladores simulados
ros2 control list_controllers

# Listar los tópicos activos en simulación
ros2 topic list
```

## 🤖 Cómo Ejecutar el Control del Robot Real

Todos los comandos de esta sección corresponden a los paquetes `robot_control` y `robot_interfaces`, destinados al funcionamiento del manipulador físico.

### 1. Lanzar el nodo de control / planta del robot
Inicia el controlador principal del robot real cargando la configuración de `params.yaml`:

```bash
ros2 launch robot_control robot1_planta.launch.py
```

### 2. Enviar trayectorias al robot físico
En una **segunda terminal** (con el entorno del workspace cargado), ejecuta el publicador de trayectorias encargado de enviar mensajes `robot_interfaces/msg/TrajectoryPoint` al sistema de control:

```bash
ros2 run robot_control trajectory_publisher
```

### Verificación del sistema real
```bash
# Inspeccionar los parámetros cargados en el nodo de control
ros2 param list

# Monitorear los mensajes de trayectoria enviados al robot
ros2 topic list
```