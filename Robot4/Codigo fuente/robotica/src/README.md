# Robot 4 - Manipulador SCARA (Tapa de Cajas)

Este repositorio contiene los paquetes de ROS 2, modelos URDF/Xacro y configuraciones para la simulación del **Robot 4**, un manipulador tipo SCARA integrado en la línea principal de automatización y empacamiento de productos farmacéuticos.

El propósito principal del **Robot 4** es tomar una tapa ubicada en una mesa auxiliar y colocarla de manera precisa sobre la caja contenedora correspondiente que avanza por la línea de producción.

---

## 📁 Estructura del Repositorio

El proyecto está dividido en dos carpetas/paquetes principales:

* **`scara_cajas/`**: 
  * **Fase de pruebas y validación matemática.**
  * Contiene los modelos, scripts y configuraciones iniciales para validar la cinemática (directa e inversa), trayectorias y dinámicas del robot junto con los elementos de prueba (cajas y entorno).

* **`scara_boceto/`**:
  * **Diseño final y CAD definitivo.**
  * Contiene la versión con la geometría fija, mallas finales y ensamblaje definitivo del robot configurado para su ejecución directa en entornos de simulación (RViz2 y Gazebo).

---

## 🛠️ Requisitos Previos

Asegúrate de contar con las siguientes herramientas instaladas en tu entorno de desarrollo:

* ROS 2 (Jazzy)
* Gazebo / Ignition Gazebo
* RViz2
* `joint_state_publisher_gui` y `robot_state_publisher`
* Controladores de ROS 2 (`ros2_control`, `ros2_controllers`)

---

## 🚀 Guía de Uso

### 1. Abrir el repositorio

Navega a tu espacio de trabajo de ROS 2 para el robot 4 y dirigirse a la siguiente direccion de carpetas en el gestor de archivos:

```bash
# /Robotica-Meca/Robot4/Codigo fuente/robotica
```
### 2. Compilar el proyecto

En el gestor de arhviso dar click derecho y seleccinar la opción abrir un terminal aqui:


Luego, en la terminal se compilan los proyectos con el siguiente comando:
```bash
# colcon build
```
### 3. Instalar dependencias

Luego, se ejecuta el sigueinte comando
```bash
# source install/setup.bash
```
## 4. Seleccionar proyecto 

Finalemte dependiendo del docuemnto a revisar se realizan los siguientes comandos:

### Para visualizar RVIZ

Para *scara_cajas*:
```bash
# ros2 launch scara_cajas display.launch.py
```

Para *scara_bocetos*:
```bash
# ros2 launch scara_boceto display.launch.py
```
