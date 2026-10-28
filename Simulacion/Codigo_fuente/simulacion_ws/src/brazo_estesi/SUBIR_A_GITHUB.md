# Cómo publicar este proyecto en GitHub

## 1. Crear el repositorio

En GitHub, cree un repositorio vacío llamado `brazo-estesi-ros2`. No seleccione
README, licencia ni `.gitignore`, porque estos archivos ya están incluidos.

## 2. Subir la carpeta

Descomprima el archivo recibido y abra un terminal dentro de la carpeta
`brazo-estesi-ros2`. Luego ejecute:

```bash
git init
git branch -M main
git add .
git commit -m "Simulacion SCARA en ROS 2 Jazzy y Gazebo Harmonic"
git remote add origin URL_DEL_REPOSITORIO
git push -u origin main
```

Sustituya `URL_DEL_REPOSITORIO` por la dirección mostrada por GitHub, por
ejemplo `https://github.com/USUARIO/brazo-estesi-ros2.git`.

Si Git solicita identidad antes del `commit`:

```bash
git config user.name "Nombre Apellido"
git config user.email "correo@ejemplo.com"
```

## 3. Verificación rápida

Después de clonar el repositorio en Ubuntu 24.04 con ROS 2 Jazzy:

```bash
mkdir -p ~/brazo_ws/src
cd ~/brazo_ws/src
git clone URL_DEL_REPOSITORIO brazo_estesi
cd ~/brazo_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select brazo_estesi
source install/setup.bash
ros2 launch brazo_estesi gazebo.launch.py
```

En otro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/brazo_ws/install/setup.bash
ros2 run brazo_estesi ejecutar_trayectoria_csv.py
```
