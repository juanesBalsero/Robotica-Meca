# Desarrollo local

Después de modificar el Xacro, controladores, mundo o scripts:

```bash
cd ~/brazo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select brazo_estesi
source install/setup.bash
```

Validaciones recomendadas:

```bash
xacro src/brazo_estesi/urdf/brazo_estesi.urdf.xacro -o /tmp/brazo_estesi.urdf
check_urdf /tmp/brazo_estesi.urdf
python3 -m py_compile src/brazo_estesi/scripts/ejecutar_trayectoria_csv.py
gz sdf -k src/brazo_estesi/worlds/empty.sdf
```

No incluya `build/`, `install/`, `log/`, `__pycache__/` ni respaldos locales
en los commits.

