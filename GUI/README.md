# GUI

Interfaz gráfica que permite visualizar datos relevantes sobre el proceso y controlarlo, dentro de la planta de embalaje de medicamentos.

Desde el panel se puede iniciar, detener y activar la parada de emergencia, elegir el medicamento a empacar, ver el progreso de cada robot, graficar variables en el tiempo, contar las cajas que entran y salen, y recibir avisos del sistema.

## Contenido

| Ruta | Descripción |
|---|---|
| [`codigo_fuente/panel_control_gatos.html`](codigo_fuente/panel_control_gatos.html) | Interfaz web (un solo archivo HTML) |
| [`codigo_fuente/panel_gatos/`](codigo_fuente/panel_gatos/) | Paquete ROS 2 con el nodo `nodo_panel` |
| [`codigo_fuente/panel.db`](codigo_fuente/panel.db) | Base de datos SQLite con datos de prueba |
| [`documentacion/DocumentacionGui.pdf`](documentacion/DocumentacionGui.pdf) | Documentación completa de la GUI |

## Inicio rápido

**Modo simulado (sin ROS):** abrir `codigo_fuente/panel_control_gatos.html` en el navegador. Funciona con datos de prueba y necesita conexión a internet para cargar la librería roslibjs.

**Modo real (con ROS 2):**

```
colcon build --packages-select panel_gatos
source install/setup.bash
ros2 run panel_gatos nodo_panel
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Después, en el panel, elegir "Conectar a ROS2 (rosbridge)" y pulsar **Conectar** (por defecto usa `ws://localhost:9090`).

Los tópicos, las tablas de la base de datos y los pendientes están en la [documentación](documentacion/DocumentacionGui.pdf).
