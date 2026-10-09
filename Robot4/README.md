# 🤖 Robot 4 - Manipulador SCARA RRP

Este repositorio contiene la documentación técnica, diseños, modelos y archivos de simulación del **Robot 4**, un manipulador robótico encargado de tomar las tapas desde una mesa auxiliar y posicionarlas sobre las cajas que transitan por la línea principal de la celda de automatización.

---

## 📐 Especificaciones Técnicas

| Parámetro | Valor / Descripción |
| :--- | :--- |
| **Arquitectura** | SCARA RRP (Revoluta - Revoluta - Prismática) |
| **Grados de Libertad (DOF)** | 3 DOF |
| **Velocidad de Junturas Rotacionales ($q_1, q_2$)** | $2\text{ rad/s}$ |
| **Velocidad de Juntura Prismática ($q_3$)** | $15\text{ cm/s}$ ($0.15\text{ m/s}$) |
| **Software CAD** | SolidWorks |

---

## 📂 Contenido del Repositorio

En esta documentación se encuentran disponibles todos los recursos referentes al diseño y construcción del robot:

* **📐 Diseño Mecánico:** Modelos CAD 3D creados en SolidWorks, ensamblajes del manipulador y archivos para simulación/URDF.
* **⚡ Diseño Eléctrico:** Planos de conexionado, esquemas eléctricos y distribución de componentes para el control del robot.
* **📊 Documentación y Simulación:** Archivos de validación cinemática y paquetes para ejecución en **RViz2** y **Gazebo**.

---

## 📁 Estructura del Proyecto

* **`scara_cajas/`**: Entorno de pruebas y validación del modelo matemático, cinemática y trayectorias.
* **`scara_boceto/`**: Versión final del diseño CAD con la geometría fija y ensamblajes listos para simulación.
