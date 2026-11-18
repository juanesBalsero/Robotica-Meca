# Documentación Técnica, Planos y Electrónica — Robot 1

## 📋 Descripción General

Este repositorio contiene el paquete completo de documentación de ingeniería del **Robot 1**. Reúne los planos mecánicos de fabricación y ensamblaje (CAD en formato PDF), el diseño electrónico de la placa de control (esquemático y rutado de PCB) y el estudio de análisis cinemático del manipulador.

---

## 📁 Estructura de Contenidos

```text
Documentacion_Robot1/
├── Análisis cinemático R1.pdf       # Estudio matemático y modelado cinemático del manipulador
├── Esquema_elecrtico_pcb.jpg        # Diagrama esquemático del circuito electrónico
├── Plano_pcb.jpg                    # Diseño de pistas y distribución de componentes (PCB)
└── Planos_Robot1/                   # Carpeta con 31 planos técnicos de piezas y ensamblajes (PDF)
    ├── EnsamblajeAntro4DOF.pdf
    ├── EnsambleBase.pdf
    ├── 1DOF.pdf ... 4DOF.pdf
    └── [Piezas estructurales, soportes, ejes y efector final]
```

---

## 📐 1. Análisis Cinemático

* **`Análisis cinemático R1.pdf`**: Documento técnico que desarrolla el modelado cinemático del manipulador, definiendo los sistemas de referencia articulares, parámetros geométricos, ecuaciones de cinemática directa/inversa y el espacio de trabajo en el plano de operación.

---

## ⚡ 2. Documentación Electrónica (PCB)

Archivos gráficos correspondientes al hardware electrónico de control e interfaz de potencia del robot:

| Archivo | Descripción |
| :--- | :--- |
| **`Esquema_elecrtico_pcb.jpg`** | Diagrama esquemático eléctrico con las conexiones entre microcontrolador, etapas de potencia/actuadores, sensores y alimentación. |
| **`Plano_pcb.jpg`** | Vista del diseño físico de la placa de circuito impreso (PCB), mostrando el enrutado de pistas, vías y ubicación de componentes para manufactura. |

---

## ⚙️ 3. Planos Mecánicos (`Planos_Robot1/`)

La carpeta `Planos_Robot1/` incluye **31 planos de ingeniería** en formato `.pdf`, organizados por subsistemas funcionales:

### 🔹 Ensamblajes Generales y por Grado de Libertad (DOF)
* **`EnsamblajeAntro4DOF.pdf`**: Plano de ensamblaje general completo del manipulador antropomórfico y su efector.
* **`EnsambleBase.pdf`**: Subensamblaje estructural de la base del robot.
* **`1DOF.pdf`**: Subensamblaje y despiece del primer grado de libertad.
* **`2DOFa.pdf`**: Subensamblaje y despiece del segundo grado de libertad.
* **`3DOF.pdf`**: Subensamblaje y despiece del tercer grado de libertad.
* **`4DOF.pdf`**: Subensamblaje y despiece correspondiente al cuarto grado de libertad / accionamiento del efector.

### 🔹 Estructura Base, Eslabones y Perfiles
* **`BASE.pdf`** y **`BASE1DOF.pdf`**: Piezas principales de la base fija y soporte giratorio inicial.
* **`BARRA1.pdf`** y **`BARRA 2.pdf`**: Eslabones principales que conforman los brazos del manipulador.
* **`CODO.PDF`**: Pieza de unión articular intermedia entre eslabones.
* **`PERFIL.pdf`** y **`perfil2.pdf`**: Perfiles estructurales de soporte y rigidez.
* **`eslabonfinal.pdf`**: Eslabón distal de conexión hacia la muñeca y el efector final.
* **`COLCHAS.pdf`**: Placas laterales / tapas estructurales de recubrimiento y refuerzo.

### 🔹 Soportes de Motor, Ejes y Anclajes
* **`SOPORTEMOTOR.pdf`** y **`SOPORTEMOTOR1DOF.pdf`**: Bases de fijación para los motores/actuadores.
* **`ABRAZADERAMOTOR.pdf`** y **`ABRAZADERAMOTOR1DOF.pdf`**: Abrazaderas de sujeción mecánica para los actuadores.
* **`SOPORTE 1DOF.pdf`** y **`soporte3DOF.pdf`**: Soportes estructurales de las articulaciones 1 y 3.
* **`Anclaje2DOF.pdf`**, **`enclaje3DOF.pdf`** y **`ANCLAJE EFECTOR.pdf`**: Piezas de acople y anclaje entre articulaciones y el efector final.
* **`EJEPRINCIPAL.pdf`**, **`EJEFINAL.pdf`** y **`EJEANCLAJES.PDF`**: Ejes mecánicos de rotación, transmisión y fijación.

### 🔹 Efector Final (Pinza / Garra)
* **`SOPORTE GARRA.pdf`**: Estructura base de montaje para el mecanismo de agarre.
* **`ESLABON PINZA.pdf`**: Eslabón de transmisión cinemática de la pinza.
* **`PINZA DERECHA.PDF`**: Mordaza o dedo derecho del efector final.
* **`PINZA IZQUIERDA.pdf`**: Mordaza o dedo izquierdo del efector final.

---

## 🛠️ Requisitos de Visualización

* **Planos y Análisis (`*.pdf`)**: Pueden abrirse con cualquier visor PDF estándar (Adobe Acrobat Reader, Evince, Okular o navegadores web) conservando la escala y cotas vectoriales.
* **Diagramas Electrónicos (`*.jpg`)**: Visualizables en alta resolución desde cualquier visor de imágenes estándar.