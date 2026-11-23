#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI gotica (estilo Poe / cuervos / gatos negros / Goetia) para mover el brazo miau.

  * Botones de movimiento  -> publican "ORIGEN DESTINO" en /comando_trayectoria
                              (lo que ya escucha nodo_cinematico, p. ej. "PH PA").
  * Cerrar pinza           -> publica "CERRAR" en el topico  rawr  (std_msgs/String)
                              y lector_velocidades lo reenvia a la ESP32.
  * La posicion actual se sigue con /estado_trayectoria (lo publica nodo_cinematico).

  * Ofrenda de datos       -> muestra la velocidad comandada (la que sale de nodo_cinematico
                              y llega a la ESP32), y la posicion/velocidad del encoder
                              (realimentacion).
  * Pestana "Pruebas"      -> sliders de velocidad por junta, publicados en el mismo topico
                              de velocidades (ver aviso en la propia pestana).

Reglas (modo normal): desde PH se puede ir a cualquier punto; desde cualquier otro
punto solo se puede volver a PH.  Todos los botones estan en pantalla; los que no
corresponden a la posicion actual quedan sellados.  "Romper los sellos" habilita
todas las combinaciones.
"""
import math
import random
import time
import tkinter as tk
import tkinter.font as tkfont

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

PUNTOS = ["PH", "PA", "PB", "P1", "P2"]
# Espiritus de la Ars Goetia, solo como ornamento de cada punto
ESPIRITUS = {"PH": "Bael", "PA": "Agares", "PB": "Vassago",
             "P1": "Samigina", "P2": "Marbas"}

# --- paleta ------------------------------------------------------------------
BG = "#09090d"
PANEL = "#12121a"
PANEL2 = "#1a1a24"
BONE = "#d8cfb8"
DIM = "#5a5366"
CRIMSON = "#8b0f1f"
BRIGHT = "#c1121f"
GOLD = "#b8962e"
MOON = "#e8e4d0"

VERSOS = [
    ("Once upon a midnight dreary, while I pondered, weak and weary,", "E. A. Poe · The Raven"),
    ("Quoth the Raven, \u201cNevermore.\u201d", "E. A. Poe · The Raven"),
    ("Deep into that darkness peering, long I stood there wondering, fearing,", "E. A. Poe · The Raven"),
    ("And the Raven, never flitting, still is sitting, still is sitting", "E. A. Poe · The Raven"),
    ("Is all that we see or seem but a dream within a dream?", "E. A. Poe · A Dream Within a Dream"),
    ("Tres juntas giran en la sombra,\nel hierro aprende a rezar.", "Salmo del brazo"),
    ("El gato negro vigila el ángulo\ndonde la última luz se rinde.", "Vigilia felina"),
    ("Bael, primer rey, abre el camino:\ntodo viaje nace y muere en PH.", "Ars Goetia"),
    ("Un cuervo cuenta los pasos del hierro;\nninguno vuelve distinto.", "Augurio"),
]


# =============================================================================
#  Logica de posicion/estado (sin Tk, para poder probarla)
# =============================================================================
class Rastreador:
    """Sigue la posicion del brazo a partir de los textos de /estado_trayectoria."""

    def __init__(self, posicion_inicial="PH"):
        self.pos = posicion_inicial     # punto actual, o "?" si se desconoce
        self.ocupado = False
        self.destino_homing = None
        self.ruta = None                # (origen, destino) mientras ejecuta
        self.texto = "libre"

    def estado(self, s):
        s = s.strip()
        self.texto = s
        low = s.lower()
        if low.startswith("homing "):
            self.ocupado = True
            self.destino_homing = s.split(None, 1)[1].strip()
            self.ruta = None
        elif low.startswith("ejecutando "):
            self.ocupado = True
            nombre = s.split(None, 1)[1].strip()
            if "_a_" in nombre:
                o, d = nombre.split("_a_", 1)
                self.ruta = (o, d)
        elif low.startswith("fin "):
            nombre = s.split(None, 1)[1].strip()
            if "_a_" in nombre:
                self.pos = nombre.split("_a_", 1)[1]
        elif low == "libre":
            if self.destino_homing:
                self.pos = self.destino_homing
            self.destino_homing = None
            self.ruta = None
            self.ocupado = False
        elif low.startswith("error") or low == "parado":
            self.pos = "?"
            self.destino_homing = None
            self.ruta = None
            self.ocupado = False

    @staticmethod
    def permitido(origen, destino, pos, ocupado, libre_total):
        if origen == destino or ocupado:
            return False
        if libre_total:
            return True
        if origen != pos:
            return False
        return origen == "PH" or destino == "PH"


# =============================================================================
#  Telemetria (sin Tk, para poder probarla)
# =============================================================================
class Telemetria:
    """Ultimos valores de velocidad comandada y de la realimentacion del encoder."""

    ALFA = 0.35   # filtro pasa-bajas de la velocidad derivada

    def __init__(self):
        self.cmd = [None, None, None]
        self.cmd_t = 0.0
        self.pos = [None, None, None]
        self.vel = [None, None, None]
        self.vel_derivada = False      # True si la velocidad sale de derivar la posicion
        self.fb_t = 0.0
        self._ant = None

    def on_cmd(self, datos, t):
        if len(datos) < 3:
            return
        self.cmd = [float(datos[0]), float(datos[1]), float(datos[2])]
        self.cmd_t = t

    def on_pos(self, q, t, vel=None):
        """q: 3 posiciones [rad]; vel: 3 velocidades medidas, o None para derivarlas."""
        self.pos = [float(x) for x in q]
        if vel is not None:
            self.vel = [float(x) for x in vel]
            self.vel_derivada = False
        else:
            self.vel_derivada = True
            if self._ant is not None:
                q0, t0 = self._ant
                dt = t - t0
                if 1e-4 < dt < 1.0:
                    nuevo = [(self.pos[i] - q0[i]) / dt for i in range(3)]
                    if self.vel[0] is None:
                        self.vel = nuevo
                    else:
                        self.vel = [self.ALFA * nuevo[i] + (1 - self.ALFA) * self.vel[i]
                                    for i in range(3)]
            self._ant = (list(self.pos), t)
        self.fb_t = t


# =============================================================================
#  Nodo ROS
# =============================================================================
class NodoGui(Node):
    def __init__(self):
        super().__init__("gui_miau")
        self.declare_parameter("topic_comando", "/comando_trayectoria")
        self.declare_parameter("topic_estado", "/estado_trayectoria")
        self.declare_parameter("topic_pinza", "rawr")
        self.declare_parameter("posicion_inicial", "PH")
        # velocidades comandadas (lo que publica nodo_cinematico y lee lector_velocidades)
        self.declare_parameter("topic_velocidades", "/velocity_controller/commands")
        # donde publican los sliders de prueba (por defecto, el mismo topico de velocidades)
        self.declare_parameter("topic_prueba", "/velocity_controller/commands")
        # realimentacion: igual que nodo_cinematico ("gazebo" -> JointState, "robot" -> Float64MultiArray)
        self.declare_parameter("fuente_realimentacion", "gazebo")
        self.declare_parameter("topic_realimentacion", "")
        self.declare_parameter("joint_names", ["J1", "J2", "J3"])
        self.declare_parameter("vel_max_prueba", 1.0)

        self.telem = Telemetria()
        self.juntas = list(self.get_parameter("joint_names").value)
        self.fuente = self.get_parameter("fuente_realimentacion").value
        topic_fb = self.get_parameter("topic_realimentacion").value or (
            "/joint_states" if self.fuente == "gazebo" else "/robot/posiciones")
        self.topic_fb = topic_fb

        self.pub_cmd = self.create_publisher(
            String, self.get_parameter("topic_comando").value, 10)
        self.pub_pinza = self.create_publisher(
            String, self.get_parameter("topic_pinza").value, 10)
        self.topic_prueba = self.get_parameter("topic_prueba").value
        self.pub_prueba = self.create_publisher(Float64MultiArray, self.topic_prueba, 10)
        self.callback_estado = None
        self.create_subscription(
            String, self.get_parameter("topic_estado").value, self._on_estado, 10)
        self.create_subscription(
            Float64MultiArray, self.get_parameter("topic_velocidades").value, self._on_cmd, 10)
        if self.fuente == "gazebo":
            self.create_subscription(JointState, topic_fb, self._on_joint_states, 10)
        else:
            self.create_subscription(Float64MultiArray, topic_fb, self._on_posiciones, 10)

    def _on_estado(self, msg):
        if self.callback_estado:
            self.callback_estado(msg.data)

    def _on_cmd(self, msg):
        self.telem.on_cmd(msg.data, time.time())

    def _on_joint_states(self, msg):
        q, v = [], []
        for nombre in self.juntas[:3]:
            if nombre not in msg.name:
                return
            i = msg.name.index(nombre)
            if i >= len(msg.position):
                return
            q.append(msg.position[i])
            v.append(msg.velocity[i] if i < len(msg.velocity) else None)
        medidas = None if any(x is None for x in v) else v
        self.telem.on_pos(q, time.time(), medidas)

    def _on_posiciones(self, msg):
        if len(msg.data) >= 3:
            self.telem.on_pos(msg.data[:3], time.time(), None)

    def mover(self, origen, destino):
        m = String()
        ruta = [origen]
        if origen in ("PA", "PB", "P1", "P2") and destino != f"{origen}_UP":
            ruta.append(f"{origen}_UP")
        involucra_b = origen in ("PB", "PB_UP") or destino in ("PB", "PB_UP")
        if involucra_b and origen != "PM" and destino != "PM":
            if ruta[-1] != "PM":
                ruta.append("PM")
        if destino in ("PA", "PB", "P1", "P2") and ruta[-1] != f"{destino}_UP":
            ruta.append(f"{destino}_UP")
        if ruta[-1] != destino:
            ruta.append(destino)
        m.data = " ".join(ruta)
        self.pub_cmd.publish(m)

    def comando(self, texto):
        m = String()
        m.data = texto
        self.pub_cmd.publish(m)

    def abrir_pinza(self):
        m = String()
        m.data = "ABRIR"
        self.pub_pinza.publish(m)

    def cerrar_pinza(self):
        m = String()
        m.data = "CERRAR"
        self.pub_pinza.publish(m)

    def velocidades_prueba(self, v):
        m = Float64MultiArray()
        m.data = [float(v[0]), float(v[1]), float(v[2])]
        self.pub_prueba.publish(m)


# =============================================================================
#  Interfaz
# =============================================================================
class Gui:
    def __init__(self, nodo, ejecutor):
        self.nodo = nodo
        self.ejecutor = ejecutor
        self.rast = Rastreador(nodo.get_parameter("posicion_inicial").value)
        nodo.callback_estado = self.on_estado
        self.esperando_desde = None
        self.armado = False            # modo prueba (sliders) activo
        self.sliders = []
        self.verso_i = 0
        self.t0 = time.time()

        self.root = tk.Tk()
        self.root.title("Cámara del Cuervo — control del brazo miau")
        self.root.configure(bg=BG)
        self.root.geometry("1120x770")
        self.root.minsize(980, 700)

        fams = set(tkfont.families())

        def elegir(opciones, defecto):
            for o in opciones:
                if o in fams:
                    return o
            return defecto

        self.f_gotica = elegir(["UnifrakturCook", "UnifrakturMaguntia", "Pirata One",
                                "Old English Text MT", "Blackadder ITC"], None)
        self.f_serif = elegir(["EB Garamond", "Garamond", "Cormorant Garamond",
                               "Palatino Linotype", "DejaVu Serif", "Georgia"], "serif")
        titulo_fam = self.f_gotica or self.f_serif
        self.ft_titulo = (titulo_fam, 30, "bold" if not self.f_gotica else "normal")
        self.ft_sub = (self.f_serif, 11, "italic")
        self.ft_btn = (self.f_serif, 11, "bold")
        self.ft_verso = (self.f_serif, 13, "italic")
        self.ft_peq = (self.f_serif, 9)
        self.f_mono = elegir(["DejaVu Sans Mono", "Consolas", "Liberation Mono", "Courier New"], "Courier")
        self.ft_mono = (self.f_mono, 11)

        self._construir_cabecera()
        cuerpo = tk.Frame(self.root, bg=BG)
        cuerpo.pack(fill="both", expand=True, padx=14, pady=(6, 4))
        cuerpo.columnconfigure(1, weight=1)
        cuerpo.rowconfigure(0, weight=1)
        self._construir_diagrama(cuerpo)
        self._construir_panel(cuerpo)
        self._construir_pie()

        self.refrescar()
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar)
        self.root.after(20, self.spin)
        self.root.after(60, self.animar)
        self.root.after(9000, self.rotar_verso)
        self.root.after(50, self.publicar_prueba)

    # ---------------------------------------------------------------- cabecera
    def _construir_cabecera(self):
        self.cab = tk.Canvas(self.root, height=140, bg=BG, highlightthickness=0)
        self.cab.pack(fill="x")
        self.cab.bind("<Configure>", lambda e: self._dibujar_cabecera())
        self._dibujar_cabecera()

    def _dibujar_cabecera(self):
        c = self.cab
        c.delete("all")
        w = max(c.winfo_width(), 900)
        rnd = random.Random(13)
        for _ in range(70):
            x, y = rnd.randint(0, w), rnd.randint(0, 135)
            r = rnd.choice([0.6, 0.8, 1.1])
            c.create_oval(x - r, y - r, x + r, y + r, fill=rnd.choice(["#6d6a60", "#938f80", "#4c4a44"]), outline="")
        # luna
        mx, my = w - 120, 62
        c.create_oval(mx - 44, my - 44, mx + 44, my + 44, fill="#2a2a2f", outline="")
        c.create_oval(mx - 40, my - 40, mx + 40, my + 40, fill=MOON, outline="")
        c.create_oval(mx - 14, my - 20, mx + 6, my, fill="#cfcab4", outline="")
        c.create_oval(mx + 8, my + 8, mx + 26, my + 26, fill="#d6d1ba", outline="")
        # murcielagos
        for bx, by, s in [(mx - 120, 36, 1.0), (mx - 175, 72, 0.7), (mx - 70, 100, 0.6)]:
            self._murcielago(c, bx, by, s)
        # rama + cuervo
        c.create_line(10, 128, 150, 118, 260, 122, smooth=True, width=5, fill="#2a1f1a", capstyle="round")
        c.create_line(150, 118, 190, 100, smooth=True, width=3, fill="#2a1f1a")
        self._cuervo(c, 120, 118, 1.0)
        # titulo
        cx = w // 2 - 30
        c.create_text(cx + 2, 52, text="Cámara del Cuervo", font=self.ft_titulo, fill="#2b0508")
        c.create_text(cx, 50, text="Cámara del Cuervo", font=self.ft_titulo, fill=BONE)
        c.create_text(cx, 92, text="rito de traslación del brazo · tres juntas, un solo augurio",
                      font=self.ft_sub, fill=GOLD)
        c.create_line(cx - 230, 112, cx + 230, 112, fill=CRIMSON, width=1)
        c.create_text(cx, 112, text="✠", font=(self.f_serif, 13), fill=BRIGHT)
        # gato
        self._gato(c, w - 250, 138, 0.9)

    def _murcielago(self, c, x, y, s):
        pts = [(-22, -4), (-14, -12), (-8, -4), (-4, -10), (0, -2), (4, -10), (8, -4), (14, -12),
               (22, -4), (14, 2), (8, 0), (0, 8), (-8, 0), (-14, 2)]
        flat = []
        for px, py in pts:
            flat += [x + px * s, y + py * s]
        c.create_polygon(flat, fill="#050507", outline="")

    def _cuervo(self, c, x, y, s):
        """Cuervo posado, mirando a la izquierda. (x, y) = patas sobre la rama."""
        cuerpo = [(-14, -60), (-6, -66), (4, -60), (10, -48), (26, -38), (40, -24), (66, -4), (58, 0),
                  (36, -6), (20, -2), (6, -4), (-8, -18), (-16, -36), (-20, -48)]
        flat = []
        for px, py in cuerpo:
            flat += [x + px * s, y + py * s]
        c.create_polygon(flat, fill="#060609", outline="#1d1d26", smooth=True)
        # pico
        c.create_polygon([x - 14 * s, y - 58 * s, x - 38 * s, y - 50 * s, x - 15 * s, y - 49 * s],
                         fill="#15151a", outline="#2a2a33")
        # ojo
        c.create_oval(x - 11 * s, y - 59 * s, x - 7 * s, y - 55 * s, fill="#d9c35a", outline="")
        c.create_oval(x - 10 * s, y - 58 * s, x - 8.5 * s, y - 56.5 * s, fill="#000", outline="")
        # patas
        c.create_line(x + 2 * s, y - 4 * s, x + 2 * s, y, fill="#101014", width=2)
        c.create_line(x + 14 * s, y - 4 * s, x + 14 * s, y, fill="#101014", width=2)

    def _gato(self, c, x, y, s):
        """Gato negro sentado de frente. (x, y) = base."""
        # cola
        c.create_line(x + 30 * s, y - 6 * s, x + 58 * s, y - 12 * s, x + 64 * s, y - 44 * s, x + 46 * s, y - 62 * s,
                      smooth=True, width=9 * s, fill="#050507", capstyle="round")
        # cuerpo
        c.create_oval(x - 34 * s, y - 78 * s, x + 34 * s, y, fill="#050507", outline="#1a1a22")
        # cabeza
        c.create_oval(x - 22 * s, y - 112 * s, x + 22 * s, y - 70 * s, fill="#050507", outline="#1a1a22")
        # orejas
        c.create_polygon(x - 21 * s, y - 96 * s, x - 22 * s, y - 128 * s, x - 5 * s, y - 108 * s, fill="#050507", outline="#1a1a22")
        c.create_polygon(x + 21 * s, y - 96 * s, x + 22 * s, y - 128 * s, x + 5 * s, y - 108 * s, fill="#050507", outline="#1a1a22")
        # ojos
        for ex in (-9, 9):
            c.create_oval(x + (ex - 6) * s, y - 98 * s, x + (ex + 6) * s, y - 88 * s, fill="#e6c200", outline="")
            c.create_oval(x + (ex - 1.2) * s, y - 98 * s, x + (ex + 1.2) * s, y - 88 * s, fill="#000", outline="")
        c.create_polygon(x - 2 * s, y - 85 * s, x + 2 * s, y - 85 * s, x, y - 82 * s, fill="#7a2a35", outline="")
        # bigotes
        for dy in (-82, -79):
            c.create_line(x - 6 * s, y + dy * s, x - 30 * s, y + (dy - 3) * s, fill="#3a3a44")
            c.create_line(x + 6 * s, y + dy * s, x + 30 * s, y + (dy - 3) * s, fill="#3a3a44")

    # ---------------------------------------------------------------- diagrama
    W = 400
    CX = CY = 200
    POS = {"PH": (200, 200), "PA": (200, 62), "P1": (338, 200), "PB": (200, 338), "P2": (62, 200)}

    def _construir_diagrama(self, padre):
        izq = tk.Frame(padre, bg=BG)
        izq.grid(row=0, column=0, sticky="n", padx=(0, 12))
        marco = tk.Frame(izq, bg=PANEL, highlightbackground=CRIMSON, highlightthickness=1)
        marco.pack()
        self.dia = tk.Canvas(marco, width=self.W, height=self.W, bg=PANEL, highlightthickness=0)
        self.dia.pack(padx=6, pady=6)
        c = self.dia
        # anillos
        c.create_oval(8, 8, 392, 392, outline=GOLD, width=2)
        c.create_oval(18, 18, 382, 382, outline=CRIMSON, width=1)
        c.create_oval(58, 58, 342, 342, outline="#2a1a26", width=1)
        # simbolos del anillo
        for i, g in enumerate("♈♉♊♋♌♍♎♏♐♑♒♓"):
            a = math.radians(i * 30 - 75)
            c.create_text(200 + 174 * math.cos(a), 200 + 174 * math.sin(a), text=g, fill=GOLD, font=(self.f_serif, 11))
        # hexagrama tenue
        for off in (-90, -30):
            pts = []
            for k in range(3):
                a = math.radians(off + k * 120)
                pts += [200 + 150 * math.cos(a), 200 + 150 * math.sin(a)]
            c.create_polygon(pts, outline="#3a1520", fill="", width=1)
        # caminos
        for p in ("PA", "PB", "P1", "P2"):
            x, y = self.POS[p]
            c.create_line(200, 200, x, y, fill="#3b2a33", width=2, dash=(4, 4))
        self.dia_dyn = "dyn"
        self._construir_telemetria(izq)

    def _construir_telemetria(self, padre):
        marco = tk.Frame(padre, bg=PANEL, highlightbackground=CRIMSON, highlightthickness=1)
        marco.pack(fill="x", pady=(8, 0))
        tk.Label(marco, text="✠  Ofrenda de datos  ✠", bg=PANEL, fg=GOLD,
                 font=(self.f_serif, 11, "italic")).grid(row=0, column=0, columnspan=4, pady=(5, 2))
        for j, h in enumerate(["J1", "J2", "J3"]):
            tk.Label(marco, text=h, bg=PANEL, fg=BONE, font=(self.f_serif, 11, "bold"),
                     width=9).grid(row=1, column=j + 1)
        filas = [("Comandada [rad/s]", "cmd"), ("Encoder pos [rad]", "pos"), ("Encoder vel [rad/s]", "vel")]
        self.tel_val = {}
        self.tel_nom = {}
        for i, (txt, clave) in enumerate(filas):
            lab = tk.Label(marco, text=txt, bg=PANEL, fg=DIM, font=(self.f_serif, 10, "italic"),
                           anchor="w", width=19)
            lab.grid(row=2 + i, column=0, padx=(8, 2), pady=1, sticky="w")
            self.tel_nom[clave] = lab
            for j in range(3):
                v = tk.Label(marco, text="—", bg=PANEL, fg=DIM, font=self.ft_mono, width=9, anchor="e")
                v.grid(row=2 + i, column=j + 1, pady=1)
                self.tel_val[(clave, j)] = v
        self.lbl_tel_estado = tk.Label(marco, text="", bg=PANEL, fg=DIM, font=self.ft_peq, anchor="w")
        self.lbl_tel_estado.grid(row=5, column=0, columnspan=4, sticky="w", padx=8, pady=(2, 5))

    def _actualizar_telemetria(self):
        t = self.nodo.telem
        ahora = time.time()
        frescos = {"cmd": ahora - t.cmd_t < 1.0, "pos": ahora - t.fb_t < 1.0, "vel": ahora - t.fb_t < 1.0}
        datos = {"cmd": t.cmd, "pos": t.pos, "vel": t.vel}
        for clave in ("cmd", "pos", "vel"):
            self.tel_nom[clave].configure(fg=BONE if frescos[clave] else DIM)
            for j in range(3):
                v = datos[clave][j]
                lbl = self.tel_val[(clave, j)]
                if v is None:
                    lbl.configure(text="—", fg=DIM)
                else:
                    lbl.configure(text=f"{v:+.3f}", fg=BONE if frescos[clave] else DIM)
        deriv = " (derivada de la posición)" if t.vel_derivada else ""
        self.tel_nom["vel"].configure(text="Encoder vel [rad/s]" + ("*" if t.vel_derivada else ""))
        self.lbl_tel_estado.configure(
            text=f"realimentación: {self.nodo.fuente} · {self.nodo.topic_fb}{deriv}")

    def _dibujar_dinamico(self):
        c = self.dia
        c.delete(self.dia_dyn)
        t = time.time() - self.t0
        pulso = 0.5 + 0.5 * math.sin(t * 3.2)
        # camino activo con ascua
        if self.rast.ruta and self.rast.ruta[0] in self.POS and self.rast.ruta[1] in self.POS:
            (x0, y0), (x1, y1) = self.POS[self.rast.ruta[0]], self.POS[self.rast.ruta[1]]
            c.create_line(x0, y0, x1, y1, fill=BRIGHT, width=3, tags=self.dia_dyn)
            f = (t * 0.6) % 1.0
            ex, ey = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            c.create_oval(ex - 8, ey - 8, ex + 8, ey + 8, fill="#ff5a36", outline="#ffb199", tags=self.dia_dyn)
        for p, (x, y) in self.POS.items():
            actual = (p == self.rast.pos)
            if actual:
                r = 32 + 5 * pulso
                c.create_oval(x - r, y - r, x + r, y + r, outline=BRIGHT, width=2, tags=self.dia_dyn)
            c.create_oval(x - 26, y - 26, x + 26, y + 26, fill=CRIMSON if actual else "#15101a",
                          outline=GOLD if actual else DIM, width=2, tags=self.dia_dyn)
            c.create_text(x, y - 3, text=p, fill=BONE if actual else "#8c8498",
                          font=(self.f_serif, 14, "bold"), tags=self.dia_dyn)
            c.create_text(x, y + 40 if p != "PA" else y - 40, text=ESPIRITUS[p],
                          fill=GOLD if actual else DIM, font=(self.f_serif, 9, "italic"), tags=self.dia_dyn)

    # ------------------------------------------------------------------- panel
    def _construir_panel(self, padre):
        der = tk.Frame(padre, bg=BG)
        der.grid(row=0, column=1, sticky="nsew")
        der.columnconfigure(0, weight=1)

        # verso
        vf = tk.Frame(der, bg=PANEL, highlightbackground="#2a1a26", highlightthickness=1)
        vf.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.lbl_verso = tk.Label(vf, text="", bg=PANEL, fg=BONE, font=self.ft_verso,
                                  wraplength=560, justify="center", height=3)
        self.lbl_verso.pack(fill="x", padx=12, pady=(8, 0))
        self.lbl_autor = tk.Label(vf, text="", bg=PANEL, fg=GOLD, font=self.ft_peq)
        self.lbl_autor.pack(pady=(0, 6))
        self._poner_verso()

        # pestanas
        tf = tk.Frame(der, bg=BG)
        tf.grid(row=1, column=0, sticky="ew")
        self.tab_btn = {}
        for clave, txt in (("mov", "☩  Traslación"), ("pru", "☩  Pruebas de juntas")):
            b = tk.Button(tf, text=txt, font=self.ft_btn, relief="flat", bd=0, padx=14, pady=6,
                          cursor="hand2", command=lambda c=clave: self.mostrar_tab(c))
            b.pack(side="left", padx=(0, 4))
            self.tab_btn[clave] = b

        self.cont = tk.Frame(der, bg=BG)
        self.cont.grid(row=2, column=0, sticky="nsew", pady=(4, 0))
        self.tab_mov = tk.Frame(self.cont, bg=BG)
        self.tab_pru = tk.Frame(self.cont, bg=BG)
        self._construir_tab_mov(self.tab_mov)
        self._construir_tab_pru(self.tab_pru)

        # barra permanente
        bf = tk.Frame(der, bg=BG)
        bf.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        self.btn_abrir_pinza = tk.Button(bf, text="☩  Abrir pinza", font=(self.f_serif, 13, "bold"),
                                         bg="#1f2a1e", fg=BONE, activebackground="#2e7d32", activeforeground="#fff",
                                         relief="flat", bd=0, padx=12, pady=10, cursor="hand2",
                                         command=self.abrir_pinza)
        self.btn_abrir_pinza.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.btn_pinza = tk.Button(bf, text="☩  Cerrar pinza", font=(self.f_serif, 13, "bold"),
                                   bg="#3a0a12", fg=BONE, activebackground=BRIGHT, activeforeground="#fff",
                                   relief="flat", bd=0, padx=12, pady=10, cursor="hand2",
                                   command=self.cerrar_pinza)
        self.btn_pinza.pack(side="left", expand=True, fill="x", padx=(0, 6))
        self.btn_home = tk.Button(bf, text="↺  Volver a PH", font=self.ft_btn, bg=PANEL2, fg=BONE,
                                  activebackground=CRIMSON, activeforeground="#fff", relief="flat",
                                  bd=0, padx=10, pady=10, cursor="hand2",
                                  command=lambda: self.nodo.comando("ir PH"))
        self.btn_home.pack(side="left", padx=3)
        self.btn_stop = tk.Button(bf, text="✖  PARAR", font=self.ft_btn, bg="#2a0a0a", fg="#ff8a8a",
                                  activebackground=BRIGHT, activeforeground="#fff", relief="flat",
                                  bd=0, padx=10, pady=10, cursor="hand2",
                                  command=self.parar)
        self.btn_stop.pack(side="left", padx=(3, 0))
        self.mostrar_tab("mov")

    def _construir_tab_mov(self, padre):
        gf = tk.Frame(padre, bg=BG)
        gf.pack(fill="x")
        self.botones = {}
        for i, o in enumerate(PUNTOS):
            fila = tk.Frame(gf, bg=PANEL, highlightbackground="#2a1a26", highlightthickness=1)
            fila.pack(fill="x", pady=2)
            lab = tk.Label(fila, text=f"{o}\n{ESPIRITUS[o]}", width=9, bg=PANEL, fg=GOLD,
                           font=(self.f_serif, 10, "bold"))
            lab.pack(side="left", padx=(6, 2), pady=3)
            self.botones[(o, "lab")] = lab
            for d in PUNTOS:
                if d == o:
                    continue
                b = tk.Button(fila, text=f"{o}  →  {d}", font=self.ft_btn, width=10, relief="flat",
                              bd=0, padx=4, pady=6, cursor="hand2",
                              command=lambda o=o, d=d: self.mover(o, d))
                b.pack(side="left", padx=3, pady=4, expand=True, fill="x")
                b.bind("<Enter>", lambda e, b=b: self._hover(b, True))
                b.bind("<Leave>", lambda e, b=b: self._hover(b, False))
                self.botones[(o, d)] = b
        self.var_libre = tk.BooleanVar(value=False)
        tk.Checkbutton(padre, text="Romper los sellos (habilitar todas las rutas)", variable=self.var_libre,
                       command=self.refrescar, bg=BG, fg=BONE, selectcolor=PANEL2,
                       activebackground=BG, activeforeground=BRIGHT, font=self.ft_peq,
                       highlightthickness=0).pack(anchor="w", pady=(8, 0))

    def _construir_tab_pru(self, padre):
        vmax = float(self.nodo.get_parameter("vel_max_prueba").value)
        self.vmax = vmax
        marco = tk.Frame(padre, bg=PANEL, highlightbackground="#2a1a26", highlightthickness=1)
        marco.pack(fill="x")
        tk.Label(marco, text="Invocación directa de velocidades por junta", bg=PANEL, fg=GOLD,
                 font=(self.f_serif, 12, "italic")).pack(pady=(8, 2))
        tk.Label(marco, text=f"Publica [v1, v2, v3] en  {self.nodo.topic_prueba}  a 20 Hz mientras esté armado.\n"
                             "Detén nodo_cinematico antes: si no, ambos escriben en el mismo tópico y se pisan.",
                 bg=PANEL, fg=DIM, font=self.ft_peq, justify="center").pack(pady=(0, 6))

        arm = tk.Frame(marco, bg=PANEL)
        arm.pack(fill="x", padx=10)
        self.btn_armar = tk.Button(arm, text="☠  ARMAR modo prueba", font=self.ft_btn, bg="#3a0a12",
                                   fg=BONE, activebackground=BRIGHT, activeforeground="#fff",
                                   relief="flat", bd=0, padx=12, pady=7, cursor="hand2",
                                   command=self.alternar_armado)
        self.btn_armar.pack(side="left")
        self.var_resorte = tk.BooleanVar(value=True)
        tk.Checkbutton(arm, text="Volver a 0 al soltar", variable=self.var_resorte, bg=PANEL, fg=BONE,
                       selectcolor=PANEL2, activebackground=PANEL, activeforeground=BRIGHT,
                       font=self.ft_peq, highlightthickness=0).pack(side="left", padx=12)
        tk.Button(arm, text="Cero todos", font=self.ft_peq, bg=PANEL2, fg=BONE, activebackground=CRIMSON,
                  activeforeground="#fff", relief="flat", bd=0, padx=8, pady=5, cursor="hand2",
                  command=self.cero_todos).pack(side="right")

        extremos = [
            ('Horario (-)', 'Antihorario (+)'),
            ('Horario (-)\nBaja', 'Antihorario (+)\nSube'),
            ('Horario (-)\nSube', 'Antihorario (+)\nBaja'),
        ]
        self.sliders = []
        self.lbl_sl = []
        for j in range(3):
            fila = tk.Frame(marco, bg=PANEL)
            fila.pack(fill="x", padx=10, pady=5)
            tk.Label(fila, text=f"J{j + 1}", bg=PANEL, fg=BRIGHT, font=(self.f_serif, 14, "bold"),
                     width=3).pack(side="left")
            tk.Label(fila, text=extremos[j][0], bg=PANEL, fg=BONE, font=self.ft_peq,
                     width=14, justify="right").pack(side="left", padx=(2, 4))
            sc = tk.Scale(fila, from_=-vmax, to=vmax, resolution=0.01, orient="horizontal",
                          showvalue=False, length=210, width=16, sliderlength=28, bd=0,
                          highlightthickness=0, relief="flat", bg="#6a1220", activebackground=BRIGHT,
                          troughcolor="#0b0b10", state="disabled")
            sc.set(0.0)
            sc.pack(side="left", expand=True, fill="x", padx=4)
            sc.bind("<ButtonRelease-1>", lambda e, j=j: self._soltar(j))
            tk.Label(fila, text=extremos[j][1], bg=PANEL, fg=BONE, font=self.ft_peq,
                     width=15, justify="left").pack(side="left", padx=(4, 2))
            lab = tk.Label(fila, text="+0.00", bg=PANEL, fg=DIM, font=self.ft_mono, width=6)
            lab.pack(side="left")
            sc.configure(command=lambda v, j=j: self._valor_slider(j))
            tk.Button(fila, text="0", font=self.ft_peq, bg=PANEL2, fg=BONE, activebackground=CRIMSON,
                      activeforeground="#fff", relief="flat", bd=0, width=3, cursor="hand2",
                      command=lambda j=j: self._cero(j)).pack(side="left", padx=(4, 0))
            self.sliders.append(sc)
            self.lbl_sl.append(lab)
        tk.Label(marco, text=f"rad/s · límite ±{vmax:.2f}", bg=PANEL, fg=DIM,
                 font=self.ft_peq).pack(pady=(0, 8))

    def _construir_pie(self):
        pie = tk.Frame(self.root, bg=PANEL, highlightbackground="#2a1a26", highlightthickness=1)
        pie.pack(fill="x", side="bottom")
        self.lbl_estado = tk.Label(pie, text="", bg=PANEL, fg=BONE, font=(self.f_serif, 11), anchor="w")
        self.lbl_estado.pack(side="left", padx=10, pady=5)
        self.lbl_msg = tk.Label(pie, text="", bg=PANEL, fg=GOLD, font=(self.f_serif, 10, "italic"), anchor="e")
        self.lbl_msg.pack(side="right", padx=10)

    # ------------------------------------------------------------ comportamiento
    def _hover(self, b, dentro):
        if str(b["state"]) == "normal":
            b.configure(bg=BRIGHT if dentro else CRIMSON)

    def _poner_verso(self):
        txt, autor = VERSOS[self.verso_i % len(VERSOS)]
        self.lbl_verso.configure(text=txt)
        self.lbl_autor.configure(text=f"— {autor}")

    def rotar_verso(self):
        self.verso_i += 1
        self._poner_verso()
        self.root.after(9000, self.rotar_verso)

    def mover(self, origen, destino):
        if self.armado:
            self.lbl_msg.configure(text="Desarma el modo prueba antes de trasladar")
            return
        if not Rastreador.permitido(origen, destino, self.rast.pos, self.rast.ocupado, self.var_libre.get()):
            return
        self.nodo.mover(origen, destino)
        self.rast.ocupado = True          # bloqueo optimista hasta que llegue /estado_trayectoria
        self.esperando_desde = time.time()
        self.lbl_msg.configure(text=f"Invocando {origen} → {destino}…")
        self.refrescar()

    def abrir_pinza(self):
        self.nodo.abrir_pinza()
        self.lbl_msg.configure(text="La pinza se abre · «ABRIR» enviado")
        self.btn_abrir_pinza.configure(bg="#2e7d32")
        self.btn_pinza.configure(bg="#3a0a12")

    def cerrar_pinza(self):
        self.nodo.cerrar_pinza()
        self.lbl_msg.configure(text="La pinza se cierra · «CERRAR» enviado")
        self.btn_pinza.configure(bg=BRIGHT)
        self.btn_abrir_pinza.configure(bg="#1f2a1e")

    def mostrar_tab(self, clave):
        self.tab_mov.pack_forget()
        self.tab_pru.pack_forget()
        (self.tab_mov if clave == "mov" else self.tab_pru).pack(fill="both", expand=True)
        for k, b in self.tab_btn.items():
            b.configure(bg=CRIMSON if k == clave else PANEL2, fg=BONE if k == clave else DIM,
                        activebackground=BRIGHT, activeforeground="#fff")

    def parar(self):
        self.cero_todos()
        self.nodo.comando("PARAR")

    # ---- modo prueba (sliders)
    def alternar_armado(self):
        self.armado = not self.armado
        estado = "normal" if self.armado else "disabled"
        for sc in self.sliders:
            sc.configure(state=estado)
        if self.armado:
            self.btn_armar.configure(text="☠  DESARMAR modo prueba", bg=BRIGHT)
            self.lbl_msg.configure(text="Modo prueba armado: los sliders mandan velocidad")
        else:
            self.cero_todos()
            for _ in range(3):
                self.nodo.velocidades_prueba([0.0, 0.0, 0.0])
            self.btn_armar.configure(text="☠  ARMAR modo prueba", bg="#3a0a12")
            self.lbl_msg.configure(text="Modo prueba desarmado · velocidades en cero")
        self.refrescar()

    def _valor_slider(self, j):
        v = self.sliders[j].get()
        self.lbl_sl[j].configure(text=f"{v:+.2f}", fg=BONE if abs(v) > 1e-9 else DIM)

    def _cero(self, j):
        sc = self.sliders[j]
        previo = str(sc.cget("state"))
        sc.configure(state="normal")     # un Scale deshabilitado ignora set()
        sc.set(0.0)
        sc.configure(state=previo)
        self._valor_slider(j)

    def cero_todos(self):
        for j in range(len(self.sliders)):
            self._cero(j)

    def _soltar(self, j):
        if self.var_resorte.get():
            self._cero(j)

    def publicar_prueba(self):
        if self.armado:
            self.nodo.velocidades_prueba([sc.get() for sc in self.sliders])
        self.root.after(50, self.publicar_prueba)

    def on_estado(self, texto):
        self.esperando_desde = None
        self.rast.estado(texto)
        if texto.lower().startswith("error"):
            self.lbl_msg.configure(text=texto[:70])
        self.refrescar()

    def refrescar(self):
        libre = self.var_libre.get()
        for o in PUNTOS:
            for d in PUNTOS:
                if o == d:
                    continue
                b = self.botones[(o, d)]
                ok = Rastreador.permitido(o, d, self.rast.pos, self.rast.ocupado or self.armado, libre)
                if ok:
                    b.configure(state="normal", bg=CRIMSON, fg=BONE, activebackground=BRIGHT,
                                activeforeground="#fff")
                else:
                    b.configure(state="disabled", bg="#14121a", fg="#3d3846", disabledforeground="#3d3846")
            self.botones[(o, "lab")].configure(fg=BRIGHT if o == self.rast.pos else GOLD)
        ocupado = "ocupado · " if self.rast.ocupado else ""
        self.lbl_estado.configure(
            text=f"Posición: {self.rast.pos}   ·   {ocupado}estado del nodo: {self.rast.texto}")
        self.btn_home.configure(state="disabled" if (self.rast.ocupado or self.armado) else "normal")

    def animar(self):
        self._dibujar_dinamico()
        self._actualizar_telemetria()
        # si mandamos una orden y el nodo nunca contesto, desbloquea
        if self.esperando_desde and time.time() - self.esperando_desde > 3.0:
            self.esperando_desde = None
            self.rast.ocupado = False
            self.lbl_msg.configure(text="Sin respuesta de nodo_cinematico (¿está lanzado?)")
            self.refrescar()
        self.root.after(60, self.animar)

    def spin(self):
        if rclpy.ok():
            for _ in range(5):
                self.ejecutor.spin_once(timeout_sec=0)
            self.root.after(20, self.spin)

    def cerrar(self):
        if self.armado:
            for _ in range(3):
                self.nodo.velocidades_prueba([0.0, 0.0, 0.0])
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoGui()
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(nodo)
    try:
        Gui(nodo, ejecutor).run()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
