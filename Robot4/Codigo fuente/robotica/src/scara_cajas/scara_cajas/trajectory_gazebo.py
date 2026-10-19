#!/usr/bin/env python3
"""
trajectory_gazebo.py  (entry point: trajectory_gazebo)

Ejecuta en Gazebo las trayectorias de los Excel generados por trayectoria_scara.m.
Publica posiciones [rot1, rot2, pris] (Float64MultiArray) a 100 Hz en
/scara_position_controller/commands, el controlador position_controllers/
JointGroupPositionController de controllers.yaml.

Para cada Excel:
  1. 'aprox': mueve suavemente el robot desde su pose actual (leida de
     /joint_states, o la ultima ordenada) hasta el primer punto del Excel,
     respetando los limites de velocidad.
  2. 'corre': reproduce la trayectoria con interpolacion cubica de Hermite.
  3. 'pausa': espera 'pausa' segundos y pasa al siguiente.

Sin parametros lee todos los .xlsx de CARPETA_DEFECTO (tray1, tray2, ...) y alarga
el tiempo de cada uno lo justo para respetar los limites de velocidad.

Cierra gazebo_mod (los sliders) antes de usarlo: ambos publican en el mismo topico.

Ejemplo:
    ros2 run scara_cajas trajectory_gazebo
    ros2 run scara_cajas trajectory_gazebo --ros-args -p pausa:=3.0 -p repetir:=true
"""
import os
import re
from glob import glob

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['rot1_joint', 'rot2_joint', 'pris_joint']
COLS = ['t', 'th1', 'th2', 'd3', 'th1_dot', 'th2_dot', 'd3_dot']
QD_MAX = np.array([0.2, 0.2, 0.15])          # [rad/s, rad/s, m/s]
ROT2_MIN, ROT2_MAX = -1.1, 4.25
PRIS_MIN, PRIS_MAX = 0.0, 0.131
# El URDF tiene el efector 0.000713 m mas alto que el modelo D-H de MATLAB
D3_OFFSET = 0.0007132557
TOPIC_DEFECTO = '/scara_position_controller/commands'

# Carpeta de donde se leen los Excel cuando no se indica 'archivos' ni 'carpeta'.
# Cambiala aqui si guardas las trayectorias en otro lugar.
CARPETA_DEFECTO = os.path.expanduser(
    '~/Robotica-Meca/Robot4/Codigo fuente/robotica/src/scara_cajas/scara_cajas')


def buscar_excels(carpeta):
    """Todos los .xlsx de la carpeta en orden natural (tray1, tray2, ..., tray10)."""
    archivos = [f for f in glob(os.path.join(carpeta, '*.xlsx'))
                if not os.path.basename(f).startswith('~$')]

    def clave(f):
        return [int(x) if x.isdigit() else x.lower()
                for x in re.split(r'(\d+)', os.path.basename(f))]

    return sorted(archivos, key=clave)


def cargar_segmento(ruta, escala=None, d3_offset=D3_OFFSET):
    df = pd.read_excel(ruta, sheet_name=0)
    faltan = [c for c in COLS if c not in df.columns]
    if faltan:
        raise ValueError(f'{ruta}: faltan columnas {faltan}')

    t = df['t'].to_numpy(float)
    q = df[['th1', 'th2', 'd3']].to_numpy(float)
    qd = df[['th1_dot', 'th2_dot', 'd3_dot']].to_numpy(float)
    if escala is None or escala <= 0:   # automatico: 2 % de margen bajo el limite
        escala = max(1.0, float((np.abs(qd).max(axis=0) / QD_MAX).max()) * 1.02)
    t = t * escala
    qd = qd / escala
    # La ultima fila del Excel trae velocidad 0 (parada). Para interpolar se usa la
    # velocidad del paso anterior: forzar 0 en el ultimo intervalo haria que la
    # curva se sobrepase ~33 % sobre el limite de velocidad.
    qd[-1] = qd[-2]
    q[:, 2] += d3_offset
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError(f'{ruta}: la columna t debe ser estrictamente creciente')

    pico = np.abs(qd).max(axis=0)
    if np.any(pico > QD_MAX * (1 + 1e-6)):
        raise ValueError(
            f'{ruta}: supera limites de velocidad. Pico |qd| = {np.round(pico, 4)} '
            f'(limites {QD_MAX}). Use escala_tiempo mayor o 0 (automatico).')

    aviso = []
    if q[:, 1].min() < ROT2_MIN or q[:, 1].max() > ROT2_MAX:
        aviso.append('th2 fuera del rango de rot2_joint')
    if q[:, 2].min() < PRIS_MIN - 1e-4 or q[:, 2].max() > PRIS_MAX + 1e-4:
        aviso.append('d3 fuera del recorrido de pris_joint (mas de 0.1 mm)')
    q[:, 2] = np.clip(q[:, 2], PRIS_MIN, PRIS_MAX)

    return {'ruta': ruta, 't': t, 'q': q, 'qd': qd, 'escala': escala, 'aviso': aviso}


def muestrear(seg, te):
    """Interpolacion cubica de Hermite con q y qd del Excel. Devuelve q (3,)."""
    t, q, qd = seg['t'], seg['q'], seg['qd']
    if te <= t[0]:
        return q[0].copy()
    if te >= t[-1]:
        return q[-1].copy()
    k = int(np.searchsorted(t, te, side='right') - 1)
    h = t[k + 1] - t[k]
    s = (te - t[k]) / h
    return ((2*s**3 - 3*s**2 + 1)*q[k] + (s**3 - 2*s**2 + s)*h*qd[k]
            + (-2*s**3 + 3*s**2)*q[k + 1] + (s**3 - s**2)*h*qd[k + 1])


def duracion_aprox(q_a, q_b, minimo):
    """Tiempo de un movimiento suave (smoothstep, pico = 1.5 x velocidad media)."""
    dq = np.abs(np.asarray(q_b) - np.asarray(q_a))
    return max(minimo, 1.5 * 1.05 * float((dq / QD_MAX).max()))


class TrajectoryGazebo(Node):
    def __init__(self):
        super().__init__('trajectory_gazebo')

        self.declare_parameter('archivos', [''])        # vacio = leer toda la carpeta
        self.declare_parameter('carpeta', '')           # vacio = CARPETA_DEFECTO
        self.declare_parameter('topic', TOPIC_DEFECTO)
        self.declare_parameter('pausa', 1.0)            # s entre trayectorias
        self.declare_parameter('escala_tiempo', 0.0)    # 0 = automatico; >1 = mas lento
        self.declare_parameter('d3_offset', D3_OFFSET)
        self.declare_parameter('hz', 100.0)             # = update_rate del controller_manager
        self.declare_parameter('lead_min', 1.0)         # s minimos de la aproximacion
        self.declare_parameter('espera_estado', 5.0)    # s esperando /joint_states
        self.declare_parameter('repetir', False)
        self.declare_parameter('tol_empalme', 0.005)

        p = self.get_parameter
        archivos = [os.path.expanduser(a) for a in p('archivos').value if a]
        if not archivos:
            carpeta = os.path.expanduser(p('carpeta').value or CARPETA_DEFECTO)
            archivos = buscar_excels(carpeta)
            if not archivos:
                raise SystemExit(f'No hay archivos .xlsx en {carpeta}. '
                                 'Cambia CARPETA_DEFECTO o usa -p carpeta:=...')
            self.get_logger().info(f'Leyendo {len(archivos)} Excel de {carpeta}')
        topic = p('topic').value
        self.pausa = float(p('pausa').value)
        self.lead_min = float(p('lead_min').value)
        self.espera_estado = float(p('espera_estado').value)
        self.repetir = bool(p('repetir').value)
        esc = float(p('escala_tiempo').value)
        d3_off = float(p('d3_offset').value)
        tol = float(p('tol_empalme').value)

        # Cargar y validar todo antes de mover nada
        self.segs = []
        for ruta in archivos:
            try:
                seg = cargar_segmento(ruta, esc, d3_off)
            except (ValueError, OSError) as e:
                raise SystemExit(f'Error leyendo Excel: {e}')
            for a in seg['aviso']:
                self.get_logger().warn(f'{ruta}: {a}')
            self.get_logger().info(
                f"{os.path.basename(ruta)}: {len(seg['t'])} filas, "
                f"duracion {seg['t'][-1]:.2f} s (escala de tiempo x{seg['escala']:.3f})")
            self.segs.append(seg)

        for a, b in zip(self.segs[:-1], self.segs[1:]):
            salto = np.abs(a['q'][-1] - b['q'][0]).max()
            if salto > tol:
                self.get_logger().warn(
                    f"El final de {os.path.basename(a['ruta'])} no coincide con el inicio de "
                    f"{os.path.basename(b['ruta'])} (salto max {salto:.4f}). "
                    'Entre ambas el robot hara una aproximacion suave.')

        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(JointState, '/joint_states', self._estado_cb, 10)
        self.get_logger().info(f'Enviando posiciones a {topic} a {p("hz").value} Hz')

        self.q_actual = None          # ultima pose medida en /joint_states
        self.q_cmd = None             # ultima pose ordenada
        self.i = -1
        self.fase = 'inicio'
        self.t_ini = self._now()
        self.ultimo_aviso = -1e9
        self.timer = self.create_timer(1.0 / float(p('hz').value), self._tick)

    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _estado_cb(self, msg):
        try:
            self.q_actual = np.array([msg.position[msg.name.index(j)] for j in JOINTS])
        except (ValueError, IndexError):
            pass

    def _publicar(self, q):
        self.q_cmd = np.asarray(q, dtype=float).copy()
        m = Float64MultiArray()
        m.data = [float(x) for x in self.q_cmd]
        self.pub.publish(m)

    def _empezar_aprox(self, i, now):
        if i >= len(self.segs):
            if not self.repetir:
                self.get_logger().info('Todas las trayectorias terminaron.')
                self.fase = 'fin'
                self.timer.cancel()
                return
            i = 0
        self.i = i
        self.q_a = self.q_cmd.copy()
        self.q_b = self.segs[i]['q'][0].copy()
        self.T_aprox = duracion_aprox(self.q_a, self.q_b, self.lead_min)
        self.get_logger().info(
            f"{i + 1}/{len(self.segs)} {os.path.basename(self.segs[i]['ruta'])}: "
            f'aproximacion al punto inicial ({self.T_aprox:.1f} s)')
        self.t_ini = now
        self.fase = 'aprox'

    def _tick(self):
        now = self._now()

        if self.fase == 'inicio':
            if self.pub.get_subscription_count() == 0:
                if now - self.ultimo_aviso > 5.0:
                    self.get_logger().warn(
                        'No hay controlador suscrito a los comandos. '
                        'Revisa: ros2 control list_controllers (scara_position_controller activo)')
                    self.ultimo_aviso = now
                return
            if self.q_actual is None:
                if now - self.t_ini < self.espera_estado:
                    return
                self.get_logger().warn('Sin /joint_states: se asume pose inicial [0, 0, 0].')
                self.q_actual = np.zeros(3)
            self.q_cmd = self.q_actual.copy()
            self._empezar_aprox(0, now)
            return

        if self.fase == 'aprox':
            s = min(1.0, (now - self.t_ini) / self.T_aprox)
            e = 3*s**2 - 2*s**3
            self._publicar(self.q_a + e*(self.q_b - self.q_a))
            if s >= 1.0:
                self.get_logger().info('Ejecutando trayectoria.')
                self.t_ini = now
                self.fase = 'corre'
            return

        if self.fase == 'corre':
            seg = self.segs[self.i]
            te = now - self.t_ini
            self._publicar(muestrear(seg, te))
            if te >= seg['t'][-1]:
                self.get_logger().info(f'Trayectoria {self.i + 1} terminada.')
                self.t_ini = now
                self.fase = 'pausa'
            return

        if self.fase == 'pausa':
            self._publicar(self.q_cmd)
            if now - self.t_ini >= self.pausa:
                self._empezar_aprox(self.i + 1, now)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryGazebo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()