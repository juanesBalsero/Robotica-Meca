#!/usr/bin/env python3
"""
trajectory_rviz.py  (entry point: trajectory_rviz)

Reproduce en RViz las trayectorias de los Excel generados por trayectoria_scara.m.
Publica sensor_msgs/JointState en /joint_states y dibuja la ruta cartesiana
(columnas x, y, z del Excel) como MarkerArray en /trayectoria_marker.

IMPORTANTE: solo un nodo debe publicar /joint_states. No lo uses junto a
gui_presets ni a joint_state_publisher(_gui).

En RViz: Fixed Frame = world, agregar RobotModel y MarkerArray (/trayectoria_marker).

Sin parametros lee todos los .xlsx de CARPETA_DEFECTO (en orden: tray1, tray2, ...)
y alarga el tiempo de cada uno lo justo para respetar los limites de velocidad.

Ejemplo con parametros:
    ros2 run scara_cajas trajectory_rviz --ros-args \
        -p archivos:="['/ruta/tray1.xlsx','/ruta/tray2.xlsx','/ruta/tray3.xlsx']" \
        -p pausa:=2.0
"""
import os
import re
from glob import glob

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

JOINTS = ['rot1_joint', 'rot2_joint', 'pris_joint']
COLS = ['t', 'th1', 'th2', 'd3', 'th1_dot', 'th2_dot', 'd3_dot']
QD_MAX = np.array([0.2, 0.2, 0.15])          # [rad/s, rad/s, m/s]
ROT2_MIN, ROT2_MAX = -1.1, 4.25
PRIS_MIN, PRIS_MAX = 0.0, 0.131
# El URDF tiene el efector 0.000713 m mas alto que el modelo D-H de MATLAB
D3_OFFSET = 0.0007132557

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
COLORES = [(1.0, 0.2, 0.2), (0.2, 0.9, 0.2), (0.2, 0.5, 1.0), (1.0, 0.8, 0.1)]


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
    q[:, 2] += d3_offset
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError(f'{ruta}: la columna t debe ser estrictamente creciente')

    pico = np.abs(qd).max(axis=0)
    if np.any(pico > QD_MAX * (1 + 1e-6)):
        raise ValueError(
            f'{ruta}: supera limites de velocidad. Pico |qd| = {np.round(pico, 4)} '
            f'(limites {QD_MAX}). Use mayor t_total o escala_tiempo > 1.')

    aviso = []
    if q[:, 1].min() < ROT2_MIN or q[:, 1].max() > ROT2_MAX:
        aviso.append('th2 fuera del rango de rot2_joint')
    if q[:, 2].min() < PRIS_MIN - 1e-4 or q[:, 2].max() > PRIS_MAX + 1e-4:
        aviso.append('d3 fuera del recorrido de pris_joint (mas de 0.1 mm)')
    q[:, 2] = np.clip(q[:, 2], PRIS_MIN, PRIS_MAX)

    xyz = df[['x', 'y', 'z']].to_numpy(float) if {'x', 'y', 'z'} <= set(df.columns) else None
    return {'ruta': ruta, 't': t, 'q': q, 'qd': qd, 'escala': escala, 'xyz': xyz, 'aviso': aviso}


def muestrear(seg, te):
    """Interpolacion cubica de Hermite con q y qd del Excel."""
    t, q, qd = seg['t'], seg['q'], seg['qd']
    if te <= t[0]:
        return q[0].copy(), qd[0].copy()
    if te >= t[-1]:
        return q[-1].copy(), np.zeros(3)
    k = int(np.searchsorted(t, te, side='right') - 1)
    h = t[k + 1] - t[k]
    s = (te - t[k]) / h
    qi = ((2*s**3 - 3*s**2 + 1)*q[k] + (s**3 - 2*s**2 + s)*h*qd[k]
          + (-2*s**3 + 3*s**2)*q[k + 1] + (s**3 - s**2)*h*qd[k + 1])
    qdi = ((6*s**2 - 6*s)*q[k] + (3*s**2 - 4*s + 1)*h*qd[k]
           + (-6*s**2 + 6*s)*q[k + 1] + (3*s**2 - 2*s)*h*qd[k + 1]) / h
    return qi, qdi


class TrajectoryRviz(Node):
    def __init__(self):
        super().__init__('trajectory_rviz')

        self.declare_parameter('archivos', [''])        # vacio = leer toda la carpeta
        self.declare_parameter('carpeta', '')           # vacio = CARPETA_DEFECTO
        self.declare_parameter('pausa', 1.0)            # s entre trayectorias
        self.declare_parameter('escala_tiempo', 0.0)    # 0 = automatico; >1 = mas lento
        self.declare_parameter('d3_offset', D3_OFFSET)
        self.declare_parameter('hz', 50.0)
        self.declare_parameter('inicio_espera', 2.0)    # s mostrando la pose inicial
        self.declare_parameter('repetir', False)
        self.declare_parameter('mostrar_ruta', True)
        self.declare_parameter('marco', 'world')
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
        self.pausa = float(p('pausa').value)
        self.repetir = bool(p('repetir').value)
        self.marco = p('marco').value
        esc = float(p('escala_tiempo').value)
        d3_off = float(p('d3_offset').value)
        tol = float(p('tol_empalme').value)

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
                    f"El final de {a['ruta']} no coincide con el inicio de {b['ruta']} "
                    f'(salto max {salto:.4f}): el robot saltara entre ambas.')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/trayectoria_marker', 10)
        if p('mostrar_ruta').value:
            self.create_timer(1.0, self._publicar_ruta)

        self.q_hold = self.segs[0]['q'][0].copy()
        self.i = -1
        self.fase = 'espera'
        self.t_ini = self._now()
        self.pausa_actual = float(p('inicio_espera').value)
        self.timer = self.create_timer(1.0 / float(p('hz').value), self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publicar_ruta(self):
        arr = MarkerArray()
        for k, seg in enumerate(self.segs):
            if seg['xyz'] is None:
                continue
            m = Marker()
            m.header.frame_id = self.marco
            m.ns = 'trayectoria'
            m.id = k
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.004
            r, g, b = COLORES[k % len(COLORES)]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in seg['xyz']]
            arr.markers.append(m)
        self.pub_mk.publish(arr)

    def _publicar(self, q, qd):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = JOINTS
        m.position = [float(x) for x in q]
        m.velocity = [float(x) for x in qd]
        self.pub.publish(m)

    def _iniciar(self, i):
        if i >= len(self.segs):
            if not self.repetir:
                self.get_logger().info('Todas las trayectorias terminaron.')
                self.fase = 'fin'
                self.timer.cancel()
                return
            i = 0
        self.i = i
        self.get_logger().info(f"Reproduciendo {i + 1}/{len(self.segs)}: {self.segs[i]['ruta']}")
        self.t_ini = self._now()
        self.fase = 'corre'

    def _tick(self):
        now = self._now()
        if self.fase == 'espera':
            self._publicar(self.q_hold, np.zeros(3))
            if now - self.t_ini >= self.pausa_actual:
                self._iniciar(self.i + 1)
            return
        if self.fase != 'corre':
            return

        seg = self.segs[self.i]
        te = now - self.t_ini
        q, qd = muestrear(seg, te)
        self.q_hold = q
        self._publicar(q, qd)

        if te >= seg['t'][-1]:
            self.get_logger().info(f'Trayectoria {self.i + 1} terminada.')
            self.fase = 'espera'
            self.t_ini = now
            self.pausa_actual = self.pausa


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryRviz()
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