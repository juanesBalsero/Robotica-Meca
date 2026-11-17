#!/usr/bin/env python3
"""trajectory_publisher: lee el CSV y publica x_d, ẋ_d y la garra.

CSV (sin encabezado): t, x, y, z, vx, vy, vz, garra     [s, m, m/s, 0|1]

Estados:
  espera    -> publica el primer punto con ẋ_d = 0 (el CC mantiene el home)
  corriendo -> recorre la trayectoria
  pausa     -> mantiene el punto actual con ẋ_d = 0
  fin       -> mantiene el último punto con ẋ_d = 0
  volviendo -> (tras ~/reset) regresa en línea recta al primer punto a
               vel_retorno_m_s y pasa a 'espera'

Servicios (std_srvs/Trigger):
  ~/start : inicia (o reanuda tras ~/stop; si ya terminó, reinicia: el CSV
            termina en el home, igual que empieza)
  ~/stop  : pausa y mantiene la posición
  ~/reset : (planta) desde 'pausa' o 'fin' vuelve al inicio sin saltos de x_d

Topic (planta): ~/estado (std_msgs/String, confiable + transient local) con el
estado actual; se publica al cambiar y se repite cada 1 s.

escala_velocidad < 1 reproduce más lento: el tiempo de la trayectoria avanza a
esa fracción y ẋ_d se multiplica por ella (consistente con x_d(t)).
"""
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import TrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('frecuencia_hz', 100.0)
        self.declare_parameter('escala_velocidad', 0.2)

        ruta = self.get_parameter('csv_path').value
        if not ruta:
            ruta = os.path.join(get_package_share_directory('robot_control'),
                                'data', 'trayectoria_robot.csv')
        freq = float(self.get_parameter('frecuencia_hz').value)
        self.escala = float(self.get_parameter('escala_velocidad').value)
        if freq <= 0.0 or self.escala <= 0.0:
            raise RuntimeError('frecuencia_hz y escala_velocidad deben ser > 0')

        self._cargar_csv(ruta)

        self.pub = self.create_publisher(TrajectoryPoint, '/trajectory_point', 10)
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        # --- Añadido para la integración con la planta (robot1_node) ---
        self.declare_parameter('vel_retorno_m_s', 0.05)
        self.vel_retorno = float(self.get_parameter('vel_retorno_m_s').value)
        if self.vel_retorno <= 0.0:
            raise RuntimeError('vel_retorno_m_s debe ser > 0')
        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.pub_estado = self.create_publisher(
            String, '~/estado',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self._estado_pub = None
        self._t_pub_estado = 0.0
        self._ret_p = np.array(self.pos[0], dtype=float)
        self._ret_g = False
        # ---------------------------------------------------------------

        self.estado = 'espera'
        self.t_traj = 0.0                    # tiempo de la trayectoria [s]
        self._t_prev = time.monotonic()
        self._periodo = 1.0 / freq
        self.create_timer(self._periodo, self._tick)

        self.get_logger().info(
            'trajectory_publisher listo | {} puntos, T = {:.2f} s | escala {:.2f} '
            '(duración real {:.1f} s) | llama a ~/start'.format(
                len(self.t), self.T, self.escala, self.T / self.escala))

    # ======================================================
    def _cargar_csv(self, ruta):
        try:
            d = np.loadtxt(ruta, delimiter=',')
        except (OSError, ValueError) as e:
            raise RuntimeError("No se pudo leer el CSV '{}': {}".format(ruta, e))
        if d.ndim != 2 or d.shape[1] != 8:
            raise RuntimeError('El CSV debe tener 8 columnas: t,x,y,z,vx,vy,vz,garra')
        # Quita filas con tiempo repetido (puntos de unión entre tramos)
        keep = np.concatenate(([True], np.diff(d[:, 0]) > 1e-9))
        d = d[keep]
        if len(d) < 2 or not np.all(np.isfinite(d)):
            raise RuntimeError('CSV inválido (pocas filas o valores no finitos)')
        self.t = d[:, 0] - d[0, 0]
        self.pos = d[:, 1:4]
        self.vel = d[:, 4:7]
        self.g = d[:, 7] > 0.5
        self.T = float(self.t[-1])
        self.get_logger().info('CSV cargado: {}'.format(ruta))

    def _muestra(self, t):
        """Interpola posición y velocidad en t; la garra se mantiene (ZOH)."""
        p = [float(np.interp(t, self.t, self.pos[:, k])) for k in range(3)]
        v = [float(np.interp(t, self.t, self.vel[:, k])) for k in range(3)]
        idx = int(np.searchsorted(self.t, t, side='right')) - 1
        idx = min(max(idx, 0), len(self.t) - 1)
        return p, v, bool(self.g[idx])

    # ======================================================
    def _tick(self):
        ahora = time.monotonic()
        dt = min(ahora - self._t_prev, 3.0 * self._periodo)   # evita saltos si hubo un tirón
        self._t_prev = ahora

        if self.estado == 'volviendo':          # añadido (planta): retorno tras ~/reset
            self._tick_retorno(dt)
            self._publicar_estado_si_cambia()
            return

        moviendo = False
        if self.estado == 'corriendo':
            self.t_traj += self.escala * dt
            if self.t_traj >= self.T:
                self.t_traj = self.T
                self.estado = 'fin'
                self.get_logger().info('Trayectoria terminada: se mantiene el último punto')
            else:
                moviendo = True
                self.get_logger().info('t = {:.1f} / {:.1f} s'.format(self.t_traj, self.T),
                                       throttle_duration_sec=2.0)

        p, v, g = self._muestra(self.t_traj)
        if not moviendo:
            v = [0.0, 0.0, 0.0]
        else:
            v = [x * self.escala for x in v]      # d/dt de x_d(escala * t)

        msg = TrajectoryPoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.position = p
        msg.velocity = v
        msg.gripper = g
        self.pub.publish(msg)
        self._publicar_estado_si_cambia()       # añadido (planta)

    # ======================================================
    def _srv_start(self, request, response):
        if self.estado == 'volviendo':          # añadido (planta)
            response.success, response.message = False, 'Está volviendo al inicio'
            return response
        if self.estado == 'corriendo':
            response.success, response.message = False, 'Ya está corriendo'
            return response
        if self.estado == 'fin':
            self.t_traj = 0.0
        self.estado = 'corriendo'
        self._t_prev = time.monotonic()
        self.get_logger().info('START (escala {:.2f})'.format(self.escala))
        response.success, response.message = True, 'Trayectoria iniciada'
        return response

    def _srv_stop(self, request, response):
        if self.estado != 'corriendo':
            response.success, response.message = False, 'No está corriendo'
            return response
        self.estado = 'pausa'
        self.get_logger().info('STOP en t = {:.2f} s: se mantiene la posición'.format(self.t_traj))
        response.success, response.message = True, 'Pausado'
        return response

    # ======================================================
    # Añadido para la integración con la planta (robot1_node)
    def _srv_reset(self, request, response):
        if self.estado == 'corriendo':
            response.success, response.message = False, 'Está corriendo: usa ~/stop antes'
        elif self.estado == 'volviendo':
            response.success, response.message = False, 'Ya está volviendo al inicio'
        elif self.estado == 'espera':
            response.success, response.message = True, 'Ya está en espera'
        else:                                    # 'pausa' o 'fin'
            p, _, g = self._muestra(self.t_traj)
            self._ret_p = np.array(p, dtype=float)
            self._ret_g = g                      # la garra no cambia durante el retorno
            self.estado = 'volviendo'
            self._t_prev = time.monotonic()
            self.get_logger().info('RESET desde t = {:.2f} s: vuelvo al inicio a {:.2f} m/s'.format(
                self.t_traj, self.vel_retorno))
            response.success, response.message = True, 'Volviendo al inicio'
        return response

    def _tick_retorno(self, dt):
        """Vuelve en línea recta al primer punto con ẋ_d consistente (sin saltos de x_d)."""
        d = self.pos[0] - self._ret_p
        dist = float(np.linalg.norm(d))
        paso = self.vel_retorno * dt
        if dist <= max(paso, 1e-6):
            self._ret_p = np.array(self.pos[0], dtype=float)
            v = [0.0, 0.0, 0.0]
            self.t_traj = 0.0
            self.estado = 'espera'
            self.get_logger().info('RESET: de vuelta en el inicio, en espera')
        else:
            u = d / dist
            self._ret_p = self._ret_p + u * paso
            v = [float(x) for x in u * self.vel_retorno]
        msg = TrajectoryPoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.position = [float(x) for x in self._ret_p]
        msg.velocity = v
        msg.gripper = self._ret_g
        self.pub.publish(msg)

    def _publicar_estado_si_cambia(self):
        ahora = time.monotonic()
        if self.estado != self._estado_pub or ahora - self._t_pub_estado >= 1.0:
            self._estado_pub = self.estado
            self._t_pub_estado = ahora
            self.pub_estado.publish(String(data=self.estado))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        print('ERROR:', e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
