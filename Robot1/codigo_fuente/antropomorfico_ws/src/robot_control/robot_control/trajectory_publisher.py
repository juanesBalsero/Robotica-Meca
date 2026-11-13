#!/usr/bin/env python3
"""trajectory_publisher: lee el CSV y publica x_d, ẋ_d y la garra.

CSV (sin encabezado): t, x, y, z, vx, vy, vz, garra     [s, m, m/s, 0|1]

Estados:
  espera    -> publica el primer punto con ẋ_d = 0 (el CC mantiene el home)
  corriendo -> recorre la trayectoria
  pausa     -> mantiene el punto actual con ẋ_d = 0
  fin       -> mantiene el último punto con ẋ_d = 0

Servicios (std_srvs/Trigger):
  ~/start : inicia (o reanuda tras ~/stop; si ya terminó, reinicia: el CSV
            termina en el home, igual que empieza)
  ~/stop  : pausa y mantiene la posición

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
from robot_interfaces.msg import TrajectoryPoint
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

    # ======================================================
    def _srv_start(self, request, response):
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
