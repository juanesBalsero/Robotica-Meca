#!/usr/bin/env python3
"""kinematic_control (CC): CLIK a nivel de velocidad.

    e  = x_d - k(q)                 (q MEDIDO, viene de /joint_states)
    q̇  = J⁻¹(q) (ẋ_d + K e)         (DLS cerca de singularidades)

El robot (lazo PID de velocidad + motores) hace de integrador.

Topics:
  sub  /trajectory_point    robot_interfaces/TrajectoryPoint
  sub  /joint_states        sensor_msgs/JointState         (rad, rad/s)
  pub  /joint_velocity_cmd  std_msgs/Float64MultiArray     (rad/s)
  pub  /gripper_cmd         std_msgs/Bool                  (solo cuando cambia)
  pub  /ee_pose             geometry_msgs/PointStamped     x_e = k(q)
  pub  /ee_desired          geometry_msgs/PointStamped     x_d
  pub  /tracking_error      geometry_msgs/Vector3Stamped   e = x_d - x_e
Servicio:
  ~/reset (Trigger): limpia una falla y vuelve a exigir el chequeo de arranque

Protecciones:
  * Armado: no actúa hasta que ||e|| < e_inicio_m (si q no tiene home, FK(q) es
    absurda y el robot saldría disparado: por eso no se mueve).
  * Falla: si ||e|| > e_falla_m con el CC armado, se detiene (latch) hasta ~/reset.
  * Si la trayectoria deja de llegar (traj_timeout_s), se mantiene x_d con ẋ_d = 0.
  * Saturación escalando todo el vector; topes articulares opcionales.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_interfaces.msg import TrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

from robot_control.kinematics import Brazo3GDL, resolver_dls, saturar_vector


class KinematicControl(Node):
    def __init__(self):
        super().__init__('kinematic_control')

        # ---------------- Parámetros ----------------
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('L1', 0.2000)
        self.declare_parameter('L2', 0.1506)
        self.declare_parameter('L3', 0.18464)
        self.declare_parameter('K', [1.0, 1.0, 1.0])
        self.declare_parameter('vel_max_rad_s', [0.5, 0.5, 0.5])
        self.declare_parameter('sing_eps', 0.02)
        self.declare_parameter('lambda_max', 0.02)
        self.declare_parameter('e_inicio_m', 0.02)
        self.declare_parameter('e_falla_m', 0.10)
        self.declare_parameter('traj_timeout_s', 0.5)
        self.declare_parameter('usar_limites', False)
        self.declare_parameter('q_min_deg', [-360.0, -360.0, -360.0])
        self.declare_parameter('q_max_deg', [360.0, 360.0, 360.0])

        gp = lambda k: self.get_parameter(k).value
        self.joint_names = list(gp('joint_names'))
        self.brazo = Brazo3GDL(gp('L1'), gp('L2'), gp('L3'))
        self.K = np.array(gp('K'), dtype=float)
        self.vmax = np.array(gp('vel_max_rad_s'), dtype=float)
        self.sing_eps = float(gp('sing_eps'))
        self.lambda_max = float(gp('lambda_max'))
        self.e_inicio = float(gp('e_inicio_m'))
        self.e_falla = float(gp('e_falla_m'))
        self.traj_timeout = float(gp('traj_timeout_s'))
        self.usar_limites = bool(gp('usar_limites'))
        self.qmin = np.radians(np.array(gp('q_min_deg'), dtype=float))
        self.qmax = np.radians(np.array(gp('q_max_deg'), dtype=float))
        for nombre, arr in (('K', self.K), ('vel_max_rad_s', self.vmax),
                            ('q_min_deg', self.qmin), ('q_max_deg', self.qmax)):
            if arr.shape != (3,):
                raise RuntimeError("El parámetro '{}' debe tener 3 valores".format(nombre))
        if len(self.joint_names) != 3:
            raise RuntimeError("joint_names debe tener 3 nombres")
        if np.any(self.vmax <= 0):
            raise RuntimeError('vel_max_rad_s debe ser > 0')

        # ---------------- Estado ----------------
        self._xd = None
        self._xd_dot = np.zeros(3)
        self._g = None
        self._t_traj = float('-inf')
        self._t_q = None
        self._armado = False
        self._falla = False
        self._g_enviada = None

        # ---------------- ROS ----------------
        self.pub_cmd = self.create_publisher(Float64MultiArray, '/joint_velocity_cmd', 10)
        self.pub_g = self.create_publisher(Bool, '/gripper_cmd', 10)
        self.pub_ee = self.create_publisher(PointStamped, '/ee_pose', 10)
        self.pub_xd = self.create_publisher(PointStamped, '/ee_desired', 10)
        self.pub_err = self.create_publisher(Vector3Stamped, '/tracking_error', 10)
        self.create_subscription(TrajectoryPoint, '/trajectory_point', self._cb_traj, 10)
        self.create_subscription(JointState, '/joint_states', self._cb_js, 10)
        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_timer(1.0, self._vigilar)

        self.get_logger().info(
            'kinematic_control listo | K={} | vel_max={} rad/s | límites={}'.format(
                self.K.tolist(), self.vmax.tolist(),
                'ON' if self.usar_limites else 'OFF'))

    # ======================================================
    # Entradas
    # ======================================================
    def _cb_traj(self, msg):
        xd = np.array(msg.position, dtype=float)
        xdd = np.array(msg.velocity, dtype=float)
        if not (np.all(np.isfinite(xd)) and np.all(np.isfinite(xdd))):
            self.get_logger().warning('TrajectoryPoint con NaN/inf: ignorado',
                                      throttle_duration_sec=2.0)
            return
        self._xd, self._xd_dot, self._g = xd, xdd, bool(msg.gripper)
        self._t_traj = time.monotonic()

    def _extraer_q(self, msg):
        try:
            if msg.name:
                q = [msg.position[msg.name.index(n)] for n in self.joint_names]
            else:
                q = list(msg.position[:3])
        except (ValueError, IndexError):
            self.get_logger().warning('joint_states no trae {}'.format(self.joint_names),
                                      throttle_duration_sec=2.0)
            return None
        q = np.array(q, dtype=float)
        if q.shape != (3,) or not np.all(np.isfinite(q)):
            return None
        return q

    def _srv_reset(self, request, response):
        self._falla = False
        self._armado = False
        self.get_logger().info('CC reiniciado: se volverá a exigir el chequeo de arranque')
        response.success = True
        response.message = 'CC desarmado y sin falla'
        return response

    # ======================================================
    # Lazo de control: se ejecuta con cada q medido
    # ======================================================
    def _cb_js(self, msg):
        q = self._extraer_q(msg)
        if q is None:
            return
        self._t_q = time.monotonic()
        if self._xd is None:
            return                                   # aún no hay referencia

        xe, J = self.brazo.fk_jacobiano(q)
        xd = self._xd
        xd_dot = self._xd_dot
        if time.monotonic() - self._t_traj > self.traj_timeout:
            xd_dot = np.zeros(3)                     # trayectoria muda: mantener posición
            self.get_logger().warning('Sin puntos de trayectoria: se mantiene x_d con ẋ_d = 0',
                                      throttle_duration_sec=2.0)
        e = xd - xe
        err = float(np.linalg.norm(e))
        self._publicar_estado(xe, xd, e)

        # ---- Armado / falla ----
        if self._falla:
            self._pub_cmd(np.zeros(3))
            return
        if not self._armado:
            if err <= self.e_inicio:
                self._armado = True
                self.get_logger().info(
                    'CC armado (error inicial {:.1f} mm)'.format(err * 1000.0))
            else:
                self.get_logger().warning(
                    'CC sin armar: ||e|| = {:.3f} m > {:.3f} m. ¿Se hizo el home? '
                    'x_e = {}'.format(err, self.e_inicio, np.round(xe, 3).tolist()),
                    throttle_duration_sec=2.0)
                self._pub_cmd(np.zeros(3))
                return
        elif err > self.e_falla:
            self._falla = True
            self.get_logger().error(
                'FALLA: error de seguimiento {:.3f} m > {:.3f} m. CC detenido. '
                'Usa ~/reset para reiniciarlo.'.format(err, self.e_falla))
            self._pub_cmd(np.zeros(3))
            return

        # ---- Garra: se pasa tal cual, solo cuando cambia ----
        if self._g is not None and self._g != self._g_enviada:
            self._g_enviada = self._g
            self.pub_g.publish(Bool(data=self._g))

        # ---- CLIK ----
        qdot, sigma_min, lam2 = resolver_dls(J, xd_dot + self.K * e,
                                             self.sing_eps, self.lambda_max)
        if lam2 > 0.0:
            self.get_logger().warning(
                'Cerca de singularidad (σmin = {:.4f}): usando DLS'.format(sigma_min),
                throttle_duration_sec=1.0)
        qdot, factor = saturar_vector(qdot, self.vmax)
        if factor < 1.0:
            self.get_logger().warning(
                'Velocidad articular saturada (x{:.2f})'.format(factor),
                throttle_duration_sec=2.0)
        if self.usar_limites:
            qdot = self._aplicar_limites(q, qdot)

        if not np.all(np.isfinite(qdot)):
            self.get_logger().error('q̇ no finito: se envía 0')
            qdot = np.zeros(3)
        self._pub_cmd(qdot)

    def _aplicar_limites(self, q, qdot):
        qdot = qdot.copy()
        for i in range(3):
            if (q[i] <= self.qmin[i] and qdot[i] < 0.0) or \
               (q[i] >= self.qmax[i] and qdot[i] > 0.0):
                qdot[i] = 0.0
                self.get_logger().warning(
                    'Articulación {} en su tope: velocidad bloqueada'.format(i + 1),
                    throttle_duration_sec=1.0)
        return qdot

    # ======================================================
    # Salidas
    # ======================================================
    def _pub_cmd(self, qdot):
        self.pub_cmd.publish(Float64MultiArray(data=[float(x) for x in qdot]))

    def _publicar_estado(self, xe, xd, e):
        stamp = self.get_clock().now().to_msg()
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = 'base_link'
        p.point.x, p.point.y, p.point.z = (float(v) for v in xe)
        self.pub_ee.publish(p)
        d = PointStamped()
        d.header.stamp = stamp
        d.header.frame_id = 'base_link'
        d.point.x, d.point.y, d.point.z = (float(v) for v in xd)
        self.pub_xd.publish(d)
        v = Vector3Stamped()
        v.header.stamp = stamp
        v.header.frame_id = 'base_link'
        v.vector.x, v.vector.y, v.vector.z = (float(c) for c in e)
        self.pub_err.publish(v)

    def _vigilar(self):
        if self._t_q is None:
            self.get_logger().info('Esperando /joint_states (¿puente y ESP32 en marcha?)...',
                                   throttle_duration_sec=5.0)
        elif time.monotonic() - self._t_q > 1.0:
            self.get_logger().warning('No llegan /joint_states desde hace más de 1 s',
                                      throttle_duration_sec=3.0)
        if self._xd is None:
            self.get_logger().info('Esperando /trajectory_point...',
                                   throttle_duration_sec=5.0)

    def destroy_node(self):
        try:
            self._pub_cmd(np.zeros(3))
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicControl()
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
