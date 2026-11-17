#!/usr/bin/env python3
"""robot1_node: cumple el contrato de la planta para el Robot 1.

Recibe del brain /robot_1/cmd, /planta/estop y /brain/latido, y publica
/robot_1/estado. Mueve el robot llamando a los servicios de trajectory_publisher
(~/start, ~/stop, ~/reset). No modifica a udp_bridge ni a kinematic_control.
La lógica está en robot1_core.py.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt32
from std_srvs.srv import Trigger

from planta_interfaces.msg import RobotCmd, RobotEstado

from robot_control.robot1_core import Parametros, Robot1Core

TRAJ = '/trajectory_publisher'


def _qos(depth=10, transient_local=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class Robot1Node(Node):
    def __init__(self):
        super().__init__('robot1_node')
        d = Parametros()
        for nombre in vars(d):
            self.declare_parameter(nombre, getattr(d, nombre))
        self.declare_parameter('periodo_estado_s', 1.0)
        params = Parametros(**{n: float(self.get_parameter(n).value) for n in vars(d)})
        self.core = Robot1Core(params, time.monotonic())
        self.get_logger().info(f'Parametros: {params}')

        # Hacia el brain
        self.pub_estado = self.create_publisher(RobotEstado, '/robot_1/estado', _qos())
        # Desde el brain / GUI
        self.create_subscription(RobotCmd, '/robot_1/cmd', self._on_cmd, _qos())
        self.create_subscription(Bool, '/planta/estop', self._on_estop, _qos(1, True))
        self.create_subscription(UInt32, '/brain/latido', self._on_latido, _qos(1))
        # Desde los nodos existentes del robot
        self.create_subscription(String, f'{TRAJ}/estado', self._on_traj, _qos(1, True))
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(Vector3Stamped, '/tracking_error', self._on_err, 10)
        # Servicios de trajectory_publisher
        self.clientes = {n: self.create_client(Trigger, f'{TRAJ}/{n}') for n in ('start', 'stop', 'reset')}

        self.create_timer(0.05, self._tick)
        self.create_timer(float(self.get_parameter('periodo_estado_s').value), self._publicar)
        self.get_logger().info('robot1_node listo: esperando enlace con la ESP32 y trajectory_publisher')

    @staticmethod
    def _ahora():
        return time.monotonic()

    # ---------------------------------------------------------------- entradas
    def _on_cmd(self, m):
        if m.robot_id not in (0, 1):
            return
        self.get_logger().info(f'Orden: {m.tarea} caja={m.caja_id} seq={m.seq}')
        self.core.on_cmd(m.tarea, int(m.caja_id), int(m.seq), self._ahora())
        self._volcar()

    def _on_estop(self, m):
        self.core.on_estop(bool(m.data), self._ahora())
        self._volcar()

    def _on_latido(self, m):
        self.core.on_latido(self._ahora())

    def _on_traj(self, m):
        self.core.on_traj_estado(m.data, self._ahora())
        self._volcar()

    def _on_js(self, m):
        self.core.on_joint_states(self._ahora())

    def _on_err(self, m):
        v = m.vector
        self.core.on_tracking_error(math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z), self._ahora())

    # ---------------------------------------------------------------- periódicos
    def _tick(self):
        self.core.tick(self._ahora())
        self._volcar()

    # ---------------------------------------------------------------- salidas
    def _publicar(self):
        """También se llama cada 1 s: el estado repetido sirve de latido hacia el brain."""
        datos = self.core.mensaje()
        if datos is None:
            return
        msg = RobotEstado()
        msg.robot_id, msg.estado, msg.caja_id, msg.seq = datos
        self.pub_estado.publish(msg)

    def _llamar(self, nombre):
        cli = self.clientes[nombre]
        if not cli.service_is_ready():
            self.core.on_srv_resultado(nombre, False, 'servicio no disponible', self._ahora())
            return
        futuro = cli.call_async(Trigger.Request())
        futuro.add_done_callback(lambda f, n=nombre: self._respuesta(n, f))

    def _respuesta(self, nombre, futuro):
        try:
            r = futuro.result()
            ok, mensaje = bool(r.success), r.message
        except Exception as e:  # noqa: BLE001
            ok, mensaje = False, f'servicio no disponible ({e})'
        self.get_logger().info(f'{nombre}: {"ok" if ok else "fallo"} - {mensaje}')
        self.core.on_srv_resultado(nombre, ok, mensaje, self._ahora())
        self._volcar()

    def _volcar(self):
        while True:
            salidas = self.core.retirar_salidas()
            if not salidas:
                return
            for s in salidas:
                if s.tipo == 'publicar':
                    self._publicar()
                elif s.tipo == 'srv':
                    self._llamar(s.dato)
                else:
                    self.get_logger().info(s.dato)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Robot1Node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
