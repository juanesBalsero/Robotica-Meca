#!/usr/bin/env python3
"""Nodo ROS 2 con robots de prueba que cumplen el contrato (ver robot_falso.py).

Permite probar el brain sin los grupos reales. Con el parametro `robots` se
eligen cuales simular (por ejemplo [2, 3, 4, 5] si el robot 1 es el real).

Fallos en caliente (valen 0 = ninguno):
  ros2 param set /robots_falsos robot_mudo 3        # el robot 3 no confirma ordenes
  ros2 param set /robots_falsos robot_error 4       # el robot 4 reporta error...
  ros2 param set /robots_falsos caja_error 2        # ...al ejecutar la caja 2
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32

from planta_interfaces.msg import RobotCmd, RobotEstado

from brain.brain_core import N_ROBOTS
from brain.robot_falso import RobotFalso


def _qos(depth=10, transient_local=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class RobotsFalsos(Node):
    PERIODO_TICK = 0.1

    def __init__(self):
        super().__init__('robots_falsos')
        self.declare_parameter('robots', list(range(1, N_ROBOTS + 1)))
        self.declare_parameter('duraciones', [5.0] * N_ROBOTS)
        self.declare_parameter('robot_mudo', 0)
        self.declare_parameter('robot_error', 0)
        self.declare_parameter('caja_error', 0)

        duraciones = [float(x) for x in self.get_parameter('duraciones').value]
        self.robots = {}
        self.pubs = {}
        for i in (int(x) for x in self.get_parameter('robots').value):
            self.robots[i] = RobotFalso(i, duracion=duraciones[i - 1])
            self.pubs[i] = self.create_publisher(RobotEstado, f'/robot_{i}/estado', _qos())
            self.create_subscription(RobotCmd, f'/robot_{i}/cmd', lambda m, i=i: self._on_cmd(i, m), _qos())
        self.create_subscription(Bool, '/planta/estop', self._on_estop, _qos(depth=1, transient_local=True))
        self.create_subscription(UInt32, '/brain/latido', self._on_latido, _qos(depth=1))

        self._t_anterior = time.monotonic()
        self.create_timer(self.PERIODO_TICK, self._tick)
        self.create_timer(1.0, self._publicar_todos)  # el estado se repite cada 1 s (sirve de latido)
        self.get_logger().info(f'Robots falsos activos: {sorted(self.robots)}')

    def _publicar(self, i):
        robot_id, estado, caja_id, seq = self.robots[i].mensaje()
        msg = RobotEstado()
        msg.robot_id, msg.estado, msg.caja_id, msg.seq = robot_id, estado, caja_id, seq
        self.pubs[i].publish(msg)

    def _publicar_todos(self):
        for i in self.robots:
            self._publicar(i)

    def _on_cmd(self, i, msg):
        self.robots[i].on_cmd(msg.tarea, int(msg.caja_id), int(msg.seq), msg.medicamento)
        self.get_logger().info(f'Robot {i} recibio {msg.tarea} caja={msg.caja_id} seq={msg.seq}')
        self._publicar_si_cambio()

    def _on_estop(self, msg):
        for r in self.robots.values():
            r.on_estop(bool(msg.data))

    def _on_latido(self, msg):
        ahora = time.monotonic()
        for r in self.robots.values():
            r.on_latido(ahora)

    def _tick(self):
        ahora = time.monotonic()
        dt, self._t_anterior = ahora - self._t_anterior, ahora
        mudo = int(self.get_parameter('robot_mudo').value)
        err_robot = int(self.get_parameter('robot_error').value)
        err_caja = int(self.get_parameter('caja_error').value)
        for i, r in self.robots.items():
            r.mudo = (i == mudo)
            r.error_en_caja = err_caja if i == err_robot else 0
            r.tick(dt, ahora)
        self._publicar_si_cambio()

    def _publicar_si_cambio(self):
        for i, r in self.robots.items():
            if r.publicar:
                r.publicar = False
                self._publicar(i)


def main(args=None):
    rclpy.init(args=args)
    nodo = RobotsFalsos()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
