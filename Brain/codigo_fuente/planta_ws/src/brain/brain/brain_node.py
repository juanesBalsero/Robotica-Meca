#!/usr/bin/env python3
"""Nodo ROS 2 del brain: traduce mensajes a llamadas del nucleo y publica sus salidas.

Toda la logica esta en brain_core.py. Este archivo solo conecta los tópicos y
servicios del contrato (ver README). El ejecutor por defecto de rclpy atiende
un callback a la vez, asi que no hacen falta bloqueos.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32
from std_srvs.srv import Trigger

from planta_interfaces.msg import Alerta, EstadoPlanta, Progreso, RobotCmd, RobotEstado
from planta_interfaces.srv import IniciarCaja

from brain.brain_core import N_ROBOTS, NucleoBrain, Parametros


def _qos(depth=10, transient_local=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain')

        # Parametros (config/brain_params.yaml). En el YAML usar decimales: 30.0, no 30.
        self.declare_parameter('t_ack', 2.0)
        self.declare_parameter('max_intentos', 3)
        self.declare_parameter('t_tarea', [30.0] * N_ROBOTS)
        self.declare_parameter('t_offline', 3.0)
        self.declare_parameter('periodo_latido', 1.0)
        self.declare_parameter('periodo_estado', 1.0)
        self.declare_parameter('periodo_tick', 0.1)

        params = Parametros(
            t_ack=float(self.get_parameter('t_ack').value),
            max_intentos=int(self.get_parameter('max_intentos').value),
            t_tarea=tuple(float(x) for x in self.get_parameter('t_tarea').value),
            t_offline=float(self.get_parameter('t_offline').value),
        )
        self.nucleo = NucleoBrain(params, self._ahora())
        self.get_logger().info(
            f'Parametros: t_ack={params.t_ack} s, max_intentos={params.max_intentos}, '
            f't_tarea={list(params.t_tarea)}, t_offline={params.t_offline} s')

        # Robots: ordenes (brain -> robot) y estados (robot -> brain)
        self.pub_cmd = {}
        for i in range(1, N_ROBOTS + 1):
            self.pub_cmd[i] = self.create_publisher(RobotCmd, f'/robot_{i}/cmd', _qos())
            self.create_subscription(
                RobotEstado, f'/robot_{i}/estado', lambda m, i=i: self._on_estado(i, m), _qos())

        # Latido y parada global
        self.pub_latido = self.create_publisher(UInt32, '/brain/latido', _qos(depth=1))
        self.create_subscription(Bool, '/planta/estop', self._on_estop, _qos(depth=1, transient_local=True))

        # GUI
        self.pub_progreso = self.create_publisher(Progreso, '/planta/progreso', _qos())
        self.pub_alerta = self.create_publisher(Alerta, '/planta/alerta', _qos())
        self.pub_estado = self.create_publisher(EstadoPlanta, '/planta/estado', _qos())
        self.create_service(IniciarCaja, '/gui/iniciar_caja', self._srv_iniciar_caja)
        self.create_service(Trigger, '/gui/continuar', self._srv_continuar)

        self._contador_latido = 0
        self.create_timer(float(self.get_parameter('periodo_tick').value), self._tick)
        self.create_timer(float(self.get_parameter('periodo_latido').value), self._latido)
        self.create_timer(float(self.get_parameter('periodo_estado').value), self._publicar_estado)
        self.get_logger().info('Brain listo. Planta en OPERANDO.')

    @staticmethod
    def _ahora():
        return time.monotonic()

    # ---------------------------------------------------------------- entradas
    def _on_estado(self, i, msg):
        if msg.robot_id != i:
            self.get_logger().warn(f'/robot_{i}/estado trae robot_id={msg.robot_id}; se usa {i}',
                                   throttle_duration_sec=5.0)
        self.nucleo.on_estado(i, int(msg.estado), int(msg.caja_id), int(msg.seq), self._ahora())
        self._volcar()

    def _on_estop(self, msg):
        self.nucleo.on_estop(bool(msg.data), self._ahora())
        self._volcar()

    def _srv_iniciar_caja(self, request, response):
        ok, motivo = self.nucleo.iniciar_caja(int(request.caja_id), request.medicamento, self._ahora())
        response.aceptada, response.motivo = ok, motivo
        self._volcar()
        return response

    def _srv_continuar(self, request, response):
        ok, mensaje = self.nucleo.continuar(self._ahora())
        response.success, response.message = ok, mensaje
        self._volcar()
        return response

    # ---------------------------------------------------------------- periodicos
    def _tick(self):
        self.nucleo.tick(self._ahora())
        self._volcar()

    def _latido(self):
        # Siempre se publica, incluso en FALLO o PARADA_GLOBAL: si no, los robots creerian que el brain cayo
        self._contador_latido += 1
        self.pub_latido.publish(UInt32(data=self._contador_latido))

    def _publicar_estado(self):
        s = self.nucleo.snapshot()
        msg = EstadoPlanta()
        msg.estado = s['estado']
        msg.estados_robots = s['estados_robots']
        msg.cajas_robots = s['cajas_robots']
        msg.cajas_en_cola = s['cajas_en_cola']
        self.pub_estado.publish(msg)

    # ---------------------------------------------------------------- salidas
    def _volcar(self):
        for salida in self.nucleo.retirar_salidas():
            d = salida.datos
            if salida.tipo == 'cmd':
                msg = RobotCmd()
                msg.robot_id, msg.caja_id, msg.tarea = d['robot_id'], d['caja_id'], d['tarea']
                msg.medicamento, msg.seq = d['medicamento'], d['seq']
                self.pub_cmd[d['robot_id']].publish(msg)
            elif salida.tipo == 'progreso':
                msg = Progreso()
                msg.caja_id, msg.etapa, msg.porcentaje = d['caja_id'], d['etapa'], d['porcentaje']
                self.pub_progreso.publish(msg)
            elif salida.tipo == 'alerta':
                msg = Alerta()
                msg.robot_id, msg.caja_id, msg.causa = d['robot_id'], d['caja_id'], d['causa']
                self.pub_alerta.publish(msg)
            else:  # log
                self.get_logger().info(salida.datos)


def main(args=None):
    rclpy.init(args=args)
    nodo = BrainNode()
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
