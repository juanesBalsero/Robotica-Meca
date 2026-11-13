#!/usr/bin/env python3
"""udp_bridge: único nodo que habla con la ESP32 (UDP + JSON).

Protocolo con la ESP32 (todo en espacio de la ARTICULACIÓN; la reducción
3:1 la maneja la ESP32, aquí NO se aplica):
    ESP32 -> PC : {"q":[deg1,deg2,deg3],"v":[rpm1,rpm2,rpm3],"g":0|1}\n
    PC -> ESP32 : {"w":[rpm1,rpm2,rpm3],"g":0|1}\n      ("g" es opcional)

Hacia ROS todo va en unidades SI (rad, rad/s):
    pub  /joint_states        sensor_msgs/JointState
    pub  /gripper_state       std_msgs/Bool              (último estado ordenado)
    sub  /joint_velocity_cmd  std_msgs/Float64MultiArray ([rad/s] x3)
    sub  /gripper_cmd         std_msgs/Bool              (True = cerrada)
"""
import json
import math
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

DEG_A_RAD = math.pi / 180.0
RPM_A_RADS = 2.0 * math.pi / 60.0
RADS_A_RPM = 60.0 / (2.0 * math.pi)


class UdpBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')

        # ---------------- Parámetros ----------------
        self.declare_parameter('esp32_ip', '')
        self.declare_parameter('esp32_port', 5005)
        self.declare_parameter('local_port', 5006)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('tx_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.1)
        self.declare_parameter('link_timeout_s', 1.0)

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = int(self.get_parameter('esp32_port').value)
        self.local_port = int(self.get_parameter('local_port').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        tx_rate = float(self.get_parameter('tx_rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self.link_timeout = float(self.get_parameter('link_timeout_s').value)
        self.n = len(self.joint_names)

        if not self.esp32_ip:
            raise RuntimeError("Falta el parámetro 'esp32_ip' (IP de la ESP32)")
        self.destino = (self.esp32_ip, self.esp32_port)

        # ---------------- Estado interno ----------------
        self._w_rads = [0.0] * self.n       # último comando de velocidad [rad/s]
        self._t_cmd = float('-inf')         # instante (monotónico) del último comando
        self._cmd_fresco = False
        self._g = None                      # None = aún no se ha ordenado la garra
        self._t_rx = None                   # instante del último paquete de la ESP32
        self._enlace_ok = False

        # ---------------- Socket UDP ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.local_port))
        self.sock.settimeout(0.2)

        # ---------------- ROS ----------------
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_g = self.create_publisher(Bool, '/gripper_state', 10)
        self.create_subscription(Float64MultiArray, '/joint_velocity_cmd', self._cb_vel, 10)
        self.create_subscription(Bool, '/gripper_cmd', self._cb_garra, 10)
        self.create_timer(1.0 / tx_rate, self._enviar)
        self.create_timer(1.0, self._vigilar)

        # ---------------- Hilo de recepción ----------------
        self._stop = threading.Event()
        self._hilo = threading.Thread(target=self._rx_loop, daemon=True)
        self._hilo.start()

        self.get_logger().info(
            'udp_bridge listo | ESP32 {}:{} | escuchando en :{}'.format(
                self.esp32_ip, self.esp32_port, self.local_port))

    # ======================================================
    # ROS -> ESP32
    # ======================================================
    def _cb_vel(self, msg):
        if len(msg.data) != self.n:
            self.get_logger().warning(
                'joint_velocity_cmd debe tener {} valores (llegaron {})'.format(
                    self.n, len(msg.data)), throttle_duration_sec=2.0)
            return
        self._w_rads = [float(x) for x in msg.data]
        self._t_cmd = time.monotonic()

    def _cb_garra(self, msg):
        self._g = 1 if msg.data else 0

    def _tx(self, paquete):
        try:
            self.sock.sendto((json.dumps(paquete) + '\n').encode(), self.destino)
        except OSError as e:
            self.get_logger().warning(
                'No se pudo enviar a la ESP32: {}'.format(e), throttle_duration_sec=2.0)

    def _enviar(self):
        """Reenvía periódicamente el último comando (alimenta el watchdog de la ESP32).
        Si el CC lleva callado más de cmd_timeout_s, o el comando es inválido, manda 0."""
        fresco = (time.monotonic() - self._t_cmd) <= self.cmd_timeout
        w = self._w_rads
        if fresco and not all(math.isfinite(x) for x in w):
            self.get_logger().warning('Comando con NaN/inf: se envía velocidad 0',
                                      throttle_duration_sec=2.0)
            fresco = False
        if fresco != self._cmd_fresco:
            self._cmd_fresco = fresco
            if not fresco and self._t_cmd != float('-inf'):
                self.get_logger().warning('Sin comandos del CC: velocidad 0 hacia la ESP32')
        if not fresco:
            w = [0.0] * self.n

        paquete = {'w': [round(x * RADS_A_RPM, 3) for x in w]}
        if self._g is not None:
            paquete['g'] = self._g
        self._tx(paquete)

    # ======================================================
    # ESP32 -> ROS
    # ======================================================
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                datos, addr = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if addr[0] != self.esp32_ip:
                continue  # ignora paquetes de otros equipos
            self._procesar(datos)

    def _procesar(self, datos):
        for linea in datos.split(b'\n'):
            if not linea.strip():
                continue
            try:
                msg = json.loads(linea.decode())
                q = [float(x) for x in msg['q']]
                v = [float(x) for x in msg['v']]
            except (ValueError, KeyError, TypeError):
                self.get_logger().warning('Paquete inválido de la ESP32',
                                          throttle_duration_sec=2.0)
                continue
            if (len(q) != self.n or len(v) != self.n
                    or not all(math.isfinite(x) for x in q + v)):
                self.get_logger().warning('Paquete con tamaño o valores inválidos',
                                          throttle_duration_sec=2.0)
                continue

            self._marcar_rx()

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = [x * DEG_A_RAD for x in q]
            js.velocity = [x * RPM_A_RADS for x in v]
            self.pub_js.publish(js)

            if 'g' in msg:
                try:
                    self.pub_g.publish(Bool(data=bool(float(msg['g']) > 0)))
                except (ValueError, TypeError):
                    pass

    def _marcar_rx(self):
        primera = self._t_rx is None
        self._t_rx = time.monotonic()
        if not self._enlace_ok:
            self._enlace_ok = True
            self.get_logger().info(
                'ESP32 conectada: primer q recibido (robot listo)' if primera
                else 'Enlace con la ESP32 recuperado')

    def _vigilar(self):
        if self._t_rx is None:
            self.get_logger().info('Esperando datos de la ESP32 (¿homing en curso?)...',
                                   throttle_duration_sec=5.0)
            return
        if self._enlace_ok and (time.monotonic() - self._t_rx) > self.link_timeout:
            self._enlace_ok = False
            self.get_logger().warning('Se perdió el enlace con la ESP32')

    # ======================================================
    # Cierre
    # ======================================================
    def destroy_node(self):
        try:
            self._stop.set()
            for _ in range(3):                      # frena el robot antes de salir
                self._tx({'w': [0.0] * self.n})
            self._hilo.join(timeout=1.0)
            self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UdpBridge()
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
