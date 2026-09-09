#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer

class ScaraGuiPresets(Node):
    def __init__(self):
        super().__init__('gui_presets_node')

        # Publicador directo idéntico a tu arquitectura funcional
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Posición deseada actual [rot1, rot2, pris, efector]
        # efector siempre en 0.0: no tiene recorrido real (limit lower=0 upper=0 en el URDF)
        self.current_positions = [0.0, 0.0, 0.0, 0.0]

        # Publicar estado de las articulaciones a 20 Hz (cada 0.05 s)
        self.timer = self.create_timer(0.05, self.publish_joints)
        self.get_logger().info("Nodo GUI Presets iniciado correctamente para SCARA")

    def set_preset(self, r1, r2, p_val):
        # efector se mantiene fijo en 0.0
        self.current_positions = [float(r1), float(r2), float(p_val), 0.0]
        self.get_logger().info(f"Nuevo Target -> rot1: {r1}, rot2: {r2}, pris: {p_val}")

    def publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['rot1', 'rot2', 'pris', 'efector']
        js.position = self.current_positions
        self.joint_pub.publish(js)


class PresetsWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Control SCARA - Presets')
        self.resize(300, 160)

        layout = QVBoxLayout()

        self.label = QLabel('Selecciona una posición objetivo:', self)
        layout.addWidget(self.label)

        # Botón Band3 (rot1=-3.2289, rot2=4.2380, pris=0.131)
        btn_band3 = QPushButton('Ir a band3', self)
        btn_band3.clicked.connect(lambda: self.node.set_preset(-3.2289, 4.2380, 0.075))
        layout.addWidget(btn_band3)

        # Botón Est2 (rot1=-0.1464, rot2=4.0046, pris=0.131)
        btn_est2 = QPushButton('Ir a est2', self)
        btn_est2.clicked.connect(lambda: self.node.set_preset(-0.1464, 4.0046, 0.131))
        layout.addWidget(btn_est2)

        self.setLayout(layout)


def main(args=None):
    rclpy.init(args=args)
    ros_node = ScaraGuiPresets()

    app = QApplication(sys.argv)
    gui = PresetsWindow(ros_node)
    gui.show()

    # Loop para integrar Qt con el spin_once de ROS 2 sin bloqueos
    q_timer = QTimer()
    q_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.001))
    q_timer.start(10)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()