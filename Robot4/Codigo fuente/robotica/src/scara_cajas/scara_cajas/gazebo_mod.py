#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QTimer

JOINTS = ['rot1_joint', 'rot2_joint', 'pris_joint']
LIMITS = {
    'rot1_joint': (-3.1416, 3.1416),
    'rot2_joint': (-1.1, 4.25),
    'pris_joint': (0.0, 0.131),
}

class GazeboModNode(Node):
    def __init__(self):
        super().__init__('gazebo_mod_node')
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/scara_position_controller/commands', 10)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        self.commands = [0.0, 0.0, 0.0]
        self.latest_state = {}  # {joint_name: (pos, vel, eff)}

    def publish_command(self):
        msg = Float64MultiArray()
        msg.data = self.commands
        self.cmd_pub.publish(msg)

    def joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            self.latest_state[name] = (pos, vel, eff)


class GazeboModWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Control SCARA Cajas - Gazebo')
        self.resize(420, 380)
        layout = QVBoxLayout()

        self.sliders = []
        self.slider_labels = []
        for i, jname in enumerate(JOINTS):
            lo, hi = LIMITS[jname]
            lbl = QLabel(f'{jname}: 0.000')
            layout.addWidget(lbl)
            self.slider_labels.append(lbl)

            sld = QSlider(Qt.Horizontal)
            sld.setMinimum(int(lo * 1000))
            sld.setMaximum(int(hi * 1000))
            sld.setValue(0)
            sld.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx, val))
            layout.addWidget(sld)
            self.sliders.append(sld)

        layout.addWidget(QLabel('---- Estado en tiempo real (Gazebo) ----'))

        self.state_labels = {}
        for jname in JOINTS:
            row = QHBoxLayout()
            lbl = QLabel(f'{jname} -> pos: 0.000  vel: 0.000  effort: 0.000')
            row.addWidget(lbl)
            layout.addLayout(row)
            self.state_labels[jname] = lbl

        self.setLayout(layout)

        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_state_labels)
        self.refresh_timer.start(100)  # 10 Hz

    def on_slider_change(self, idx, val):
        pos = val / 1000.0
        self.node.commands[idx] = pos
        self.slider_labels[idx].setText(f'{JOINTS[idx]}: {pos:.3f}')
        self.node.publish_command()

    def refresh_state_labels(self):
        for jname in JOINTS:
            if jname in self.node.latest_state:
                pos, vel, eff = self.node.latest_state[jname]
                self.state_labels[jname].setText(
                    f'{jname} -> pos: {pos:.3f}  vel: {vel:.3f}  effort: {eff:.3f}'
                )


def main(args=None):
    rclpy.init(args=args)
    ros_node = GazeboModNode()

    app = QApplication(sys.argv)
    gui = GazeboModWindow(ros_node)
    gui.show()

    q_timer = QTimer()
    q_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.001))
    q_timer.start(10)

    exit_code = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()