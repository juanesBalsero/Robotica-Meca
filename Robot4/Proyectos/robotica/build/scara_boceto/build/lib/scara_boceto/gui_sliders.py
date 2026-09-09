#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QTimer

JOINTS = ['rot1', 'rot2', 'pris']
LIMITS = {
    'rot1': (-3.1416, 3.1416),
    'rot2': (-0.5, 3.3),
    'pris': (-0.131, 0.0),
}

class GuiSlidersNode(Node):
    def __init__(self):
        super().__init__('gui_sliders_node')
        self.pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.positions = [0.0, 0.0, 0.0]

    def publish(self):
        msg = Float64MultiArray()
        msg.data = self.positions
        self.pub.publish(msg)


class SlidersWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Control SCARA Boceto - Gazebo')
        self.resize(350, 220)
        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []
        for i, jname in enumerate(JOINTS):
            lo, hi = LIMITS[jname]
            lbl = QLabel(f'{jname}: 0.000')
            layout.addWidget(lbl)
            self.labels.append(lbl)

            sld = QSlider(Qt.Horizontal)
            sld.setMinimum(int(lo * 1000))
            sld.setMaximum(int(hi * 1000))
            sld.setValue(0)
            sld.valueChanged.connect(lambda val, idx=i: self.on_change(idx, val))
            layout.addWidget(sld)
            self.sliders.append(sld)

        self.setLayout(layout)

    def on_change(self, idx, val):
        pos = val / 1000.0
        self.node.positions[idx] = pos
        self.labels[idx].setText(f'{JOINTS[idx]}: {pos:.3f}')
        self.node.publish()


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiSlidersNode()

    app = QApplication(sys.argv)
    gui = SlidersWindow(ros_node)
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