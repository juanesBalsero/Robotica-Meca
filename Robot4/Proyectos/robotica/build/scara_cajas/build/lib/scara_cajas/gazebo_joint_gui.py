#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class GazeboJointGUI(Node):
    def __init__(self):
        super().__init__('gazebo_joint_gui')
        
        # Publicador hacia el controlador de ros2_control en Gazebo
        self.pub = self.create_subscription if False else self.create_publisher(
            Float64MultiArray, 
            '/scara_position_controller/commands', 
            10
        )
        
        # Suscriptor para leer la telemetría real de Gazebo
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.joint_data = {}

    def send_positions(self, rot1, rot2, pris):
        msg = Float64MultiArray()
        msg.data = [float(rot1), float(rot2), float(pris)]
        self.pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            self.joint_data[name] = (pos, vel, eff)

class AppUI:
    def __init__(self, root, node: GazeboJointGUI):
        self.root = root
        self.node = node
        self.root.title("Control de Articulaciones - Gazebo ROS 2")
        self.root.geometry("520x420")

        # Configuración de los rangos según el URDF
        self.joints = [
            ("rot1_joint", -3.1416, 3.1416, 0.0),
            ("rot2_joint", -1.1, 4.25, 0.0),
            ("pris_joint", 0.0, 0.131, 0.0)
        ]
        
        self.sliders = {}
        self.labels_telemetry = {}

        for name, min_val, max_val, init_val in self.joints:
            frame = ttk.LabelFrame(root, text=f" Articulación: {name} ")
            frame.pack(fill="x", padx=10, pady=5)

            # Slider
            slider = ttk.Scale(
                frame, 
                from_=min_val, 
                to=max_val, 
                orient="horizontal", 
                command=lambda val, n=name: self.on_slider_move()
            )
            slider.set(init_val)
            slider.pack(fill="x", padx=10, pady=2)
            self.sliders[name] = slider

            # Telemetría (Pos, Vel, Effort)
            lbl = ttk.Label(frame, text="Pos: 0.000 | Vel: 0.000 | Effort: 0.000", font=("Monospace", 9))
            lbl.pack(padx=10, pady=2)
            self.labels_telemetry[name] = lbl

        self.update_telemetry()

    def on_slider_move(self):
        r1 = self.sliders["rot1_joint"].get()
        r2 = self.sliders["rot2_joint"].get()
        pr = self.sliders["pris_joint"].get()
        self.node.send_positions(r1, r2, pr)

    def update_telemetry(self):
        for name in self.labels_telemetry:
            if name in self.node.joint_data:
                pos, vel, eff = self.node.joint_data[name]
                self.labels_telemetry[name].config(
                    text=f"Pos: {pos:+.3f} rad/m | Vel: {vel:+.3f} | Effort: {eff:+.3f} N(m)"
                )
        self.root.after(100, self.update_telemetry)

def main():
    rclpy.init()
    node = GazeboJointGUI()

    # Ejecutar ROS 2 spin en un hilo secundario
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Interfaz Tkinter en el hilo principal
    root = tk.Tk()
    app = AppUI(root, node)
    root.mainloop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()