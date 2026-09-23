#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# ==============================================================================
# MODELO MATEMÁTICO (Traducción exacta de tus funciones MATLAB)
# ==============================================================================

def tqi(tht, d, alp, a):
    costh = math.cos(tht)
    sinth = math.sin(tht)
    cosalp = math.cos(alp)
    sinalp = math.sin(alp)
    
    return np.array([
        [costh, -sinth * cosalp,  sinth * sinalp, a * costh],
        [sinth,  costh * cosalp, -costh * sinalp, a * sinth],
        [0.0,    sinalp,         cosalp,        d],
        [0.0,    0.0,            0.0,           1.0]
    ])

def cinematicad(th1, th2, d3d):
    """ Cinemática Directa SCARA """
    tht1 = th1
    tht2 = th2 - math.pi / 2.0
    tht3 = 0.0

    # Parámetros geométricos
    d1 = 0.16 + 0.024  # 0.184 m
    d2 = 0.0
    d3 = d3d

    alp1, alp2, alp3 = 0.0, math.pi, 0.0
    a1, a2, a3 = 0.33, 0.223, 0.0

    A1 = tqi(tht1, d1, alp1, a1)
    A2 = tqi(tht2, d2, alp2, a2)
    A3 = tqi(tht3, d3, alp3, a3)

    T = A1 @ A2 @ A3

    Px = float(T[0, 3])
    Py = float(T[1, 3])
    Pz = float(T[2, 3])

    return T, Px, Py, Pz

def cinematicai(Px_mundo, Py_mundo, Pz_mundo, codo_arriba=True):
    """ Cinemática Inversa SCARA """
    a1 = 0.33
    a2 = 0.223
    d1 = 0.16 + 0.024  # 0.184 m

    Px = Px_mundo
    Py = Py_mundo
    Pz = Pz_mundo

    aph = math.atan2(Py, Px)
    d = math.sqrt(Px**2 + Py**2)

    # Protección de rango para acos [-1, 1]
    cos_B = (d**2 + a1**2 - a2**2) / (2.0 * d * a1)
    cos_B = max(-1.0, min(1.0, cos_B))
    B = math.acos(cos_B)

    cos_phi = (a1**2 + a2**2 - d**2) / (2.0 * a1 * a2)
    cos_phi = max(-1.0, min(1.0, cos_phi))
    phi = math.acos(cos_phi)

    if codo_arriba:
        tht1 = aph + B
        tht2 = (math.pi - phi) - math.pi / 2.0
    else:
        tht1 = aph - B
        tht2 = (-(math.pi - phi) - math.pi / 2.0) * (-1.0)

    # Desplazamiento d3 articular
    d3 = d1 - Pz

    return tht1, tht2, d3

# ==============================================================================
# NODO ROS 2 Y GENERADOR DE TRAYECTORIA
# ==============================================================================

class ScaraTrajectoryNode(Node):
    def __init__(self):
        super().__init__('scara_trajectory_node')
        
        # Publicador de comandos hacia ros2_control / Gazebo
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/scara_position_controller/commands', 
            10
        )
        
        # Suscriptor para leer estado real
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.current_joints = [0.0, 0.0, 0.0]
        self.has_joint_state = False

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 3:
            # Mapeo según el orden del controlador
            self.current_joints = [msg.position[0], msg.position[1], msg.position[2]]
            self.has_joint_state = True

    def execute_trajectory(self, target_P, total_time=16.0, steps=100):
        """ Divide la trayectoria cartesiana en subpuntos y la ejecuta """
        while not self.has_joint_state and rclpy.ok():
            self.get_logger().info("Esperando el primer reporte de /joint_states...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # Atualizar la posición del suscriptor para capturar la posición exacta de inicio
        rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Calcular posición actual con Cinemática Directa
        q1, q2, q3 = self.current_joints
        _, Px_init, Py_init, Pz_init = cinematicad(q1, q2, q3)
        start_P = [Px_init, Py_init, Pz_init]

        self.get_logger().info("=====================================================")
        self.get_logger().info(f"INICIANDO TRAYECTORIA CÁLCULO LINEAL")
        self.get_logger().info(f"Punto Inicial (Cartesiano): X={start_P[0]:.4f}, Y={start_P[1]:.4f}, Z={start_P[2]:.4f}")
        self.get_logger().info(f"Punto Destino (Cartesiano): X={target_P[0]:.4f}, Y={target_P[1]:.4f}, Z={target_P[2]:.4f}")
        self.get_logger().info(f"Tiempo Total: {total_time} s | Pasos: {steps}")
        self.get_logger().info("=====================================================")

        dt = total_time / steps

        for i in range(steps + 1):
            if not rclpy.ok():
                break

            # Mantiene actualizado /joint_states durante el bucle de ejecución
            rclpy.spin_once(self, timeout_sec=0.001)

            alpha = i / float(steps)  # Factor de interpolación 0.0 -> 1.0
            
            # Interpolación lineal en espacio cartesiano (X, Y, Z)
            px_step = start_P[0] + alpha * (target_P[0] - start_P[0])
            py_step = start_P[1] + alpha * (target_P[1] - start_P[1])
            pz_step = start_P[2] + alpha * (target_P[2] - start_P[2])

            try:
                # 2. Cinemática Inversa para el punto cartesiano interpolado
                th1_sol, th2_sol, d3_sol = cinematicai(px_step, py_step, pz_step)

                # 3. Enviar comando articular al robot
                msg = Float64MultiArray()
                msg.data = [float(th1_sol), float(th2_sol), float(d3_sol)]
                self.cmd_pub.publish(msg)

                self.get_logger().info(
                    f"Paso {i}/{steps} | Pos Cartesiana: [{px_step:.3f}, {py_step:.3f}, {pz_step:.3f}] "
                    f"-> Articular: [θ1={th1_sol:.3f}, θ2={th2_sol:.3f}, d3={d3_sol:.3f}]"
                )

            except Exception as e:
                self.get_logger().error(f"Punto inalcanzable fuera del espacio de trabajo: {e}")

            time.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrajectoryNode()

    # Definición de los dos puntos de destino [Px, Py, Pz]
    target_point_1 = [-0.1526, -0.0908, 0.094]
    target_point_2 = [0.3299, -0.223, 0.1847]

    # --- TRAYECTORIA 1 ---
    node.get_logger().info("Ejecutando Trayectoria al PUNTO 1...")
    node.execute_trajectory(target_P=target_point_1, total_time=16.0, steps=80)

    # Pausa opcional entre puntos para estabilizar el manipulador
    time.sleep(1.0)

    # --- TRAYECTORIA 2 ---
    node.get_logger().info("Ejecutando Trayectoria al PUNTO 2...")
    node.execute_trajectory(target_P=target_point_2, total_time=16.0, steps=80)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()