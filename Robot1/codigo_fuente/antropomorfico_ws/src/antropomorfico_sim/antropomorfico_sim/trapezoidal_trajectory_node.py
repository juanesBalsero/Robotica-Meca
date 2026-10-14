#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrapezoidalTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trapezoidal_trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # Espera 1 segundo para asegurar la conexión del publisher con Gazebo
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.get_logger().info('Nodo de Trayectoria Trapezoidal Inicializado')

    def eval_lspb(self, t, q_i, q_f, t_f, t_c, qc_dot, qc_ddot):
        """Calcula q(t), q_dot(t) y q_ddot(t) según la trayectoria trapezoidal de Siciliano."""
        if t <= t_c:
            # Tramo 1: Aceleración
            q = q_i + 0.5 * qc_ddot * (t ** 2)
            v = qc_ddot * t
            a = qc_ddot
        elif t <= (t_f - t_c):
            # Tramo 2: Velocidad de crucero
            q = q_i + qc_ddot * t_c * (t - t_c / 2.0)
            v = qc_dot
            a = 0.0
        else:
            # Tramo 3: Desaceleración
            q = q_f - 0.5 * qc_ddot * ((t_f - t) ** 2)
            v = qc_ddot * (t_f - t)
            a = -qc_ddot
        return q, v, a

    def publish_trajectory(self):
        self.timer.cancel()  # Ejecutar solo una vez

        msg = JointTrajectory()
        # Se incluyen las 3 articulaciones principales y la garra para coincidir con arm_controller
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint_dedoizq']

        # Parámetros calculados en MATLAB para cada articulación
        joints_params = [
            # Joint 1: qi = 1.57, qf = -1.57, tf = 16.0, tc = 0.30, qc_dot = -0.20, qc_ddot = -0.6667
            {'qi': 1.57, 'qf': -1.57, 'tf': 16.0, 'tc': 0.3000, 'qc_dot': -0.20, 'qc_ddot': -0.6667},
            # Joint 2: qi = 1.00, qf = 0.57,  tf = 16.0, tc = 5.25, qc_dot = -0.04, qc_ddot = -0.0076
            {'qi': 1.00, 'qf': 0.57,  'tf': 16.0, 'tc': 5.2500, 'qc_dot': -0.04, 'qc_ddot': -0.0076},
            # Joint 3: qi = -2.08, qf = -1.71, tf = 16.0, tc = 6.75, qc_dot = 0.04, qc_ddot = 0.0059
            {'qi': -2.08, 'qf': -1.71, 'tf': 16.0, 'tc': 6.7500, 'qc_dot': 0.04, 'qc_ddot': 0.0059}
        ]

        t_f = 16.0
        dt = 0.05  # Muestreo cada 50ms (20 Hz)
        steps = int(t_f / dt)

        for i in range(steps + 1):
            t = min(i * dt, t_f)
            point = JointTrajectoryPoint()
            
            positions = []
            velocities = []
            accelerations = []

            # Evaluar el estado kinemático para theta1, theta2 y theta3
            for p in joints_params:
                q, v, a = self.eval_lspb(t, p['qi'], p['qf'], p['tf'], p['tc'], p['qc_dot'], p['qc_ddot'])
                positions.append(q)
                velocities.append(v)
                accelerations.append(a)

            # Garra en reposo (joint_dedoizq = 0.0)
            positions.append(0.0)
            velocities.append(0.0)
            accelerations.append(0.0)

            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations

            # Asignación del tiempo incremental del punto
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            msg.points.append(point)

        self.get_logger().info('Publicando trayectoria trapezoidal suave (321 puntos)...')
        self.publisher_.publish(msg)
        self.get_logger().info('¡Trayectoria enviada correctamente a Gazebo!')

def main(args=None):
    rclpy.init(args=args)
    node = TrapezoidalTrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
