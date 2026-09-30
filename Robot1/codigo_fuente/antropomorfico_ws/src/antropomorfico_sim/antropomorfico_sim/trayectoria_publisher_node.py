#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__('trayectoria_publisher_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint_dedoizq']
        self.timer = self.create_timer(0.5, self.publish_trajectory)
        self.trajectory_sent = False

    def publish_trajectory(self):
        if self.trajectory_sent:
            return

        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info('Esperando a que /arm_controller se conecte al tópico...')
            return

        self.get_logger().info('¡Controlador detectado! Procesando CSV de Octave...')
        self.trajectory_sent = True
        self.timer.cancel()

        pkg_share = get_package_share_directory('antropomorfico_sim')
        csv_path = os.path.join(pkg_share, 'config', 'trayectoria_robot.csv')

        if not os.path.exists(csv_path):
            self.get_logger().error(f'No se encontró el archivo CSV en: {csv_path}')
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        try:
            with open(csv_path, 'r') as file:
                reader = csv.reader(file)
                rows = [r for r in reader if r]

            if not rows:
                self.get_logger().error('El archivo CSV está vacío.')
                return

            NS_IN_SEC = 1_000_000_000
            # Margen inicial de 3s para que el robot viaje del cero de Gazebo a Home de forma suave
            OFFSET_INICIAL_NS = 3 * NS_IN_SEC
            last_ns = -1

            for row in rows:
                raw_t = float(row[0])
                q_values = list(map(float, row[1:5]))

                # Tiempo del CSV a nanosegundos + offset inicial
                target_ns = int(round(raw_t * NS_IN_SEC)) + OFFSET_INICIAL_NS

                # Corrección estricta para evitar tiempos duplicados
                if target_ns <= last_ns:
                    target_ns = last_ns + 1_000_000  # Fuerza +1 milisegundo

                last_ns = target_ns

                p = JointTrajectoryPoint()
                p.positions = q_values
                p.time_from_start = Duration(nanoseconds=target_ns).to_msg()
                msg.points.append(p)

            self.publisher_.publish(msg)
            
            t_final = last_ns / NS_IN_SEC
            self.get_logger().info(f'¡Trayectoria enviada con éxito! ({len(rows)} puntos, duración total: {t_final:.2f}s)')

        except Exception as e:
            self.get_logger().error(f'Error al procesar el CSV: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()