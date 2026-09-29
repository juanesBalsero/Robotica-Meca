import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )

        # Longitudes de eslabones (en metros, adaptadas de tus valores de MATLAB)
        self.L1 = 0.2000   # 20.0 cm
        self.L2 = 0.1506   # 15.06 cm
        self.L3 = 0.18464  # 18.464 cm

        self.get_logger().info('Nodo IK inicializado (Recorrido de Matriz de Puntos)')

    def cinematica_inversa_3dof(self, px, py, pz):
        """Calcula IK en modo Codo Arriba."""
        theta1_rad = math.atan2(py, px)

        num = px**2 + py**2 + (pz - self.L1)**2 - self.L2**2 - self.L3**2
        den = 2 * self.L2 * self.L3
        cos_theta3 = num / den

        if abs(cos_theta3) > 1.0:
            return None

        sin_theta3 = -math.sqrt(1 - cos_theta3**2)
        theta3_rad = math.atan2(sin_theta3, cos_theta3)

        alpha = math.atan2(pz - self.L1, math.sqrt(px**2 + py**2))
        beta = math.atan2(self.L3 * sin_theta3, self.L2 + self.L3 * cos_theta3)
        theta2_rad = alpha - beta

        return theta1_rad, theta2_rad, theta3_rad

    def move_matrix_trajectory(self, point_matrix, dt=0.1):
        """
        Recorre la matriz de puntos enviando la trayectoria al controlador.
        - point_matrix: Lista/matriz con puntos (x, y, z) en metros.
        - dt: Tiempo (en segundos) que le toma al robot pasar de un punto al siguiente.
        """
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']

        self.get_logger().info(
            f'Procesando matriz de {len(point_matrix)} puntos con DT = {dt} s por punto...'
        )

        current_time = 0.0
        valid_points = 0

        for i, point in enumerate(point_matrix):
            px, py, pz = point
            angles = self.cinematica_inversa_3dof(px, py, pz)

            if angles is None:
                self.get_logger().warn(
                    f'Punto #{i+1} [{px:.3f}, {py:.3f}, {pz:.3f}] m fuera del espacio de trabajo. Saltando...'
                )
                continue

            t1, t2, t3 = angles
            current_time += dt  # Incrementar el tiempo para el siguiente punto

            pt_msg = JointTrajectoryPoint()
            pt_msg.positions = [t1, t2, t3]
            pt_msg.time_from_start = Duration(
                sec=int(current_time),
                nanosec=int((current_time % 1) * 1e9)
            )
            msg.points.append(pt_msg)
            valid_points += 1

        if valid_points > 0:
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Trayectoria publicada exitosamente ({valid_points}/{len(point_matrix)} puntos válidos).\n'
                f'Tiempo total estimado de ejecución: {current_time:.2f} s.'
            )
            return current_time
        else:
            self.get_logger().error('Ningún punto de la matriz fue válido. No se envió trayectoria.')
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()

    # Tiempo constante entre cada punto consecutivo (en segundos).
    DT = 0.04616

    MATRIZ_PUNTOS = [
    	(-0.132350, 0.000000, 0.118000),
    	(-0.129703, -0.003647, 0.119000),
    	(-0.127056, -0.007294, 0.120000),
    	(-0.124409, -0.010941, 0.121000),
    	(-0.121762, -0.014588, 0.122000),
    	(-0.119115, -0.018235, 0.123000),
    	(-0.116468, -0.021882, 0.124000),
    	(-0.113821, -0.025529, 0.125000),
    	(-0.111174, -0.029176, 0.126000),
    	(-0.108527, -0.032823, 0.127000),
    	(-0.105880, -0.036470, 0.128000),
    	(-0.103233, -0.040117, 0.129000),
    	(-0.100586, -0.043764, 0.130000),
    	(-0.097939, -0.047411, 0.131000),
    	(-0.095292, -0.051058, 0.132000),
    	(-0.092645, -0.054705, 0.133000),
    	(-0.089998, -0.058352, 0.134000),
    	(-0.087351, -0.061999, 0.135000),
    	(-0.084704, -0.065646, 0.136000),
    	(-0.082057, -0.069293, 0.137000),
    	(-0.079410, -0.072940, 0.138000),
    	(-0.076763, -0.076587, 0.139000),
    	(-0.074116, -0.080234, 0.140000),
    	(-0.071469, -0.083881, 0.141000),
    	(-0.068822, -0.087528, 0.142000),
    	(-0.066175, -0.091175, 0.143000),
    	(-0.063528, -0.094822, 0.144000),
    	(-0.060881, -0.098469, 0.145000),
    	(-0.058234, -0.102116, 0.146000),
    	(-0.055587, -0.105763, 0.147000),
    	(-0.052940, -0.109410, 0.148000),
    	(-0.050293, -0.113057, 0.149000),
    	(-0.047646, -0.116704, 0.150000),
    	(-0.044999, -0.120351, 0.151000),
    	(-0.042352, -0.123998, 0.152000),
    	(-0.039705, -0.127645, 0.153000),
    	(-0.037058, -0.131292, 0.154000),
    	(-0.034411, -0.134939, 0.155000),
    	(-0.031764, -0.138586, 0.156000),
    	(-0.029117, -0.142233, 0.157000),
    	(-0.026470, -0.145880, 0.158000),
    	(-0.023823, -0.149527, 0.159000),
    	(-0.021176, -0.153174, 0.160000),
    	(-0.018529, -0.156821, 0.161000),
    	(-0.015882, -0.160468, 0.162000),
    	(-0.013235, -0.164115, 0.163000),
    	(-0.010588, -0.167762, 0.164000),
    	(-0.007941, -0.171409, 0.165000),
    	(-0.005294, -0.175056, 0.166000),
    	(-0.002647, -0.178703, 0.167000),
    	(0.000000, -0.182350, 0.168000),
     ]

    # =========================================================================

    try:
        # Enviar la trayectoria completa
        total_time = node.move_matrix_trajectory(MATRIZ_PUNTOS, dt=DT)
        
        # Esperar a que el robot termine la trayectoria completa antes de cerrar
        if total_time > 0:
            rclpy.spin_once(node, timeout_sec=total_time + 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
