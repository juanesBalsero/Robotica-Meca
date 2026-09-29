import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trayectoria')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        self.get_logger().info('Nodo Reproductor de Articulaciones Inicializado')

    def play_trajectory(self, joint_matrix, dt=0.1):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']

        current_time = 0.0

        for i, joints in enumerate(joint_matrix):
            t1, t2, t3 = joints

            current_time += dt

            pt = JointTrajectoryPoint()
            pt.positions = [t1, t2, t3]
            pt.time_from_start = Duration(
                sec=int(current_time),
                nanosec=int((current_time % 1) * 1e9)
            )
            msg.points.append(pt)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Trayectoria de {len(joint_matrix)} puntos enviada a Gazebo.\n'
            f'Duración total de ejecución: {current_time:.2f} s'
        )
        return current_time

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPlayer()
    
    DT = 0.04616  # Tiempo entre puntos en segundos

    MATRIZ_ARTICULACIONES = [
   	[-3.1135, 0.7496, -2.2145],  # Intervalo 1
    	[-3.0842, 0.7600, -2.2326],  # Intervalo 2
    	[-3.0539, 0.7703, -2.2500],  # Intervalo 3
    	[-3.0224, 0.7806, -2.2666],  # Intervalo 4
    	[-2.9897, 0.7908, -2.2825],  # Intervalo 5
    	[-2.9559, 0.8009, -2.2976],  # Intervalo 6
    	[-2.9210, 0.8111, -2.3118],  # Intervalo 7
    	[-2.8849, 0.8211, -2.3252],  # Intervalo 8
    	[-2.8479, 0.8312, -2.3376],  # Intervalo 9
    	[-2.8099, 0.8413, -2.3492],  # Intervalo 10
    	[-2.7709, 0.8514, -2.3598],  # Intervalo 11
    	[-2.7312, 0.8615, -2.3694],  # Intervalo 12
    	[-2.6908, 0.8715, -2.3780],  # Intervalo 13
    	[-2.6497, 0.8816, -2.3855],  # Intervalo 14
    	[-2.6082, 0.8916, -2.3920],  # Intervalo 15
    	[-2.5664, 0.9016, -2.3974],  # Intervalo 16
    	[-2.5243, 0.9115, -2.4017],  # Intervalo 17
    	[-2.4823, 0.9213, -2.4049],  # Intervalo 18
    	[-2.4403, 0.9310, -2.4069],  # Intervalo 19
    	[-2.3986, 0.9404, -2.4078],  # Intervalo 20
    	[-2.3573, 0.9496, -2.4076],  # Intervalo 21
    	[-2.3166, 0.9585, -2.4062],  # Intervalo 22
    	[-2.2765, 0.9671, -2.4037],  # Intervalo 23
    	[-2.2371, 0.9753, -2.4000],  # Intervalo 24
    	[-2.1986, 0.9830, -2.3953],  # Intervalo 25
    	[-2.1611, 0.9903, -2.3894],  # Intervalo 26
    	[-2.1245, 0.9970, -2.3825],  # Intervalo 27
    	[-2.0891, 1.0031, -2.3745],  # Intervalo 28
    	[-2.0547, 1.0087, -2.3655],  # Intervalo 29
    	[-2.0215, 1.0136, -2.3554],  # Intervalo 30
    	[-1.9894, 1.0179, -2.3444],  # Intervalo 31
    	[-1.9584, 1.0215, -2.3325],  # Intervalo 32
    	[-1.9286, 1.0244, -2.3197],  # Intervalo 33
    	[-1.8999, 1.0267, -2.3059],  # Intervalo 34
    	[-1.8724, 1.0282, -2.2913],  # Intervalo 35
    	[-1.8459, 1.0291, -2.2759],  # Intervalo 36
    	[-1.8205, 1.0293, -2.2597],  # Intervalo 37
    	[-1.7961, 1.0288, -2.2428],  # Intervalo 38
    	[-1.7727, 1.0276, -2.2251],  # Intervalo 39
    	[-1.7503, 1.0258, -2.2066],  # Intervalo 40
    	[-1.7288, 1.0233, -2.1875],  # Intervalo 41
    	[-1.7082, 1.0202, -2.1677],  # Intervalo 42
    	[-1.6884, 1.0165, -2.1473],  # Intervalo 43
    	[-1.6694, 1.0122, -2.1262],  # Intervalo 44
    	[-1.6513, 1.0073, -2.1045],  # Intervalo 45
    	[-1.6338, 1.0019, -2.0823],  # Intervalo 46
    	[-1.6171, 0.9959, -2.0594],  # Intervalo 47
    	[-1.6010, 0.9893, -2.0359],  # Intervalo 48
    	[-1.5856, 0.9822, -2.0119],  # Intervalo 49
    	[-1.5708, 0.9746, -1.9872],  # Intervalo 50  
    ]

    try:
        # Pausa para estabilizar la conexión del topic ROS 2
        rclpy.spin_once(node, timeout_sec=0.5)
        
        # LLAMADA A LA FUNCIÓN (Pasa la matriz y el tiempo DT)
        total_time = node.play_trajectory(MATRIZ_ARTICULACIONES, dt=DT)
        
        # Espera activa mientras el robot completa los 50 puntos en simulación
        if total_time > 0:
            rclpy.spin_once(node, timeout_sec=total_time + 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
