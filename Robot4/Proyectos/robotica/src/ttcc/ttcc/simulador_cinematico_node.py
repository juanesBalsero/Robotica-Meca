import rclpy
from rclpy.node import Node
import numpy as np
import time
from sensor_msgs.msg import JointState

class SimuladorNodoROS(Node):
    def __init__(self):
        super().__init__('simulador_cinematico_node')
        # Publicador para enviar los estados de las articulaciones a RViz
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info('Nodo de simulacion cinematica para RViz iniciado correctamente.')

    def animar_por_articulaciones(self, q_inicial, q_final, joint_names, titulo='Animación'):
        pasos = 50
        trayectoria_articular = np.linspace(q_inicial, q_final, pasos)
        
        for q in trayectoria_articular:
            if not rclpy.ok():
                break
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = [float(val) for val in q]
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publicando [{titulo}]: {msg.position}')
            
            time.sleep(0.05)

    def ejecutar_simulacion(self):
        # 1. Definición de articulaciones según el URDF
        scara_joints = ['1', '2', '3', '4'] 
        q_scara_home = np.array([0.0, 0.0, 0.0, 0.0])
        q_scara_p1   = np.array([1.851, 0.943, -0.079, 2.089])

        antro_joints = ['11', '22', '33', '44', '55', '66']
        q_antro_home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_antro_p2   = np.array([-1.206, 0.288, 0.221, -0.187, -1.681, 1.851])

        # Bucle continuo con la secuencia ordenada
        while rclpy.ok():
            # 1. SCARA de 0 a P1
            self.get_logger().info('SCARA: Moviendo de 0 a P1...')
            self.animar_por_articulaciones(q_scara_home, q_scara_p1, scara_joints, titulo='SCARA a P1')
            time.sleep(0.5)

            # 2. Antropomórfico de 0 a P2
            self.get_logger().info('Antropomórfico: Moviendo de 0 a P2...')
            self.animar_por_articulaciones(q_antro_home, q_antro_p2, antro_joints, titulo='Antropomórfico a P2')
            time.sleep(0.5)

            # 3. SCARA regresa de P1 a 0
            self.get_logger().info('SCARA: Regresando de P1 a 0...')
            self.animar_por_articulaciones(q_scara_p1, q_scara_home, scara_joints, titulo='SCARA a 0')
            time.sleep(0.5)

            # 4. Antropomórfico regresa de P2 a 0
            self.get_logger().info('Antropomórfico: Regresando de P2 a 0...')
            self.animar_por_articulaciones(q_antro_p2, q_antro_home, antro_joints, titulo='Antropomórfico a 0')
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    nodo = SimuladorNodoROS()
    
    try:
        time.sleep(1.0)
        nodo.ejecutar_simulacion()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()