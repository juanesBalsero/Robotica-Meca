#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import math

BASE_FRAME = 'base_link'
EE_FRAME = 'pris'   # cambia si tienes un link/frame de efector final distinto

class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF no disponible aún: {e}', throttle_duration_sec=2.0)
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'Efector -> x: {pose.pose.position.x:.4f}  '
            f'y: {pose.pose.position.y:.4f}  z: {pose.pose.position.z:.4f}',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()