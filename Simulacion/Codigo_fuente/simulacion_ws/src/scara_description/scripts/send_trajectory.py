#!/usr/bin/env python3
import csv
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectorySender(Node):
    def __init__(self, csv_path):
        super().__init__('scara_trajectory_sender')
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.csv_path = csv_path

    def run(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('No aparece el controlador')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']

        with open(self.csv_path, newline='') as f:
            rows = list(csv.DictReader(f))

        for row in rows:
            p = JointTrajectoryPoint()

            p.positions = [
                float(row['q1_rad']),
                float(row['q2_rad']),
                float(row['q3_m'])
            ]

            # Solo control por posicion.
            # Las velocidades se registran despues desde Gazebo.

            t = float(row['Tiempo_s'])

            # Evitamos empezar exactamente en t=0
            t = t + 0.5

            sec = int(t)
            nsec = int((t - sec) * 1e9)

            p.time_from_start.sec = sec
            p.time_from_start.nanosec = nsec

            goal.trajectory.points.append(p)

        self.get_logger().info(
            f'Enviando {len(goal.trajectory.points)} puntos'
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error('Trayectoria rechazada')
            return

        self.get_logger().info('Trayectoria ACEPTADA')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        self.get_logger().info(
            f'Trayectoria terminada. status={result.status}'
        )


def main():
    rclpy.init()

    csv_path = os.path.abspath(sys.argv[1])

    node = TrajectorySender(csv_path)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
