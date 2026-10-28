#!/usr/bin/env python3
import csv
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointLogger(Node):
    def __init__(self):
        super().__init__('scara_joint_logger')
        out = os.path.expanduser('~/scara_joint_states.csv')
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_s','q1','q2','q3','dq1','dq2','dq3','tau1','tau2','F3'])
        self.t0 = time.time()
        self.create_subscription(JointState, '/joint_states', self.cb, 50)
        self.get_logger().info(f'Guardando en {out}. Ctrl+C para terminar.')

    def cb(self, msg):
        d = {name:i for i,name in enumerate(msg.name)}
        names = ['joint_1','joint_2','joint_3']
        if not all(n in d for n in names):
            return
        q=[]; dq=[]; ef=[]
        for n in names:
            i=d[n]
            q.append(msg.position[i] if i < len(msg.position) else float('nan'))
            dq.append(msg.velocity[i] if i < len(msg.velocity) else float('nan'))
            ef.append(msg.effort[i] if i < len(msg.effort) else float('nan'))
        self.w.writerow([time.time()-self.t0, *q, *dq, *ef])
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = JointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
