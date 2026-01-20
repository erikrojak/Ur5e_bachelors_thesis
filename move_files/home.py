#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class HomeMover(Node):
    def __init__(self):
        super().__init__('home_mover')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        self.timer = self.create_timer(1.0, self.send_home)

    def send_home(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]   # classic home
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 4                        # 4 seconds to reach

        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info('Sent home trajectory via scaled_joint_trajectory_controller')
        self.timer.cancel()

def main():
    rclpy.init()
    node = HomeMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()