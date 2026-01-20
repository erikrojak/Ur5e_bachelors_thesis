#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class WaveAndHomeNode(Node):
    def __init__(self):
        super().__init__('wave_and_home')
        
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Joint order for UR5e (never change this!)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Start the sequence after everything is ready
        self.create_timer(1.0, self.start_sequence)
        self.step = 0

    def send_trajectory(self, positions, duration_sec=3.0, log=""):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1)*1e9))
        
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(log)

    def start_sequence(self):
        if self.step == 0:
            # 1) Wave right
            self.send_trajectory(
                [ 0.6, -1.9,  1.9, -1.57, -1.57, 0.0],
                duration_sec=2.8,
                log="Waving → right"
            )
            self.step += 1
            self.create_timer(3.5, self.start_sequence)   # wait for motion + pause

        elif self.step == 2:
            # 3) Return home
            self.send_trajectory(
                [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                duration_sec=3.0,
                log="Returning to home position – done!"
            )
            self.step += 1
            # Finished → shut down cleanly after a few seconds
            self.create_timer(4.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = WaveAndHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()