#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header

class FakeArucoPublisher(Node):
    def __init__(self):
        super().__init__('fake_aruco_publisher')

        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate', 5.0)  # Hz

        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value

        self.publisher = self.create_publisher(PoseArray, '/aruco_poses', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_poses)

        self.get_logger().info(
            f"Publishing fake marker poses @ {rate} Hz in frame '{self.frame_id}'"
        )
        self.get_logger().info("Use these IDs / positions for simulation testing")

        self.markers = [
            {
                'id': 1,
                'position': [0.50, 0.70, 0.70],     # in camera frame: ~70 cm below camera (along optical axis)
                'orientation': [0.0, 0.0, 0.0, 1.0] # identity → marker Z points along camera Z (toward camera)
            },
        ]

    def publish_poses(self):
        self.get_logger().info("publish_poses callback triggered")   

        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.get_logger().info(f"Adding {len(self.markers)} hardcoded markers")  

        for marker in self.markers:
            pose = PoseStamped()
            pose.header = msg.header

            pose.pose.position.x = marker['position'][0]
            pose.pose.position.y = marker['position'][1]
            pose.pose.position.z = marker['position'][2]

            pose.pose.orientation.x = marker['orientation'][0]
            pose.pose.orientation.y = marker['orientation'][1]
            pose.pose.orientation.z = marker['orientation'][2]
            pose.pose.orientation.w = marker['orientation'][3]

            msg.poses.append(pose.pose)

            self.get_logger().info(
                f"  → Added ID {marker['id']}: "
                f"x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f} z={pose.pose.position.z:.3f}"
            )  # ← add this

        self.publisher.publish(msg)
        self.get_logger().info("Published PoseArray with " + str(len(msg.poses)) + " poses")   


def main(args=None):
    rclpy.init(args=args)
    node = FakeArucoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        exit(0)



if __name__ == '__main__':
    main()