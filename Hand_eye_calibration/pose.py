#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException, Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

class EEPoseGetter(Node):
    def __init__(self):
        super().__init__('ee_pose_getter')
        
        self.ee_link = "tool0"          # Change if your TCP is named differently (e.g. "grasp_frame", "tcp")
        self.target_frame = "base_link" # Usually this one for robot base frame
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Waiting for TF transform: {self.target_frame} → {self.ee_link}")
        self.get_logger().info("Jog the robot or wait until driver publishes TF...")

    def get_current_pose(self) -> PoseStamped | None:
        try:
            # Lookup the latest transform (timeout 5 seconds)
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.ee_link,
                rclpy.time.Time(),          # Latest available
                timeout=Duration(seconds=5.0)
            )
            
            pose = PoseStamped()
            pose.header = trans.header
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return None

    def run_once(self):
        pose = self.get_current_pose()
        if pose:
            p = pose.pose.position
            q = pose.pose.orientation
            self.get_logger().info(
                f"Current EE pose in '{self.target_frame}':\n"
                f"  Position: x={p.x:.4f}  y={p.y:.4f}  z={p.z:.4f}\n"
                f"  Orientation (quat): x={q.x:.4f}  y={q.y:.4f}  z={q.z:.4f}  w={q.w:.4f}"
            )
        else:
            self.get_logger().warn("No valid pose available yet – check TF tree")

def main(args=None):
    rclpy.init(args=args)
    node = EEPoseGetter()
    
    try:
        while rclpy.ok():
            node.run_once()
            rclpy.spin_once(node, timeout_sec=1.0)  # Check every second (adjust as needed)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    main()