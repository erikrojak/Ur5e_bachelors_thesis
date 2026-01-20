#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import time

class PickPlaceSucker(Node):
    def __init__(self):
        super().__init__('pick_place_sucker')
        self.attach_pub = self.create_publisher(AttachedCollisionObject, 
                                               '/attached_collision_object', 10)
        self.detach_pub = self.create_publisher(CollisionObject, 
                                               '/collision_object', 10)
        self.script_pub = self.create_publisher(String, 
                                               '/urscript_interface/script_command', 10)
        
        self.get_logger().info("Pick & Place with sucker READY")

    def send_script(self, func):
        msg = String()
        msg.data = func()
        self.script_pub.publish(msg)

    def suck_on(self):
        self.send_script(lambda: "set_tool_digital_out(0, True)\nsleep(0.5)")

    def suck_off(self):
        self.send_script(lambda: "set_tool_digital_out(0, False)\nsleep(0.3)")

    def attach_box(self, box_name="Red_part"):
        aco = AttachedCollisionObject()
        aco.object.header.frame_id = "tool0"        # or "ee_link" / "wrist_3_link"
        aco.object.id = box_name
        aco.link_name = "tool0"
        aco.touch_links = ["tool0", "wrist_3_link"]  # important for vacuum!
        
        # The actual box
        box = CollisionObject()
        box.header.frame_id = "tool0"
        box.id = box_name
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.06, 0.06, 0.06]
        box.primitives = [primitive]
        box.primitive_poses = [Pose()]  # position zero = centered on tool0
        
        aco.object = box
        aco.object.operation = CollisionObject.ADD
        self.attach_pub.publish(aco)
        self.get_logger().info(f"Attached {box_name} to tool0")

    def detach_box(self, box_name="Red_part", pose_in_world=None):
        # Remove from robot
        aco = AttachedCollisionObject()
        aco.object.id = box_name
        aco.object.operation = CollisionObject.REMOVE
        self.attach_pub.publish(aco)
        
        # Place back in world at current TCP pose (or specify)
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = box_name
        co.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.06, 0.06, 0.06]
        co.primitives = [primitive]
        co.primitive_poses = [pose_in_world or self.get_current_tcp_pose()]
        
        self.detach_pub.publish(co)
        self.get_logger().info(f"Detached {box_name} into world")

    def get_current_tcp_pose(self):
        # Simplified – in real code you would read /joint_states or /tf
        p = Pose()
        p.position.x = 0.55
        p.position.y = 0.35
        p.position.z = 0.95   # above the bin
        return p

def main():
    rclpy.init()
    node = PickPlaceSucker()
    time.sleep(2)

    # 1. Go above red part → plan & execute in RViz with manipulator group
    print("→ Plan & execute to pre-pick pose in RViz, then press Enter")
    input()

    # 2. Lower → pick → attach → lift
    node.send_script(lambda: "movel(pose_trans(get_actual_tcppose(), p[0,0,-0.10,0,0,0]), a=0.3, v=0.1)")
    time.sleep(4)
    node.suck_on()
    node.attach_box("Red_part")        # ← this is the magic line
    time.sleep(1)
    node.send_script(lambda: "movel(pose_trans(get_actual_tcppose(), p[0,0,0.15,0,0,0]), a=0.3, v=0.1)")
    time.sleep(4)

    # 3. Go to place → release → detach
    print("→ Plan & execute to pre-place pose in RViz, then press Enter")
    input()
    node.send_script(lambda: "movel(pose_trans(get_actual_tcppose(), p[0,0,-0.12,0,0,0]), a=0.3, v=0.1)")
    time.sleep(4)
    node.suck_off()
    node.detach_box("Red_part")
    time.sleep(1)

    node.get_logger().info("Pick & place with sucker simulation complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()