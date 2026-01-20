#!/usr/bin/env python3
# vision_pick_executor.py – works with ROS 2 Jazzy + MoveIt2 + ur_robot_driver (2025)

import geometry_msgs
import moveit_msgs
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject, AttachedCollisionObject
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from vision_msgs.msg import Detection3DArray
import time

class VisionPickExecutor(Node):
    def __init__(self):
        super().__init__('vision_pick_executor')

        # 1. MoveIt2 action clients (official ROS 2 way)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_client    = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # 2. URScript sender (you already use this)
        self.script_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)

        # 3. Subscribe to grasp candidates (from your fake_vision → grasp_planner)
        self.sub = self.create_subscription(
            PoseStamped, '/grasp_pose', self.execute_pick, 10)   # we will publish simple PoseStamped

        self.get_logger().info("VisionPickExecutor READY – waiting for /grasp_pose")

    # ------------------------------------------------------------------
    def send_script(self, script: str):
        msg = String()
        msg.data = script
        self.script_pub.publish(msg)

    def suck_on(self):
        self.send_script("set_tool_digital_out(0, True)\nsleep(0.6)")

    def suck_off(self):
        self.send_script("set_tool_digital_out(0, False)\nsleep(0.3)")

    # ------------------------------------------------------------------
    def execute_pick(self, grasp_pose_msg: PoseStamped):
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            return

        # 1. Go to pre-grasp (10 cm above)
        pre_grasp = PoseStamped()
        pre_grasp.header = grasp_pose_msg.header
        pre_grasp.pose = grasp_pose_msg.pose
        pre_grasp.pose.position.z += 0.10

        self.get_logger().info("Moving to pre-grasp")
        self.move_to_pose(pre_grasp)

        # 2. Lower to part
        self.send_script("movel(pose_trans(get_actual_tcppose(), p[0,0,-0.105,0,0,0]), a=0.3, v=0.08)")
        time.sleep(4.0)

        # 3. Suck + attach object
        self.suck_on()
        self.attach_object("Red_part")
        time.sleep(0.5)

        # 4. Lift
        self.send_script("movel(pose_trans(get_actual_tcppose(), p[0,0,0.20,0,0,0]), a=0.5, v=0.2)")
        time.sleep(4.0)

        # 5. Go to place (home)
        self.move_to_named("home")

        # 6. Release
        self.send_script("movel(pose_trans(get_actual_tcppose(), p[0,0,-0.10,0,0,0]), a=0.3, v=0.08)")
        time.sleep(3.0)
        self.suck_off()
        self.detach_object("Red_part")

        self.get_logger().info("Pick & place cycle complete!")

    # ------------------------------------------------------------------
    def move_to_pose(self, pose_stamped: PoseStamped):
        goal = MoveGroup.Goal()
        goal.request.group_name = "manipulator"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.planning_options.plan_only = False
        goal.request.goal_constraints.append(
            moveit_msgs.msg.Constraints(
                position_constraints=[moveit_msgs.msg.PositionConstraint(
                    header=pose_stamped.header,
                    link_name="tool0",
                    target_point_offset=geometry_msgs.msg.Vector3(),
                    constraint_region=moveit_msgs.msg.BoundingVolume(
                        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])],
                        primitive_poses=[pose_stamped.pose]
                    )
                )],
                orientation_constraints=[moveit_msgs.msg.OrientationConstraint(
                    header=pose_stamped.header,
                    link_name="tool0",
                    orientation=pose_stamped.pose.orientation,
                    absolute_x_axis_tolerance=0.1,
                    absolute_y_axis_tolerance=0.1,
                    absolute_z_axis_tolerance=3.14
                )]
            )
        )
        self.move_group_client.send_goal_async(goal).add_done_callback(self.response_callback)

    def move_to_named(self, name: str):
        goal = MoveGroup.Goal()
        goal.request.group_name = "manipulator"
        self.move_group_client.send_goal_async(goal)

    def response_callback(self, future):
        result = future.result()
    # ------------------------------------------------------------------
    def attach_object(self, name: str):
        aco = AttachedCollisionObject()
        aco.object.header.frame_id = "tool0"
        aco.object.id = name
        aco.link_name = "tool0"
        aco.touch_links = ["tool0", "wrist_3_link"]
        aco.object.operation = aco.object.ADD
        self.get_logger().info(f"Attached {name}")
        # You can publish to /attached_collision_object if you want visual feedback

    def detach_object(self, name: str):
        # Simple version: just remove from planning scene
        co = CollisionObject()
        co.id = name
        co.operation = co.REMOVE
        self.get_logger().info(f"Detached {name}")

def main():
    rclpy.init()
    node = VisionPickExecutor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()