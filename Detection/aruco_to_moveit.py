#!/usr/bin/env python3

from build.ur_dashboard_msgs.ament_cmake_python.ur_dashboard_msgs.ur_dashboard_msgs import msg
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlannerInterfaceDescription, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation as R


class ArucoToMoveIt(Node):
    def __init__(self):
        super().__init__('aruco_to_moveit')

        # Transform listener for TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client to /move_action provided by move_group node
        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # Waiting for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Movegroup action server not available")
            raise RuntimeError("Cannot connect to Movegroup action server")

        self.get_logger().info("Connected to /move_action")

        # Subscriber to ArUco poses
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_callback,
            10
        )
        self.get_logger().info("Waiting for /aruco_poses")

    def aruco_callback(self, msg: PoseArray):
        if not msg.poses:
            return
        #First marker only(later implement by ID)
        target_pose_cam = PoseStamped()
        target_pose_cam.header = msg.header
        target_pose_cam.pose = msg.poses[0]

        try:
            # Transform to base_link
            target_pose_base = self.tf_buffer.transform(
                target_pose_cam,
                "base_link",
                timeout=Duration(seconds=2.0)
            )

            # Create hover pose (8 cm above because of MODBUS length)
            hover_pose = PoseStamped()
            hover_pose.header = target_pose_base.header
            hover_pose.pose = target_pose_base.pose
            hover_pose.pose.position.z += 0.08 
            hover_pose.pose.position.y -= 0.08
            hover_pose.pose.position.x = 0.00

            self.get_logger().info(
                f"Planning to hover above marker: "
                f"x={hover_pose.pose.position.x:.3f}  "
                f"y={hover_pose.pose.position.y:.3f}  "
                f"z={hover_pose.pose.position.z:.3f}\n"
                f"xq={hover_pose.pose.orientation.x:.3f}  "
                f"yq={hover_pose.pose.orientation.y:.3f}  "
                f"zq={hover_pose.pose.orientation.z:.3f}  "
                f"wq={hover_pose.pose.orientation.w:.3f}"
            )

            # Prepare MoveGroup goal
            goal = MoveGroup.Goal()
            goal.request.group_name = "ur_manipulator"
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.4
            goal.request.max_acceleration_scaling_factor = 0.4
            goal.request.planner_id = "OMPL"   # or PILZ, RRTConnectkConfigDefault

            # Pose goal
            goal.request.goal_constraints.append(
                self.create_pose_constraint(hover_pose)
            )

            # Send goal asynchronously
            
            send_future = self.action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)

            if not send_future.done() or not send_future.result().accepted:
                self.get_logger().warn("MoveIt hover goal rejected")
                return

            goal_handle = send_future.result()
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

            result = result_future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Hover reached → sending robot home")
                self.home()
                rclpy.sleep(6.0)
            else:
                self.get_logger().warn(f"Hover failed — code {result.error_code.val}")

        except Exception as e:
            self.get_logger().error(f"Error preparing goal: {e}")

    def create_pose_constraint(self, pose_stamped: PoseStamped):
        #Constrait for end-effector to reach pose_stamped
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = "tool0"              
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE))
        pc.constraint_region.primitives[0].dimensions = [0.01]  # 1 cm tolerance
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = "tool0"
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.3   # more tolerance around Z
        oc.weight = 1.0

        constr = Constraints()
        constr.position_constraints.append(pc)
        constr.orientation_constraints.append(oc)

        return constr

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted → waiting for result")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Movement executed successfully")
        else:
            self.get_logger().warn(f"Move failed – error code: {result.error_code.val}")

    def home(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(seconds=4, nanoseconds=0).to_msg()

        msg.points = [point]
        self.home_pub.publish(msg)
        self.get_logger().info("Sent home trajectory (4s) via scaled controller")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToMoveIt()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()