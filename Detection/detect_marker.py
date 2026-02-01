#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header

CAM_INDEX       = 0
USE_V4L2        = True
FRAME_W         = 1280
FRAME_H         = 720
FPS             = 30

USE_CALIB       = True
CALIB_CAMERA_MATRIX_FILE = "../ArucoGen/calibration_files/camera_matrix_charuco2.npy"
CALIB_DIST_COEFFS_FILE   = "../ArucoGen/calibration_files/dist_coeffs_charuco2.npy"
MARKER_SIZE_M   = 0.054  

CAMERA_FRAME    = "camera_link" 


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # ROS 2 publisher
        self.pub_poses = self.create_publisher(PoseArray, '/aruco_poses', 10)

        # Timer for processing
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.use_calibration = USE_CALIB

        # Camera
        backend = cv2.CAP_V4L2 if USE_V4L2 else 0
        self.cap = cv2.VideoCapture(CAM_INDEX, backend) if backend != 0 else cv2.VideoCapture(CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {CAM_INDEX}")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_FPS,          FPS)
        self.get_logger().info(f"Camera opened {FRAME_W}x{FRAME_H} @ {FPS} FPS")

        # Load camera calibration
        self.camera_matrix = None
        self.dist_coeffs   = None
        if self.use_calibration:
            try:
                self.camera_matrix = np.load(CALIB_CAMERA_MATRIX_FILE)
                self.dist_coeffs   = np.load(CALIB_DIST_COEFFS_FILE)
                self.get_logger().info("Calibration loaded")
            except Exception as e:
                self.get_logger().warn(f"Calibration load failed: {e} → pose estimation disabled")
                self.use_calibration = False

        # ArUco
        self.aruco_dict      = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters_create()   
        self.publish_counter = 0
        self.publish_every_n = 450

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame not read")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = CAMERA_FRAME

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            ids_list = ids.flatten().tolist()
            self.get_logger().info(f"Detected marker IDs: {ids_list}")

            if self.use_calibration:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE_M, self.camera_matrix, self.dist_coeffs
                )

                for i, marker_id in enumerate(ids_list):
                    tvec = tvecs[i][0]      
                    rvec = rvecs[i][0]      
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                    # Conversion of rotation vector to quaternion
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    from scipy.spatial.transform import Rotation as R
                    quat = R.from_matrix(rot_mat).as_quat()  

                    # PoseStamped message for the marker
                    pose = PoseStamped()
                    pose.header = pose_array.header
                    pose.pose.position.x = -float(tvec[0])
                    pose.pose.position.y = float(tvec[1])
                    pose.pose.position.z = float(tvec[2])
                    pose.pose.orientation.x = float(quat[0])
                    pose.pose.orientation.y = float(quat[1])
                    pose.pose.orientation.z = float(quat[2])
                    pose.pose.orientation.w = float(quat[3])

                    pose_array.poses.append(pose.pose)

                    self.get_logger().debug(f"Marker ID {marker_id} pose published")

                    # Optional: individual topic per marker
                    # if marker_id not in self.per_marker_pubs:
                    #     self.per_marker_pubs[marker_id] = self.create_publisher(PoseStamped, f'/aruco/marker_{marker_id}', 10)
                    # self.per_marker_pubs[marker_id].publish(pose)

                    self.get_logger().info(
                        f"ID {marker_id} | pos [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}] m | "
                        f"quat [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]"
                    )

        self.pub_poses.publish(pose_array)

        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            self.get_logger().info("ESC pressed → shutting down")
            raise KeyboardInterrupt

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()