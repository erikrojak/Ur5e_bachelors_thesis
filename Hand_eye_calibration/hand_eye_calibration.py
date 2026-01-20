#!/usr/bin/env python3
"""
Hand-eye calibration for marker-on-flange (eye-to-hand) using OpenCV.

Run this script and move the robot to many poses, pressing SPACE for each pose
when the marker is visible. The script records:
 - robot flange pose in base frame (you must implement get_robot_flange_pose())
 - marker pose in camera frame (from ArUco rvec/tvec)

After collecting N samples, it runs cv2.calibrateHandEye and prints the result.

You must implement get_robot_flange_pose() to return (R, t)
where R is 3x3 numpy rotation matrix, t is 3x1 translation (meters).
"""

import cv2
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np

class FlangePoseReader(Node):
    def __init__(self):
        super().__init__('flange_pose_reader')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_robot_flange_pose(self,
                              base_frame='base_link',
                              flange_frame='tool0'):
        """
        Returns:
          R (3x3 numpy rotation matrix)
          t (3x1 numpy translation vector, meters)
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                flange_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            raise RuntimeError(f"TF lookup failed: {e}")

        # Translation
        t = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ], dtype=float)

        # Quaternion
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w

        # Quaternion → Rotation matrix (manual, no scipy)
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ], dtype=float)

        return R, t.reshape(3, 1)



# -----------------------
# CONFIG
# -----------------------
CAM_INDEX = 0
MARKER_SIZE_M = 0.04
MIN_SAMPLES = 12

# ArUco setup (OpenCV 4.6 style)
aruco = cv2.aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters_create()

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera index {}".format(CAM_INDEX))

rclpy.init()
flange_reader = FlangePoseReader()

def get_robot_flange_pose():
    return flange_reader.get_robot_flange_pose(
        base_frame='base_link',
        flange_frame='tool0'
    )


# -----------------------
# Collection loop
# -----------------------
R_gripper2base = []  # rotation matrices (gripper in base)
t_gripper2base = []  # translations (gripper in base)
R_target2cam = []    # rotation matrices (target/marker in camera)
t_target2cam = []    # translations (marker in camera)

print("Press SPACE to record a sample when marker is visible. ESC to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    cameraMatrix=np.load("camera_matrix_charuco.npy")
    distCoeffs=np.load("dist_coeffs_charuco.npy")

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        # pose estimation for first detected marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_M, cameraMatrix, distCoeffs)
        # Note: we will save rvec/tvec using real calibration. For now assume no distortion.
        # draw for visual feedback (works without explicit camera params only for visualization)
        for rvec, tvec in zip(rvecs, tvecs):
            try:
                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.03)  # visual only
            except Exception:
                pass

    cv2.imshow("Calib - Press SPACE to capture sample", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        print("Exiting collection loop.")
        break
    if key == 32:  # SPACE pressed -> capture sample
        if ids is None:
            print("No marker visible - sample ignored.")
            continue

        # 1) read robot flange pose from your robot
        try:
            Rb, tb = get_robot_flange_pose()
        except Exception as e:
            print("Error acquiring robot pose:", e)
            continue

        # 2) get marker pose from camera for first marker (we assume rvecs/tvecs defined)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]  # in meters

        # Convert rvec -> rotation matrix
        R_marker_cam, _ = cv2.Rodrigues(rvec)  # rotation of marker in camera coords
        t_marker_cam = tvec.reshape(3, 1)

        # NOTE on shapes: OpenCV expects lists of rotations/translations in shape Nx3 and Nx1
        R_gripper2base.append(Rb)          # gripper (flange) in base
        t_gripper2base.append(tb.reshape(3, 1))
        R_target2cam.append(R_marker_cam)  # target (marker) in camera
        t_target2cam.append(t_marker_cam)

        print(f"Captured sample #{len(R_gripper2base)}")
        if len(R_gripper2base) >= MIN_SAMPLES:
            print("Enough samples collected — press 'c' to calibrate, or keep collecting.")
    if key == ord('c'):
        print("Calibration requested.")
        break

cap.release()
cv2.destroyAllWindows()

# Check sample count
n = len(R_gripper2base)
if n < 3:
    raise RuntimeError("Not enough samples for calibration: got {}".format(n))

# Convert lists into arrays for OpenCV
R_gripper2base_cv = [R.astype(np.float64) for R in R_gripper2base]
t_gripper2base_cv = [t.reshape(3).astype(np.float64) for t in t_gripper2base]
R_target2cam_cv = [R.astype(np.float64) for R in R_target2cam]
t_target2cam_cv = [t.reshape(3).astype(np.float64) for t in t_target2cam]

# -----------------------
# Run OpenCV hand-eye calibration
# -----------------------
# Methods: cv2.CALIB_HAND_EYE_TSAI, PARK, HORAUD, etc.
retR, retT = cv2.calibrateHandEye(
    R_gripper2base_cv, t_gripper2base_cv,
    R_target2cam_cv, t_target2cam_cv,
    method=cv2.CALIB_HAND_EYE_TSAI
)
# retR: 3x3 rotation matrix, retT: 3x1 translation vector
print("Calibration complete.")
print("Hand->Eye rotation:\n", retR)
print("Hand->Eye translation (m):\n", retT.reshape(3))

# Build homogeneous transform T_flange_camera (hand->eye)
T_flange_camera = np.eye(4)
T_flange_camera[:3, :3] = retR
T_flange_camera[:3, 3] = retT.reshape(3)

print("\nT_flange_camera (4x4):\n", T_flange_camera)
# You can compute camera in base for any sample:
# T_base_camera = T_base_flange * T_flange_camera

# -----------------------
# Verification: compute residuals
# -----------------------
def homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T

errors = []
for i in range(n):
    T_b_f = homog(R_gripper2base[i], t_gripper2base[i].reshape(3))
    T_cam_marker = homog(R_target2cam[i], t_target2cam[i].reshape(3))

    # predicted T_cam_marker from model:
    # T_cam_marker_pred = (T_flange_camera)^{-1} * (T_b_f)^{-1} * T_b_f * T_flange_camera ???  
    # Simpler verification: transform marker to base using computed transforms and compare consistency.
    # We compute T_base_camera = T_b_f @ T_flange_camera
    T_base_camera = T_b_f @ T_flange_camera

    # transform the marker pose (in camera) into base:
    T_camera_marker = T_cam_marker
    T_base_marker_pred = T_base_camera @ T_camera_marker

    # Now compute marker pose relative to flange measured from robot via known geometry:
    # If marker is rigidly attached to flange with unknown transform, we can't directly compare to a ground truth marker in base.
    # But we can test consistency by reprojecting between pairs; compute transform from marker observed in camera to marker predicted via other sample
    errors.append(0)  # placeholder — full residual check uses A X = X B residuals

print("Done. (You should now publish T_base_camera as static transform or save it.)")
