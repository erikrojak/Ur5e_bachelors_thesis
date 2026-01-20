#!/usr/bin/env python3
"""
ArUco detection + optional pose estimation compatible with OpenCV 4.6.0.

Usage:
  - If you HAVE camera calibration, create two numpy files:
      camera_matrix.npy
      dist_coeffs.npy
    and set `USE_CALIB = True` below.

  - If you DON'T have calibration, set `USE_CALIB = False`.
    Pose estimation will be skipped but detection/IDs will still work.
"""

import cv2
import numpy as np
import sys

# -----------------------
# Config
# -----------------------
CAM_INDEX = 0                       # try 0,1,2... for USB camera device
USE_V4L2 = True                     # Linux: use cv2.CAP_V4L2
FRAME_W = 1280
FRAME_H = 720
FPS = 30

# Set True if you have camera calibration files in same folder
USE_CALIB = True
CALIB_CAMERA_MATRIX_FILE = "calibration_files/camera_matrix_charuco.npy"
CALIB_DIST_COEFFS_FILE = "calibration_files/dist_coeffs_charuco.npy"
# Marker side length in meters (only used if USE_CALIB=True)
MARKER_SIZE_M = 0.04  # e.g. 4 cm

# -----------------------
# Open camera
# -----------------------
backend = cv2.CAP_V4L2 if USE_V4L2 else 0
cap = cv2.VideoCapture(CAM_INDEX, backend) if backend != 0 else cv2.VideoCapture(CAM_INDEX)

if not cap.isOpened():
    print("ERROR: Cannot open camera index", CAM_INDEX)
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_FPS, FPS)
print("Camera opened (index {}). Resolution {}x{} @ {}FPS".format(CAM_INDEX, FRAME_W, FRAME_H, FPS))

# -----------------------
# Load calibration if requested
# -----------------------
camera_matrix = None
dist_coeffs = None
if USE_CALIB:
    try:
        camera_matrix = np.load(CALIB_CAMERA_MATRIX_FILE)
        dist_coeffs = np.load(CALIB_DIST_COEFFS_FILE)
        print("Loaded camera calibration:", CALIB_CAMERA_MATRIX_FILE, CALIB_DIST_COEFFS_FILE)
    except Exception as e:
        print("WARNING: Failed to load calibration files:", e)
        print("Disabling pose estimation.")
        USE_CALIB = False

# -----------------------
# ArUco setup (OpenCV 4.6 style)
# -----------------------
aruco = cv2.aruco
# choose dictionary: change if you printed a different one
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector_params = aruco.DetectorParameters_create()

# -----------------------
# Main loop
# -----------------------
print("Starting detection loop. Press ESC to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("WARNING: Frame not read from camera")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detectMarkers returns corners, ids, rejectedCandidates
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

    if ids is not None and len(ids) > 0:
        # draw detected markers on the color frame
        aruco.drawDetectedMarkers(frame, corners, ids)
        

        # print ids
        ids_list = ids.flatten().tolist()
        print("Detected marker IDs:", ids_list)

        # if calibration available -> estimate pose for each marker
        if USE_CALIB:
            # rvecs and tvecs are returned as arrays shaped (N,1,3)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_M, camera_matrix, dist_coeffs)

            for i, marker_id in enumerate(ids_list):
                rvec = rvecs[i][0]   # rotation vector
                tvec = tvecs[i][0]   # translation vector (meters)

                # Draw axis for each marker (length 0.03 m)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                # Print pose (in camera frame)
                print(f" ID {marker_id} -> t (m): [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}] "
                      f" rvec (rad): [{rvec[0]:.3f}, {rvec[1]:.3f}, {rvec[2]:.3f}]")
    else:
        # optional: show rejected candidates for debugging
        # aruco.drawDetectedMarkers(frame, rejected, borderColor=(100,0,255))
        pass

    cv2.imshow("ArUco (OpenCV 4.6)", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
