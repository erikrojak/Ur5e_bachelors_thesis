#!/usr/bin/env python3

import cv2
import numpy as np
import glob

# ---------- SETTINGS ----------
IMAGE_FOLDER = "calibration_images"

# ChArUco board parameters
SQUARES_X = 5          # number of chessboard squares (X)
SQUARES_Y = 7          # number of chessboard squares (Y)
SQUARE_LENGTH = 0.049    # set real size if you want metric units
MARKER_LENGTH = 0.028    # typically 0.65–0.8 * SQUARE_LENGTH

ARUCO_DICT = cv2.aruco.DICT_4X4_50
# ------------------------------

# Create dictionary and ChArUco board
aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
board = cv2.aruco.CharucoBoard_create(
    squaresX=SQUARES_X,
    squaresY=SQUARES_Y,
    squareLength=SQUARE_LENGTH,
    markerLength=MARKER_LENGTH,
    dictionary=aruco_dict
)

# Detector parameters
aruco_params = cv2.aruco.DetectorParameters_create()

# Storage for detected corners
all_charuco_corners = []
all_charuco_ids = []

images = glob.glob(f"{IMAGE_FOLDER}/*.jpg")
print(f"Found {len(images)} calibration images.")

img_size = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape[::-1]

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=aruco_params
    )

    if ids is None or len(ids) < 4:
        print(f"✘ Not enough markers in {fname}")
        continue

    # Interpolate ChArUco corners
    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )

    if retval >= 4:
        print(f"✔ Detected {retval} ChArUco corners in {fname}")
        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)

        # Visualization
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
        cv2.imshow("ChArUco Calibration", img)
        cv2.waitKey(200)
    else:
        print(f"✘ Not enough ChArUco corners in {fname}")

cv2.destroyAllWindows()

# ------------------------------
# CALIBRATION
# ------------------------------
print("\nRunning ChArUco calibration...")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    charucoCorners=all_charuco_corners,
    charucoIds=all_charuco_ids,
    board=board,
    imageSize=img_size,
    cameraMatrix=None,
    distCoeffs=None
)

if not ret:
    print("Calibration failed.")
    exit(1)

print("\n--- Calibration Results ---")
print("Camera Matrix:\n", camera_matrix)
print("\nDistortion Coefficients:\n", dist_coeffs)
print("------------------------------------")

# Save results
np.save("camera_matrix_charuco.npy", camera_matrix)
np.save("dist_coeffs_charuco.npy", dist_coeffs)

print("\nSaved:")
print(" -> camera_matrix_charuco.npy")
print(" -> dist_coeffs_charuco.npy")
print("\nCalibration complete.")
