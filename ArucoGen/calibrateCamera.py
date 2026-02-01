#!/usr/bin/env python3

import cv2
import numpy as np
import glob

# ---------- SETTINGS ----------
CHESSBOARD_SIZE = (9, 6)             # inner corners
SQUARE_SIZE = 1.0                    # arbitrary unless you need scale
IMAGE_FOLDER = "calibration_images"  # where your calibration photos are stored
# ------------------------------

# Prepare object points for the chessboard
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0],
                       0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # scale in real world units

objpoints = []  # 3D points
imgpoints = []  # 2D points

images = glob.glob(f"{IMAGE_FOLDER}/*.jpg")

print(f"Found {len(images)} calibration images.")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        print(f"✔ Detected corners in {fname}")
        objpoints.append(objp)

        # refine corner locations for better accuracy
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER,
                      30, 0.001)
        )
        imgpoints.append(corners2)

        # draw and show
        cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
        cv2.imshow("Calibration", img)
        cv2.waitKey(200)
    else:
        print(f"✘ No corners detected in {fname}")

cv2.destroyAllWindows()

# ------------------------------
# CALIBRATION
# ------------------------------
print("\nRunning calibration...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if not ret:
    print("Calibration failed.")
    exit()

print("\n--- Calibration Results ---")
print("Camera Matrix:\n", camera_matrix)
print("\nDistortion Coefficients:\n", dist_coeffs)
print("------------------------------------")

# Save results
np.save("camera_matrix_charuco.npy", camera_matrix)
np.save("dist_coeffs_charuco.npy", dist_coeffs)

print("\nSaved:")
print(" -> camera_matrix.npy")
print(" -> dist_coeffs.npy")
print("\nCalibration complete.")
