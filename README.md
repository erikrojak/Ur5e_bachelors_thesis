UR5e bachelor thesis

Repository structure:

ArucoGen -> program files for Aruco image generation, cakibration, and detection
  - GenerateAruco.py -> generates Aruco marker images
  - capture_image.py -> captures images for camera calibration
  - detectAruco.py -> detects Aruco markers in video stream, estimate pose if calibrated
  - calibrateCamera_charuco.py/calibrateCamera.py -> calibration files for charuco or classic checkerboard respectively.
  - calibration_files -> files with calibration inputs from calibrateCamera_charuco/calibrateCamera
  - calibration_images -> images for calibration, made with capture_image.py

Hand_eye_calibration -> program files for creation of camera_link, transformation between base_link. 
  - marker_pose.py -> position of markers relative to tool when touching.
  - marker_rcev_to_quat.pt -> helper file for transformation of camera perceived markers rvecs -> quats.
  - pose.py -> used for recording postion of robot
  - Transform.py -> uses collected data from poses of markers relative to camera and robot and transforms them. result is camera_link.
