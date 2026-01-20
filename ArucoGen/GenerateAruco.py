import cv2
import cv2.aruco as aruco

# Define the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # 4x4 grid, 50 markers

# Generate a marker (ID 0) with size 200x200 pixels
marker_image = aruco.drawMarker(aruco_dict, id=9, sidePixels=200)

# Save to file
cv2.imwrite("aruco_marker_10.png", marker_image)

# Optional: display
cv2.imshow("Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()