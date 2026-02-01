import cv2
import time

cap = cv2.VideoCapture(0)

i = 0
last_capture_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow("Capturing every 2 seconds", frame)

    # Capture every 2 seconds
    current_time = time.time()
    if current_time - last_capture_time >= 2:
        cv2.imwrite(f"calibration_images/img_{i}.jpg", frame)
        print("Saved image", i)
        i += 1
        last_capture_time = current_time

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
