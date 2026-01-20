import cv2

cap = cv2.VideoCapture(0)

i = 0
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow("Press SPACE to capture", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 32:  # space
        cv2.imwrite(f"calibration_images/img_{i}.jpg", frame)
        print("Saved image", i)
        i += 1
    elif key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
