import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# QR Code detector
detector = cv2.QRCodeDetector()

while cap.isOpened():
    start = time.perf_counter()
    success, img = cap.read()
    if not success:
        break

    value, points, _ = detector.detectAndDecode(img)

    if points is not None:
        points = points[0]  # Get the first set of points

        x1, y1 = int(points[0][0]), int(points[0][1])
        x2, y2 = int(points[2][0]), int(points[2][1])

        x_center, y_center = (x1 + x2) // 2, (y1 + y2) // 2
        radius = 10  # Define your desired radius

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 225, 0), thickness=2)
        cv2.circle(img, (x_center, y_center), radius, (0, 225, 0), thickness=2)

        if value:
            cv2.putText(img, value, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    end = time.perf_counter()
    totalTime = end - start
    fps = 1 / totalTime

    cv2.putText(img, f'FPS: {int(fps)}', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('img', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
