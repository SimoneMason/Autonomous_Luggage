import cv2
import numpy as np
import time

# Constants
SAMPLE_RATE = 200
SAMPLE_TIME = 1 / SAMPLE_RATE

# Initialize Camera
cap = cv2.VideoCapture(0)  # Change this to 0 if your primary camera is the one you want to use

# Function to detect and highlight the largest yellow object and display its size
def highlight_yellow_color(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the range for yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 2)  # Rectangle color is black

        # Display the size of the contour
        size_text = f"Width: {w} Height: {h}"
        cv2.putText(image, size_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return image, (x + w // 2, y + h // 2)

    return image, None

# Main loop
while True:
    start = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    highlighted_frame, center = highlight_yellow_color(frame)

    cv2.imshow('Yellow Highlight', highlighted_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Pause to maintain the sample rate
    time.sleep(max(0, SAMPLE_TIME - (time.time() - start)))

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
