import cv2
import numpy as np
import time

def init_colour_detect():
    # Constants
    SAMPLE_RATE = 200
    SAMPLE_TIME = 1 / SAMPLE_RATE

    # Initialize Camera
    cap = cv2.VideoCapture(0)  # Change this to 0 if your primary camera is the one you want to use
    return SAMPLE_RATE, SAMPLE_TIME, cap

# initialises variables required for function calc_angle
def init_angle():
    prev_time = time.time()
    mid_point_X = 0
    angle = 0
    return prev_time, mid_point_X, angle

# calculates the angle the luggage should move to follow person
def calc_angle(prev_time, mid_point_X, angle):
    # calculate a new angle every 1 second 
    curr_time = time.time() 
    #print("curr_time: ", curr_time, "prev_time: ", prev_time)
    if curr_time > (prev_time + 0.05):

        #(mid_point_X,) = center
        difference = mid_point_X - 325

        # person to the left of luggage
        angle = difference / 3.75

        print("Angle: ", angle, "degrees")
        prev_time = curr_time
        return (prev_time, angle)
    return prev_time, angle

# Function to detect and highlight the largest yellow object
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
        #print("x: ", x, ",y: ", y, ",w: ", w, ",h: ", h)
        return image, (x + w // 2, y + h // 2)

    return image, None

# initialisation for functions used
SAMPLE_RATE, SAMPLE_TIME, cap = init_colour_detect()
prev_time, mid_point_X, angle = init_angle()

# Main loop
while True:

    ret, frame = cap.read()
    if not ret:
        break

    highlighted_frame, center = highlight_yellow_color(frame)
    #print("Center: ", center)
    if center is not None:
        (mid_point_X, mid_point_Y) = center
        prev_time, angle = calc_angle(prev_time, mid_point_X, angle)

    cv2.imshow('Yellow Highlight', highlighted_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Pause to maintain the sample rate
    time.sleep(max(0, SAMPLE_TIME - (time.time() - prev_time)))

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
