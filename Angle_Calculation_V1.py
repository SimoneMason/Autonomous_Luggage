import cv2
import imutils
import numpy as np 
import time

protopath = "model/object_detection/MobileNetSSD_deploy.prototxt"
modelpath = "model/object_detection/MobileNetSSD_deploy.caffemodel"
detector = cv2.dnn.readNetFromCaffe(protopath, modelpath)
detector.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
detector.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)


CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

person_idx = CLASSES.index("person")

cap = cv2.VideoCapture(0)

skip_frames = 1
frame_count = 0
prev_time = time.time()
(startX, startY, endX, endY) = (0,0,0,0)
angle = 0

# calculates the angle the luggage should move to follow person
def calc_angle(prev_time, startX, endX, angle):
    # calculate a new angle every 1 second 
    curr_time = time.time() 
    if curr_time > (prev_time + 0.1):

        mid_point_X = endX - startX
        difference = mid_point_X - 300

        # person to the left of luggage
        if difference < 0:
            angle = difference / 3.75

        # person to the right of luggage    
        else:
            angle = - difference / 3.75    
        print("Angle: ", angle, "degrees")
        prev_time = curr_time
        return (prev_time, angle)
    return (prev_time, angle)

while True:
    ret, frame = cap.read()
    frame_count += 1
    
    if frame_count % skip_frames == 0:
        frame = imutils.resize(frame, width=600)

        (H, W) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (W, H), 127.5)
        detector.setInput(blob)
        person_detections = detector.forward()

        for i in np.arange(0, person_detections.shape[2]):
            confidence = person_detections[0, 0, i, 2]
            if confidence > 0.7:
                idx = int(person_detections[0, 0, i, 1])

                if idx != person_idx:
                    continue

                person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = person_box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

        cv2.imshow("Application", frame)

    (prev_time, angle) = calc_angle(prev_time, startX, endX, angle)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break

cv2.destroyAllWindows()
