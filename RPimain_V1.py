import RPi.GPIO as GPIO
from hcsr04sensor import sensor
import time
from time import sleep
import smbus					#import SMBus module of I2C
import random
import cv2
import numpy as np

# Initialise pins for motor and sensors
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Set PWM & Direction for motors 1 & 2 on MD10-hat
AN2 = 13				
AN1 = 12				
DIG2 = 24				
DIG1 = 26				

#set pins for Ultrasonic sensor 1 (left)
trig1 = 17
echo1 = 27

#set pins for Ultrasonic sensor 2 (Right)
trig2 = 23
echo2 = 25

#set pins for Ultrasonic sensor 3 (Straight) (NOTE: MAY NEED TO CHANGE PINS)
trig3 = 5
echo3 = 6

#Set warnings to off and set board to BCM
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) 

# Setting trig pins as outputs and echo pins as inputs
GPIO.setup(trig1, GPIO.OUT)
GPIO.setup(trig2, GPIO.OUT)
GPIO.setup(trig3, GPIO.OUT)
GPIO.setup(echo1, GPIO.IN)
GPIO.setup(echo2, GPIO.IN)
GPIO.setup(echo3, GPIO.IN)

#setting motor driver pins as outputs
GPIO.setup(AN2, GPIO.OUT)		
GPIO.setup(AN1, GPIO.OUT)		
GPIO.setup(DIG2, GPIO.OUT)		
GPIO.setup(DIG1, GPIO.OUT)	

p1 = GPIO.PWM(AN1, 100)			# set pwm for M1
p2 = GPIO.PWM(AN2, 100)			# set pwm for M2

#initialisation function for the MPU 6050
def MPU_Init():
	#Write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# Function to read data from the MPU
def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

# initialises variables required for function calc_angle
def init_angle():
    mid_point_X = 0
    angle = 0
    return mid_point_X, angle

# calculates the angle the luggage should move to follow person
def calc_angle(mid_point_X):
    
    # calculate angle 
    difference = mid_point_X - 325
    angle = difference / 3.75
    print("Angle: ", angle, "degrees")

    return angle

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
        return image, (x + w // 2, y + h // 2)

    return image, None

def colour_detection(cap):
    ret, frame = cap.read()
    if not ret:
        print("I did not get ret")
        return

    highlighted_frame, center = highlight_yellow_color(frame)

    cv2.imshow('Yellow Highlight', highlighted_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return

    return center

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

# initialise variables required for other functions
mid_point_X, angle = init_angle()

# Initialize Camera
cap = cv2.VideoCapture(0) 

#assign raw data to variables and process data
while True:
    start = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    highlighted_frame, center = highlight_yellow_color(frame)

    cv2.imshow('Yellow Highlight', highlighted_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.7)

    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    
    Gx = gyro_x/131.0 + 0.073
    Gy = gyro_y/131.0 - 0.02
    Gz = gyro_z/131.0 - 0.04
    
    #print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax,     "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 
    
    #Loop through until program ends or keyboard interupt is pressed (Ctrl+c)
    try:
        while True:
            start = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            highlighted_frame, center = highlight_yellow_color(frame)
            cv2.imshow('Yellow Highlight', highlighted_frame)

            # send out pulse on Ultrasonic sensor 1 (LEFT)
            GPIO.output(trig1, GPIO.HIGH)
            time.sleep(0.0001)
            GPIO.output(trig1, GPIO.LOW)

            # Measure time taken for pulse to return
            while GPIO.input(echo1) == 0:
                pulse_start1 = time.time()
            while GPIO.input(echo1) == 1:
                pulse_end1 = time.time()
            # Calculate distance
            pulse_duration1 =  pulse_end1 - pulse_start1
            left_dist = round((pulse_duration1 * 34300)/2, 2)
            #print("Left Distance: ", left_dist, "cm")

            # send out pulse on Ultrasonic sensor 2 (RIGHT)
            GPIO.output(trig2, GPIO.HIGH)
            time.sleep(0.0001)
            GPIO.output(trig2, GPIO.LOW)
        
            # Measure time taken for pulse to return
            while GPIO.input(echo2) == 0:
                pulse_start2 = time.time()
            while GPIO.input(echo2) == 1:
                pulse_end2 = time.time()
            
            # Calculated distance
            pulse_duration2 =  pulse_end2 - pulse_start2
            right_dist = round((pulse_duration2 * 34300)/2, 2)
            #print("Right Distance: ", right_dist, "cm")

            # send out pulse on Ultrasonic sensor 3 (STRAIGHT)
            GPIO.output(trig3, GPIO.HIGH)
            time.sleep(0.0001)
            GPIO.output(trig3, GPIO.LOW)
        
            # Measure time taken for pulse to return
            while GPIO.input(echo3) == 0:
                pulse_start3 = time.time()
            while GPIO.input(echo3) == 1:
                pulse_end3 = time.time()                
            
            # Calculated distance
            pulse_duration3 =  pulse_end3 - pulse_start3
            straight_dist = round((pulse_duration3 * 34300)/2, 2)
            #print("Straight Distance: ", straight_dist, "cm")	


            #If gyro variables are between -1 & 1, then..... otherwise luggage has fallen, end program
            if (Gx or Gy or Gz) < 1 and (Gx or Gy or Gz) > -1:

                # If the front ultrasonic sensor reading is smaller than 20cm
                if straight_dist < 20:
                    print("Front Distance less than 20cm")
                    # Luggage pauses
                    p1.start(0)                     
                    p2.start(0)

                    # break loop otherwise code gets stuck on motor settings
                    break         

                # If left distance sensor < 40 and right distance sensor >= 40
                elif left_dist < 40 and right_dist >= 40:
                    print("Turning Right")

                    #both motors drive forwards, right motor slows to turn right
                    GPIO.output(DIG1, GPIO.LOW)       
                    GPIO.output(DIG2, GPIO.LOW)    
                    p1.start(20)                     
                    p2.start(100)

                    # break loop otherwise code gets stuck on motor settings
                    break         

                # if right sensor < 40 cm then:          
                elif left_dist >= 40 and right_dist < 40:
                    print("Turning Left")

                    # both motors drive forwards, left motor slows to turn left
                    GPIO.output(DIG1, GPIO.LOW)		
                    GPIO.output(DIG2, GPIO.LOW)		
                    p1.start(100)			
                    p2.start(20)           
                    break	
                
                # left and right ultrasonic sensors have things close by, luggage goes straight
                elif left_dist < 40 and right_dist < 40:
                    print("Going Straight")
                    # both motors drive forwards
                    GPIO.output(DIG1, GPIO.LOW)		
                    GPIO.output(DIG2, GPIO.LOW)		
                    p1.start(100)			
                    p2.start(100)           
                    break	

                # path is clear, follow person
                else:
            
                    print("Follow person")
                    (mid_point_X, mid_point_Y) = colour_detection(cap)
                    print("mid_point_X: ", mid_point_X)
                    
                    angle = calc_angle(mid_point_X)  
                    if angle >= 0 :
                        p1.start(100-angle)
                        p2.start(100)
                    else:
                        p1.start(100)
                        p2.start(100-angle)    
                    print("I made it here")                   
                    break

            # luggage has fallen over
            else:
                print("I have fallen over")
                # stop motors
                p1.start(0)
                p2.start(0) 
                break     

    # If keyboard interrupt pressed:                        
    except:

        # Release the camera and close all OpenCV windows
        cap.release()
        cv2.destroyAllWindows()

        #set pulse timers to None, stop motors, cleanup GPIO pins and exit program
        pulse_start1 = None
        pulse_end1 = None
        pulse_start2 = None
        pulse_end2 = None
        p1.start(0)
        p2.start(0)
        GPIO.cleanup()


# avoid_objects()