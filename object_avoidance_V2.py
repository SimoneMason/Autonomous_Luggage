'''
V2 VERSION NOTES

This should be a working version of sensors and motors using the existing left and right ultrasonic sensors. Not yet tested on Pi.
'''


import RPi.GPIO as GPIO
from hcsr04sensor import sensor
import time
from time import sleep
import smbus					#import SMBus module of I2C
import random



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

#Set warnings to off and set board to BCM
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) 

# Setting trig pins as outputs and echo pins as inputs
GPIO.setup(trig1, GPIO.OUT)
GPIO.setup(trig2, GPIO.OUT)
GPIO.setup(echo1, GPIO.IN)
GPIO.setup(echo2, GPIO.IN)

#setting motor driver pins as outputs
GPIO.setup(AN2, GPIO.OUT)		
GPIO.setup(AN1, GPIO.OUT)		
GPIO.setup(DIG2, GPIO.OUT)		
GPIO.setup(DIG1, GPIO.OUT)	

p1 = GPIO.PWM(AN1, 100)			# set pwm for M1
p2 = GPIO.PWM(AN2, 100)			# set pwm for M2

#initialisation function for the MPU 6050
def MPU_Init():
	#write to sample rate register
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

# Placeholder function for qr_detection. Generates two random x and y location integers
def qr_detection():
    loc_x = random.randint(1,100)
    loc_y = random.randint(1,100)
    print("QR Location: (", loc_x, ",", loc_y, ")")
    return loc_x, loc_y


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

#assign raw data to variables and process data
while True:
    time.sleep(0.5)

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

    (prev_x, prev_y) = (0,0)
    
    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 
    
    #Loop through until program ends or keyboard interupt is pressed (Ctrl+c)
    try:
        while True:


            # send out pulse on Ultrasonic sensor 1 (LEFT)
            GPIO.output(trig1, GPIO.HIGH)
            time.sleep(0.0001)
            GPIO.output(trig1, GPIO.LOW)

            #Measure time taken for pulse to return
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
        
            #Measure time taken for pulse to return
            while GPIO.input(echo2) == 0:
                pulse_start2 = time.time()

            while GPIO.input(echo2) == 1:
                pulse_end2 = time.time()
            
            # Calculated distance
            pulse_duration2 =  pulse_end2 - pulse_start2
            right_dist = round((pulse_duration2 * 34300)/2, 2)
            #print("Right Distance: ", right_dist, "cm")	

            # check if user has moved
            loc_x, loc_y = qr_detection()
            if loc_x != prev_x and loc_y != prev_y:

                #If gyro variables are between -1 & 1, then..... otherwise luggage has fallen, end program
                if (Gx or Gy or Gz) < 1 and (Gx or Gy or Gz) > -1:

                    # If left distance sensor < 40 and right distance sensor >= 40
                    if left_dist < 40 and right_dist >= 40:
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
                        # INSERT CALCULATE ANGLE FUNCTION                       
                        break

                # luggage has fallen over
                else:
                    print("I have fallen over")
                    # stop motors and exit program
                    p1.start(0)
                    p2.start(0) 
                    break
            else:
                print("User has not moved")
                # stop motors and exit program
                p1.start(0)
                p2.start(0) 
                break        

    # If keyboard interrupt pressed:                        
    except:
        #set pulse timers to None, stop motors, cleanup GPIO pins and exit program
        pulse_start1 = None
        pulse_end1 = None
        pulse_start2 = None
        pulse_end2 = None
        p1.start(0)
        p2.start(0)
        GPIO.cleanup()

