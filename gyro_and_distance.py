import RPi.GPIO as GPIO
from hcsr04sensor import sensor
import time

import smbus					#import SMBus module of I2C
from time import sleep          #import

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

trig1 = 17
echo1 = 27

trig2 = 23
echo2 = 24

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) 

GPIO.setup(trig1, GPIO.OUT)
GPIO.setup(trig2, GPIO.OUT)
GPIO.setup(echo1, GPIO.IN)
GPIO.setup(echo2, GPIO.IN)

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


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

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
	
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
	

    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 
    ## Left sensor
    GPIO.output(trig1, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(trig1, GPIO.LOW)
   

    while GPIO.input(echo1) == 0:
        pulse_start1 = time.time()

    while GPIO.input(echo1) == 1:
        pulse_end1 = time.time()
    
    pulse_duration1 =  pulse_end1 - pulse_start1
    distance1 = round((pulse_duration1 * 34300)/2, 2)
    print("Left Distance: ", distance1, "cm")

    ## Right sensor
    GPIO.output(trig2, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(trig2, GPIO.LOW)
   

    while GPIO.input(echo2) == 0:
        pulse_start2 = time.time()

    while GPIO.input(echo2) == 1:
        pulse_end2 = time.time()
    
    pulse_duration2 =  pulse_end2 - pulse_start2
    distance2 = round((pulse_duration2 * 34300)/2, 2)
    print("Right Distance: ", distance2, "cm")	


     
    pulse_start1 = None
    pulse_end1 = None
    pulse_start2 = None
    pulse_end2 = None

GPIO.cleanup()