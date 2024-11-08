import time
import math
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import time
import board
from adafruit_motorkit import MotorKit
import numpy as np
from scipy.signal import butter, lfilter

def clamp(n, min_value, max_value):
    return max(min_value, min(n, max_value))

# Create an MPU9250 instance
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,  # In case the MPU9250 is connected to another I2C device
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

mpu.abias = [0.0571044921875, 0.022967529296875, 0.0008789062500000888] # Set the master accelerometer biases
mpu.gbias = [-2.4566650390625, -0.07972717285156249, -0.07762908935546875] # Set the master gyroscope biases

# Configure the MPU9250
mpu.configure()
kit = MotorKit(i2c=board.I2C())
kS = 0.153
kP = .04
kD = 0.005

setpoint = 0.00
deadband = 2

last_error = 0
angle_estimate = 0
last_time = time.time()

while True:
    # Read the accelerometer, gyroscope, and magnetometer values
    accel_data = mpu.readAccelerometerMaster()
    gyro_data = mpu.readGyroscopeMaster()

    #new_gyro_reading = mpu.readAccelerometerMaster()[1]
    roll = math.atan2(accel_data[1] , accel_data[2]) * 57.3
    rate = (gyro_data[0] )#/ 32.8)

    dt = time.time() - last_time
    last_time = time.time()

    alpha = 0.98  # Complementary filter weight, between 0 and 1
    angle_estimate = alpha * (angle_estimate + rate * dt) + (1 - alpha) * roll


    error = angle_estimate - setpoint
    d_error = (error-last_error)*dt
    last_error = error

    if (abs(error) > deadband):
        if (error > 0):
            output = float(clamp(kP*error + kD*d_error + kS,-1,1))
        else: 
            output = float(clamp(kP*error + kD*d_error - kS,-1,1)) 
    else:
        output = 0
    

    #print([rate,rate*dt])
    print([error,output])
    kit.motor1.throttle = -1 * output
    kit.motor2.throttle = output

