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

# Function to create a Butterworth low-pass filter
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Parameters
cutoff_frequency = 10.0  # cutoff frequency in Hz
sampling_rate = 100.0    # sampling rate in Hz
filter_order = 4        # order of the filter
buffer_size = 10       # size of the data buffer

# Initialize filter coefficients
b, a = butter_lowpass(cutoff_frequency, sampling_rate, filter_order)

# Initialize buffer to store the gyro data
gyro_buffer = np.zeros(buffer_size)

def update_gyro_buffer(new_reading):
    """
    Updates the gyro buffer with a new reading, applying the Butterworth filter
    in real-time.
    """
    # Shift the buffer to make room for the new reading at the end
    global gyro_buffer
    gyro_buffer = np.roll(gyro_buffer, -1)
    gyro_buffer[-1] = new_reading
    
    # Apply the filter to the current buffer
    filtered_data = lfilter(b, a, gyro_buffer)
    
    # Return the latest filtered value
    return filtered_data[-1]

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
min_power = 0.153
p = .008
setpoint = 0.00
deadband = 1

while True:
    # Read the accelerometer, gyroscope, and magnetometer values
    accel_data = mpu.readAccelerometerMaster()
    gyro_data = mpu.readGyroscopeMaster()
    mag_data = mpu.readMagnetometerMaster()

    # Print the sensor values
    #print("Accelerometer:", accel_data)
    #print(accel_data[1])
    #print("Gyroscope:", gyro_data)
    #print("Magnetometer:", mag_data)

    # Wait for 1 second before the next reading
    #time.sleep(.01)

    #new_gyro_reading = mpu.readAccelerometerMaster()[1]
    roll = math.atan2(accel_data[1] , accel_data[2]) * 57.3
    
    # Get the filtered reading
    filtered_reading = update_gyro_buffer(roll)

    error = filtered_reading - setpoint

    #print(error)
    if (abs(error) > deadband):
        if (error > 0):
            output = float(clamp(p*error + min_power,-1,1))
        else: 
            output = float(clamp(p*error - min_power,-1,1)) 
    else:
        output = 0
    
    
    #print(roll)
    print([filtered_reading,error,output])
    kit.motor1.throttle = -1 * output
    kit.motor2.throttle = output

    time.sleep(0.0001)

