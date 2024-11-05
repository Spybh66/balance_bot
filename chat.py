import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import time
import board
from adafruit_motorkit import MotorKit
import numpy as np
from scipy.signal import butter, lfilter
from pidcontroller import PIDController

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

# Configure the MPU9250
mpu.configure()
kit = MotorKit(i2c=board.I2C())
min_power = 0.153
p = .45
setpoint = 0.022
deadband = 0.02

KP = 0.48  # Proportional gain
KI = 0.0   # Integral gain
KD = 0.00   # Derivative gain

# Variables for PID
error_sum = 0
last_error = 0
target_angle = setpoint  # target angle to maintain balance

def pid_controller(current_angle, dt):
    global error_sum, last_error
    error = target_angle - current_angle
    error_sum += error * dt
    d_error = (error - last_error) / dt if dt > 0 else 0
    last_error = error
    
    # PID output for motor control
    pid_output = KP * error + KI * error_sum + KD * d_error
    return pid_output

def balance_robot():
    last_time = time.time()
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Get sensor data
        new_gyro_reading = mpu.readAccelerometerMaster()[1]
        filtered_reading = update_gyro_buffer(new_gyro_reading)
        
        # Calculate the estimated current angle (simple integration of gyro)
        #current_angle = accel_angle + gyro_rate * dt
        
        # Get control output from PID controller
        motor_speed = pid_controller(-filtered_reading, dt)
        
        if (abs(motor_speed) > deadband):
            if (motor_speed > 0):
                output = float(clamp(motor_speed + min_power,-1,1))
            else: 
                output = float(clamp(motor_speed - min_power,-1,1)) 
        else:
            output = 0
        
        print([filtered_reading,motor_speed,output])
        kit.motor1.throttle = -1 * output
        kit.motor2.throttle = output
        time.sleep(0.01)  

try:
    balance_robot()
except KeyboardInterrupt:
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0