# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for using adafruit_motorkit with a DC motor"""
import time
import board
from adafruit_motorkit import MotorKit

kit = MotorKit(i2c=board.I2C())

kit.motor1.throttle = -0.153
kit.motor2.throttle = 0.153
time.sleep(0.5)
kit.motor1.throttle = 0
kit.motor2.throttle = 0
