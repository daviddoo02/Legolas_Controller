#!/usr/bin/env python3
  
import rospy
from ros_servo.msg import servo_reading

import matplotlib.pyplot as plt
import numpy as np

import time

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from micropython import const



class Servo_Calibrate:
    def __init__(self) -> None:

        # Setting up servos' pin numbers

        RHip1 = 5
        RHip2 = 2
        RThigh = 10
        RForeleg = 12
        RCalf = 14

        LHip1 = 6
        LHip2 = 3
        LThigh = 11
        LForeleg = 13
        LCalf = 15
        
        # Setting up i2c bus
        i2c = busio.I2C(SCL, SDA)

        # Setting up ADCs w proper addresses
        ads1 = ADS.ADS1115(i2c, address= const(0x4a))
        ads2 = ADS.ADS1115(i2c, address= const(0x4b))
        ads3 = ADS.ADS1115(i2c, address= const(0x49))

        self.RHip1_reading = AnalogIn(ads2, ADS.P0)
        self.RHip2_reading = AnalogIn(ads2, ADS.P1)
        self.RThigh_reading = AnalogIn(ads2, ADS.P2)
        self.RForeleg_reading = AnalogIn(ads2, ADS.P3)
        self.RCalf_reading = AnalogIn(ads3, ADS.P1)

        self.LHip1_reading = AnalogIn(ads1, ADS.P0)
        self.LHip2_reading = AnalogIn(ads1, ADS.P1)
        self.LThigh_reading = AnalogIn(ads1, ADS.P2)
        self.LForeleg_reading = AnalogIn(ads1, ADS.P3)
        self.LCalf_reading = AnalogIn(ads3, ADS.P0)

        # Setting up Servo Driver and Initialize Servo

        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.RHip1_servo = servo.Servo(self.pca.channels[RHip1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RHip2_servo = servo.Servo(self.pca.channels[RHip2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RThigh_servo = servo.Servo(self.pca.channels[RThigh], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RForeleg_servo = servo.Servo(self.pca.channels[RForeleg], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RCalf_servo = servo.Servo(self.pca.channels[RCalf], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.LHip1_servo = servo.Servo(self.pca.channels[LHip1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LHip2_servo = servo.Servo(self.pca.channels[LHip2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LThigh_servo = servo.Servo(self.pca.channels[LThigh], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LForeleg_servo = servo.Servo(self.pca.channels[LForeleg], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LCalf_servo = servo.Servo(self.pca.channels[LCalf], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.reset()

        self.calibrate()
    
    def reset(self):
        # # Standing 1 config

        # self.RHip1_servo.angle      = 15
        # self.LHip1_servo.angle      = 175

        # self.RHip2_servo.angle      = 80
        # self.LHip2_servo.angle      = 100

        # self.RThigh_servo.angle     = 110
        # self.LThigh_servo.angle     = 65

        # self.RForeleg_servo.angle   = 190
        # self.LForeleg_servo.angle   = 155

        # self.RCalf_servo.angle      = 90
        # self.LCalf_servo.angle      = 185

        # # Crouching 0 config

        # self.RHip1_servo.angle      = 5
        # self.LHip1_servo.angle      = 184

        # self.RHip2_servo.angle      = 100
        # self.LHip2_servo.angle      = 80

        # self.RThigh_servo.angle     = 110
        # self.LThigh_servo.angle     = 65

        # self.RForeleg_servo.angle   = 120
        # self.LForeleg_servo.angle   = 230

        # self.RCalf_servo.angle      = 85
        # self.LCalf_servo.angle      = 190

        # Crouching 0 config

        self.RHip1_servo.angle      = 5
        self.LHip1_servo.angle      = 184

        self.RHip2_servo.angle      = 100
        self.LHip2_servo.angle      = 80

        self.RThigh_servo.angle     = 110
        self.LThigh_servo.angle     = 65

        self.RForeleg_servo.angle   = 120
        self.LForeleg_servo.angle   = 230

        self.RCalf_servo.angle      = 85
        self.LCalf_servo.angle      = 190

        time.sleep(3)
        
        return

    def calibrate(self):

        # print("Calibrating Right Leg ...")

        # right_servos = [self.RHip1_servo, self.RHip2_servo, self.RThigh_servo, self.RForeleg_servo, self.RCalf_servo]
        # right_servos_reading = [self.RHip1_reading, self.RHip2_reading, self.RThigh_reading, self.RForeleg_reading, self.RCalf_reading]
        # right_ROM = [[0, 20, 15], [120, 70, 80], [70, 170, 110], [120, 230, 190], [45, 140, 90]]

        # for servo_id in range(5):
        #     # Go to min position
        #     right_servos[servo_id].angle = right_ROM[servo_id][0]
        #     time.sleep(3)
        #     print(right_servos_reading[servo_id].voltage)

        #     # Go to max position
        #     right_servos[servo_id].angle = right_ROM[servo_id][1]
        #     time.sleep(3)
        #     print(right_servos_reading[servo_id].voltage)

        #     # Go to 0 position
        #     right_servos[servo_id].angle = right_ROM[servo_id][2]
        #     time.sleep(3)
        #     print(right_servos_reading[servo_id].voltage)

        return
        
        

if __name__ == '__main__':
    try:
        ros_servo = Servo_Calibrate()
    except rospy.ROSInterruptException:
        pass
