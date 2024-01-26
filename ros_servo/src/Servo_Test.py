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



class Servo:
    def __init__(self) -> None:

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
        
        i2c = busio.I2C(SCL, SDA)

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

        self.servo_test()

    def map_range(self, x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        """
        Map a number from one range to another linearly and clamp the result.

        Parameters:
        - x (float): Input value to be mapped.
        - in_min (float): Minimum value of the input range.
        - in_max (float): Maximum value of the input range.
        - out_min (float): Minimum value of the output range.
        - out_max (float): Maximum value of the output range.

        Returns:
        - float: Mapped value within the specified output range.
        """
        # Calculate the mapped value, ensuring it's within the output range
        mapped = max(min((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_max), out_min)
        return mapped

    def get_voltage(self,pin):
        return (pin.value * 3.3) / 65536
    
    def enc_2_deg(self, pin, Vmin = 0, Vmax = 3.3):
        angle = self.map_range(pin.voltage, Vmin, Vmax, 0, 270)
        #angle = self.map_range(self.get_voltage(pin), Vmin, Vmax, 0, 270)
        return angle
    
    def servo_test(self):
        # Crouching 0 config
        
        self.RHip1_servo.angle      = 5
        self.RHip2_servo.angle      = 100
        self.RThigh_servo.angle     = 110
        self.RForeleg_servo.angle   = 120
        self.RCalf_servo.angle      = 85

        self.LHip1_servo.angle      = 184
        self.LHip2_servo.angle      = 80
        self.LThigh_servo.angle     = 65
        self.LForeleg_servo.angle   = 230
        self.LCalf_servo.angle      = 190

        time.sleep(5)

        # Standing 1 config

        self.RHip1_servo.angle    = 15
        self.RHip2_servo.angle    = 80
        self.RThigh_servo.angle   = 110
        self.RForeleg_servo.angle = 190
        self.RCalf_servo.angle    = 90

        self.LHip1_servo.angle    = 175
        self.LHip2_servo.angle    = 100
        self.LThigh_servo.angle   = 65
        self.LForeleg_servo.angle = 155
        self.LCalf_servo.angle    = 185

        time.sleep(5)

        # Crouching 0 config

        self.RHip1_servo.angle      = 5
        self.RHip2_servo.angle      = 100
        self.RThigh_servo.angle     = 110
        self.RForeleg_servo.angle   = 120
        self.RCalf_servo.angle      = 85

        self.LHip1_servo.angle      = 184
        self.LHip2_servo.angle      = 80
        self.LThigh_servo.angle     = 65
        self.LForeleg_servo.angle   = 230
        self.LCalf_servo.angle      = 190

        time.sleep(5)

        # print("Right Hip 1  :", self.enc_2_deg(self.RHip1_reading))
        # print("Right Hip 2  :", self.enc_2_deg(self.RHip2_reading))
        # print("Right Thigh  :", self.enc_2_deg(self.RThigh_reading))
        # print("Right Foreleg:", self.enc_2_deg(self.RForeleg_reading))
        # print("Right Calf   :", self.enc_2_deg(self.RCalf_reading))
        # print()
        # print("Left Hip 1   :", self.enc_2_deg(self.LHip1_reading))
        # print("Left Hip 2   :", self.enc_2_deg(self.LHip2_reading))
        # print("Left Thigh   :", self.enc_2_deg(self.LThigh_reading))
        # print("Left Foreleg :", self.enc_2_deg(self.LForeleg_reading))
        # print("Left Calf    :", self.enc_2_deg(self.LCalf_reading))
        return
        
        

if __name__ == '__main__':
    try:
        ros_servo = Servo()
    except rospy.ROSInterruptException:
        pass
