#!/usr/bin/env python3
import rospy
from legolas_biped.msg import joint_readings

import numpy as np

import time

from board import SCL, SDA
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn



class Encoders:
    def __init__(self) -> None:
        
        i2c = busio.I2C(SCL, SDA)
        ads = ADS.ADS1115(i2c)
        self.chan0 = AnalogIn(ads, ADS.P0)

        pub_topic = 'encoder_readings'
        self.servo_pub = rospy.Publisher(pub_topic, joint_readings, queue_size=1)
        
        self.servo_msg = joint_readings()

        self.read_joint_angles()

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

    def enc_2_deg(self, val_reading, V_reading, valmin = 2090, valmax = 25565, Vmin = 0.261, Vmax = 3.573):
        a = 0.75
        angle1 = self.map_range(V_reading, Vmin, Vmax, 0, 270)
        angle2 = self.map_range(val_reading, valmin, valmax, 0, 270)
        angle = angle1*a + angle2*(1-a)
        return angle

    def exponential_moving_average(data, alpha):
        ema = [data[0]]
        for i in range(1, len(data)):
            ema.append(alpha * data[i] + (1 - alpha) * ema[-1])
        return ema
    
    def ros_publish_readings(self, left_angles, right_angles):
        self.servo_msg.Left_Joint_Readings = left_angles
        self.servo_msg.Right_Joint_Readings = right_angles

        self.servo_pub.publish(self.servo_msg)
        return
    
    def read_joint_angles(self):
        return
        

if __name__ == '__main__':
    rospy.init_node('servo_encoders')
    
    encoders = Encoders()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rate.sleep()