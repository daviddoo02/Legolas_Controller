#!/usr/bin/env python3
import rospy
from legolas_biped.msg import joint_readings
import numpy as np
import time
from board import SCL, SDA
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import argparse

class Pots:
    def __init__(self, left) -> None:
        self.left = left

        if self.left:
            rospy.init_node('left_leg_pots')
            pub_topic = 'left_joint_readings'
        else:
            rospy.init_node('right_leg_pots')
            pub_topic = 'right_joint_readings'

        i2c = busio.I2C(SCL, SDA)

        ads1 = ADS.ADS1115(i2c, address=0x4a)
        self.Lhip1 = AnalogIn(ads1, ADS.P0)
        self.Lhip2 = AnalogIn(ads1, ADS.P1)
        self.Lthigh = AnalogIn(ads1, ADS.P2)
        self.Lforeleg = AnalogIn(ads1, ADS.P3)

        ads2 = ADS.ADS1115(i2c, address=0x4b)
        self.Rhip1 = AnalogIn(ads2, ADS.P0)
        self.Rhip2 = AnalogIn(ads2, ADS.P1)
        self.Rthigh = AnalogIn(ads2, ADS.P2)
        self.Rforeleg = AnalogIn(ads2, ADS.P3)

        ads3 = ADS.ADS1115(i2c, address=0x49)
        self.Lcalf = AnalogIn(ads3, ADS.P0)
        self.Rcalf = AnalogIn(ads3, ADS.P1)

        self.servo_pub = rospy.Publisher(pub_topic, joint_readings, queue_size=1)
        
        self.servo_msg = joint_readings()

        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            self.read_joint_angles()
            rate.sleep()

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
    
    def read_joint_angles(self):
        if self.left:
            L_angles = [self.Lhip1.value, self.Lhip2.value, self.Lthigh.value, self.Lforeleg.value, self.Lcalf.value]
            self.servo_msg.Joint_Readings = L_angles
            self.servo_pub.publish(self.servo_msg)
        else:
            R_angles = [self.Rhip1.value, self.Rhip2.value, self.Rthigh.value, self.Rforeleg.value, self.Rcalf.value]        
            self.servo_msg.Joint_Readings = R_angles
            self.servo_pub.publish(self.servo_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Potentiometers')
    parser.add_argument('--right', action="store_false", help='enable Right Leg')
    args, unknown = parser.parse_known_args()

    Left_leg = args.right
    encoders = Pots(Left_leg)