#!/usr/bin/env python3
  
import rospy
import typing
from ros_servo.msg import servo_reading

import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import matplotlib.pyplot as plt

class Servo:
    def __init__(self) -> None:
        
        i2c = busio.I2C(SCL, SDA)

        ads = ADS.ADS1115(i2c)

        self.chan0 = AnalogIn(ads, ADS.P0)

        self.pca = PCA9685(i2c)

        self.pca.frequency = 50

        self.servo = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.servo.angle = 0

        pub_topic = 'servo_reading'

        self.servo_pub = rospy.Publisher(pub_topic, servo_reading, queue_size=1)
        
        rospy.init_node('ros_servo')

        rate = rospy.Rate(1)

        self.servo_msg = servo_reading()

        while not rospy.is_shutdown():
            self.constant_value_test()
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

    def enc_2_deg(self, V_reading, Vmin = 0.262, Vmax = 3.053):
        angle = self.map_range(V_reading, Vmin, Vmax, 0, 270)
        return angle

    def constant_value_test(self):
        desiredPos = float(input("Degrees (0-270): "))
        self.servo.angle = desiredPos

        print("{:>5}\t{:>5}\t{:>5}".format('raw', 'v', 'deg'))
        n = 0

        while n < 20:
            value = self.chan0.value
            voltage = self.chan0.voltage
            angle = self.enc_2_deg(self.chan0.voltage)

            self.servo_msg.encoder_value = value
            self.servo_msg.encoder_voltage = voltage
            self.servo_msg.servo_angle = angle
            self.servo_pub.publish(self.servo_msg)


            print("{:>5}\t{:>5.3f}\t{:>5.3f}".format(value, voltage, angle))
            n += 1

if __name__ == '__main__':
    try:
        ros_servo = Servo()
    except rospy.ROSInterruptException:
        pass