#!/usr/bin/env python3
  
import rospy
from ros_servo.msg import servo_reading

import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import numpy as np

import typing
import time

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn



class Servo:
    def __init__(self) -> None:
        
        i2c = busio.I2C(SCL, SDA)
        ads = ADS.ADS1115(i2c)
        self.chan0 = AnalogIn(ads, ADS.P0)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servo = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.servo.angle = 0

        ros = False

        if ros:
            pub_topic = 'servo_reading'
            self.servo_pub = rospy.Publisher(pub_topic, servo_reading, queue_size=1)
            rospy.init_node('ros_servo')
            rate = rospy.Rate(1)
            self.servo_msg = servo_reading()

            while not rospy.is_shutdown():
                self.filter_test(pub=True)
                rate.sleep()
        
        else:
            self.filter_test(pub=False)

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

    def ros_publish_readings(self, value, voltage, angle):
        self.servo_msg.encoder_value = value
        self.servo_msg.encoder_voltage = voltage
        self.servo_msg.servo_angle = angle

        self.servo_pub.publish(self.servo_msg)
        return

    def exponential_moving_average(data, alpha):
        ema = [data[0]]
        for i in range(1, len(data)):
            ema.append(alpha * data[i] + (1 - alpha) * ema[-1])
        return ema
    
    def butter_lowpass_filter(self, data, cutoff_freq, sampling_rate, order=4):
        nyquist = 0.5 * sampling_rate
        normal_cutoff = cutoff_freq / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)

        # Add padding to the input data
        padlen = 15
        padded_data = [data[0]] * padlen + data

        # Apply the filter
        filtered_data = filtfilt(b, a, padded_data)

        # Remove the padded values
        filtered_data = filtered_data[padlen:]

        return filtered_data

    def filter_test(self, pub):
        test = [0, 90, 180, 270, 180, 90, 0]

        raw_angle_list = []

        moving_average_list = []
        window_size = 2

        exp_average_list = []
        alpha = 0.75

        low_pass_list = []
        cutoff_freq = 0.1


        print("{:>5}\t{:>5}\t{:>5}".format('raw', 'v', 'deg'))

        for theta in test:
            desiredPos = theta
            self.servo.angle = desiredPos
            n = 0

            while n < 20:
                value = self.chan0.value
                voltage = self.chan0.voltage
                angle = self.enc_2_deg(self.chan0.voltage)

                raw_angle_list.append(angle)

                # Apply moving average filter
                if len(raw_angle_list) >= window_size:
                    moving_avg_angle = sum(raw_angle_list[-window_size:]) / window_size
                else:
                    moving_avg_angle = angle  # If there are not enough readings, use the raw angle

                moving_average_list.append(moving_avg_angle)

                # Apply Exponential Moving Average filter
                if len(exp_average_list) == 0:
                    exp_filtered_angle = angle
                else:
                    exp_filtered_angle = alpha * angle + (1 - alpha) * exp_average_list[-1]

                exp_average_list.append(exp_filtered_angle)

                # Apply low-pass Butterworth filter
                if len(raw_angle_list) > 2:
                    low_pass_angle = self.butter_lowpass_filter(raw_angle_list[-3:], cutoff_freq, sampling_rate=1, order=4)[-1]
                else:
                    low_pass_angle = angle  # If there are not enough readings, use the raw angle

                low_pass_list.append(low_pass_angle)

                print("{:>5}\t{:>5.3f}\t{:>5.3f}".format(value, voltage, angle))

                if pub:
                    self.ros_publish_readings(value, voltage, angle)

                n += 1

        
        # Plot the readings
        plt.plot(raw_angle_list, label='raw')
        plt.plot(moving_average_list, label='moving average')
        plt.plot(exp_average_list, label='exp moving average')
        # plt.plot(low_pass_list, label='low pass')
        plt.xlabel('Reading Number')
        plt.ylabel('Angle (Degrees)')
        plt.legend()

        plt.yticks(range(0, 270 + 1, 30))
        plt.grid(True)

        plt.savefig('Filtered_Data.png')

if __name__ == '__main__':
    try:
        ros_servo = Servo()
    except rospy.ROSInterruptException:
        pass