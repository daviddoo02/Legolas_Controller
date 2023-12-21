#!/usr/bin/env python3
  
import rospy
from ros_servo.msg import servo_reading
from filterpy.kalman import KalmanFilter

import matplotlib.pyplot as plt
import numpy as np

from typing import Tuple
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
        self.servo.angle = 270
        time.sleep(1)

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
    
    def kalman_filter(self, measurement: float, initial_state: float, process_variance: float, measurement_variance: float ) -> float:
        kf = KalmanFilter(dim_x=1, dim_z=1)

        # State transition matrix
        kf.F = np.array([[1]])

        # Measurement function
        kf.H = np.array([1])

        # Initial state
        kf.x = np.array([initial_state])

        # Initial state covariance
        kf.P *= 1e2

        # Process noise covariance
        kf.Q *= process_variance

        # Measurement noise covariance
        kf.R *= measurement_variance

        # Prediction step
        kf.predict()

        # Update step
        kf.update(np.array([measurement]))  # Ensure measurement is a 1D array

        # Get filtered state mean
        filtered_state_mean = kf.x[0]

        return filtered_state_mean
    
    def filter_test(self, pub):
        # test = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0]
        test = [270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270]
        # test = np.linspace(0, 270, 3)

        prediction = []

        raw_angle_list = []

        exp_average_list = []
        alpha = 0.6

        kalman_filtered_list = []


        print("{:>5}\t{:>5}\t{:>5}".format('raw', 'v', 'deg'))

        for theta in test:
            desiredPos = theta
            self.servo.angle = desiredPos
            prev_angle = self.enc_2_deg(self.chan0.voltage)
            n = 0

            while n < 5:
                value = self.chan0.value
                voltage = self.chan0.voltage
                angle = self.enc_2_deg(self.chan0.voltage)

                raw_angle_list.append(angle)

                # Apply Exponential Moving Average filter
                if len(exp_average_list) == 0:
                    exp_filtered_angle = angle
                else:
                    exp_filtered_angle = alpha * angle + (1 - alpha) * exp_average_list[-1]

                exp_average_list.append(exp_filtered_angle)

                # Apply Kalman filter
                if len(kalman_filtered_list) == 0:
                    kalman_angle = angle
                else:
                    kalman_angle = self.kalman_filter(angle, initial_state=theta, process_variance=1, measurement_variance=1)

                kalman_filtered_list.append(kalman_angle)

                prediction.append(theta)

                print("{:>5}\t{:>5.3f}\t{:>5.3f}".format(value, voltage, angle))

                if pub:
                    self.ros_publish_readings(value, voltage, angle)

                n += 1
                prev_angle = angle

        
        # Plot the readings
        plt.plot(prediction, label='prediction', color='red')
        plt.plot(raw_angle_list, label='raw', color='blue')
        plt.plot(exp_average_list, label='exp moving average', linestyle='-.', color='black')
        plt.plot(kalman_filtered_list, label='kalman', linestyle='-.', color='purple')
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