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



class Servo:
    def __init__(self) -> None:
        
        i2c = busio.I2C(SCL, SDA)
        ads = ADS.ADS1115(i2c)
        self.chan0 = AnalogIn(ads, ADS.P0)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servo = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)

        ros = True

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

    def enc_2_deg(self, val_reading, V_reading, valmin = 2090, valmax = 25565, Vmin = 0.261, Vmax = 3.573):
        a = 0.75
        angle1 = self.map_range(V_reading, Vmin, Vmax, 0, 270)
        angle2 = self.map_range(val_reading, valmin, valmax, 0, 270)
        angle = angle1*a + angle2*(1-a)
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
    
    def filter_test(self, pub):
        test = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0]
        # test = [270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270] 
        # test = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270]

        self.servo.angle = test[0]
        time.sleep(2)

        prediction = []

        error_list = []

        raw_angle_list = []

        # exp_average_list = []
        # alpha = 0.8


        print("{:>5}\t{:>5}\t{:>5}".format('raw', 'v', 'deg'))

        for theta in test:
            desiredPos = theta
            self.servo.angle = desiredPos
            n = 0

            while n < (3 + 1):
                value = self.chan0.value
                voltage = self.chan0.voltage
                angle = self.enc_2_deg(self.chan0.value, self.chan0.voltage)

                # Adjusting for Error
                angle = angle + 0.13*angle + 0.7

                raw_angle_list.append(angle)

                # Apply Exponential Moving Average filter
                """if len(exp_average_list) == 0:
                    exp_filtered_angle = angle
                else:
                    exp_filtered_angle = alpha * angle + (1 - alpha) * exp_average_list[-1]

                exp_average_list.append(exp_filtered_angle)"""
                
                
                prediction.append(theta)

                print("{:>5}\t{:>5.3f}\t{:>5.3f}".format(value, voltage, angle))

                if pub:
                    self.ros_publish_readings(value, voltage, angle)

                n += 1
            
            error = theta - angle

            error_list.append(error)
        
        if not pub:
            # Plot the readings
            plt.plot(prediction, label='prediction', color='red')
            plt.plot(raw_angle_list, label='raw', color='blue')
            # plt.plot(exp_average_list, label='exp moving average', linestyle='-.', color='black')
            plt.xlabel('Reading Number')
            plt.ylabel('Angle (Degrees)')
            plt.legend()

            plt.yticks(range(0, 270 + 1, 30))
            plt.grid(True)

            plt.savefig('Filtered_Data.png')
            plt.close()

            '''
            plt.plot(error_list, label='error', color='red')
            plt.xlabel('Reading Number')
            plt.ylabel('Error (Degrees)')
            plt.legend()
            plt.grid(True)

            plt.savefig('Error.png')
            plt.close()

            
            # Fit a linear polynomial (degree=1)
            coefficients = np.polyfit(test, error_list, 1)

            # Create the best-fit line using the obtained coefficients
            best_fit_line = np.poly1d(coefficients)

            # Generate errors for the best-fit line
            errors_fit = best_fit_line(test)

            # Plot the data points and the best-fit line
            plt.scatter(test, error_list, label='error')
            plt.plot(test, errors_fit, label=f'Best Fit Line: error = {coefficients[0]:.2f} * angle + {coefficients[1]:.2f}', color='red')
            plt.xlabel('Angle (degrees)')
            plt.ylabel('Error (degrees)')
            plt.legend()

            plt.savefig('Error_Regression.png')
            plt.close()
            '''
        

if __name__ == '__main__':
    try:
        ros_servo = Servo()
    except rospy.ROSInterruptException:
        pass