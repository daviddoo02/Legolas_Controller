#!/usr/bin/env python3
  
import rospy
from inverse_kinematics.msg import joint_angles

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import time

class Biped:
    
    def __init__(self):

        sub_topic = 'joint_angles'			

        i2c = busio.I2C(SCL, SDA)
        
        self.pca = PCA9685(i2c)

        self.pca.frequency = 50
    
        self.hip1_servo = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.hip2_servo = servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.thigh_servo = servo.Servo(self.pca.channels[2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.foreleg_servo = servo.Servo(self.pca.channels[3], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.calf_servo = servo.Servo(self.pca.channels[4], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.thigh_prev = 65
        self.foreleg_prev = 35
        self.calf_prev = 45

        self.joint = rospy.Subscriber(sub_topic, joint_angles , self.move_leg)

        while not rospy.is_shutdown():
            rospy.spin()

    def hip_pitch_callback(self, msg):
        return msg.hip_pitch
    
    def knee_callback(self, msg):
        return msg.knee
    
    def calf_callback(self, msg):
        return msg.calf

    def move_leg(self, msg):
        thigh_angle = self.hip_pitch_callback(msg)
        foreleg_angle = self.knee_callback(msg)
        calf_angle = self.calf_callback(msg)

        # sf1, sf2, sf3 = 0.1, 0.1, 0.1

        # thigh_smoothed = (thigh_angle * sf1) + (self.thigh_prev * (1-sf1))
        # foreleg_smoothed = (foreleg_angle * sf2) + (self.foreleg_prev * (1-sf2))
        # calf_smoothed = (calf_angle * sf3) + (self.calf_prev * (1-sf3))

        # self.thigh_prev = thigh_smoothed
        # self.foreleg_prev = foreleg_smoothed
        # self.calf_prev = calf_smoothed

        # self.thigh_servo.angle = thigh_smoothed
        # self.foreleg_servo.angle = foreleg_smoothed
        # self.calf_servo.angle = calf_smoothed

        self.thigh_servo.angle = thigh_angle
        self.foreleg_servo.angle = foreleg_angle
        self.calf_servo.angle = calf_angle

        

if __name__ == '__main__':
    rospy.init_node('biped')

    biped = Biped()

    # Declare this after ros spin

    # i2c = busio.I2C(SCL, SDA)

    # pca = PCA9685(i2c)

    # pca.deinit()