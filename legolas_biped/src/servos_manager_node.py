#!/usr/bin/env python3
  
import rospy
from legolas_biped.msg import joint_angles
import time

import numpy as np

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class Biped:
    def __init__(self) -> None:

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        self.initialize_servos()
        self.reset_biped()

        sub_topic = 'joint_angles'

        self.joint = rospy.Subscriber(sub_topic, joint_angles , self.move_biped)


    def initialize_servos(self):
        
        # Declaring pin numbers of each servos on servo driver

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

        # Initializing each servos arcordingly

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

        return

    def reset_biped(self):
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

        print("Resetting Biped ", end='')

        # Adding a comma after 1 second
        time.sleep(1)
        print(".", end='', flush=True)

        time.sleep(1)
        print(".", end='', flush=True)

        time.sleep(1)
        print(".", end=' ', flush=True)

        time.sleep(1)
        print("Complete!", end='', flush=True)

        return
    
    def left_leg_callback(self, msg):
        return msg.Left_Joints
    
    def right_leg_callback(self, msg):
        return msg.Right_Joints

    
    def move_biped(self, msg):
        RHip1_angle, RHip2_angle, RThigh_angle, RForeleg_angle, RCalf_angle = self.right_leg_callback(msg)
        LHip1_angle, LHip2_angle, LThigh_angle, LForeleg_angle, LCalf_angle = self.left_leg_callback(msg)

        self.RHip1_servo.angle      = RHip1_angle
        self.RHip2_servo.angle      = RHip2_angle
        self.RThigh_servo.angle     = RThigh_angle
        self.RForeleg_servo.angle   = RForeleg_angle
        self.RCalf_servo.angle      = RCalf_angle

        self.LHip1_servo.angle      = LHip1_angle
        self.LHip2_servo.angle      = LHip2_angle
        self.LThigh_servo.angle     = LThigh_angle
        self.LForeleg_servo.angle   = LForeleg_angle
        self.LCalf_servo.angle      = LCalf_angle

        return
        
        
if __name__ == '__main__':
    rospy.init_node('biped')
        
    biped = Biped()

    rate = rospy.Rate(5) # ROS Rate at 5Hz
    
    while not rospy.is_shutdown():
        rate.sleep()
