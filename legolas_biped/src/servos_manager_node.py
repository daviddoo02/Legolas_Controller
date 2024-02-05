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

        self.joint = rospy.Subscriber(sub_topic, joint_angles, self.move_biped)

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

        self.RHip1_servo = servo.Servo(
            self.pca.channels[RHip1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RHip2_servo = servo.Servo(
            self.pca.channels[RHip2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RThigh_servo = servo.Servo(
            self.pca.channels[RThigh], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RForeleg_servo = servo.Servo(
            self.pca.channels[RForeleg], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.RCalf_servo = servo.Servo(
            self.pca.channels[RCalf], min_pulse=500, max_pulse=2600, actuation_range=270)

        self.LHip1_servo = servo.Servo(
            self.pca.channels[LHip1], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LHip2_servo = servo.Servo(
            self.pca.channels[LHip2], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LThigh_servo = servo.Servo(
            self.pca.channels[LThigh], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LForeleg_servo = servo.Servo(
            self.pca.channels[LForeleg], min_pulse=500, max_pulse=2600, actuation_range=270)
        self.LCalf_servo = servo.Servo(
            self.pca.channels[LCalf], min_pulse=500, max_pulse=2600, actuation_range=270)

        return

    def sim2real_angles(self, simth1, simth2, simth3, simth4, simth5, left=True):
        if left:
            # Crouching 0 config
            th_1_0 = 184
            th_2_0 = 80
            th_3_0 = 65
            th_4_0 = 230
            th_5_0 = 190

            # Map to real angles
            real_th1 = th_1_0 - simth1
            real_th2 = th_2_0 + simth2
            real_th3 = th_3_0 - simth3
            real_th4 = th_4_0 - simth4
            real_th5 = th_5_0 - simth5
        else:
            # Crouching 0 config
            th_1_0 = 5
            th_2_0 = 100
            th_3_0 = 110
            th_4_0 = 120
            th_5_0 = 85

            # Map to real angles
            real_th1 = th_1_0 + simth1
            real_th2 = th_2_0 - simth2
            real_th3 = th_3_0 + simth3
            real_th4 = th_4_0 + simth4
            real_th5 = th_5_0 + simth5

        return real_th1, real_th2, real_th3, real_th4, real_th5

    def reset_biped(self):
        # Crouching 0 config

        rhip1, rhip2, rthigh, rforeleg, rcalf = self.sim2real_angles(
            0, 0, 0, 0, 0, left=False)
        lhip1, lhip2, lthigh, lforeleg, lcalf = self.sim2real_angles(
            0, 0, 0, 0, 0, left=True)

        self.RHip1_servo.angle = rhip1
        self.RHip2_servo.angle = rhip2
        self.RThigh_servo.angle = rthigh
        self.RForeleg_servo.angle = rforeleg
        self.RCalf_servo.angle = rcalf

        self.LHip1_servo.angle = lhip1
        self.LHip2_servo.angle = lhip2
        self.LThigh_servo.angle = lthigh
        self.LForeleg_servo.angle = lforeleg
        self.LCalf_servo.angle = lcalf

        print("Resetting Biped ", end='')

        # Adding a comma after 1 second
        time.sleep(1)
        print(".", end='', flush=True)

        time.sleep(0.5)
        print(".", end='', flush=True)

        time.sleep(0.5)
        print(".", end=' ', flush=True)

        time.sleep(0.5)
        print("Complete!", end='', flush=True)

        return

    def left_leg_callback(self, msg):
        simhip1, simhip2, simthigh, simforeleg, simcalf = msg.Left_Joints

        realhip1, realhip2, realthigh, realforeleg, realcalf = self.sim2real_angles(
            simhip1, simhip2, simthigh, simforeleg, simcalf, left=True)
        return realhip1, realhip2, realthigh, realforeleg, realcalf

    def right_leg_callback(self, msg):
        simhip1, simhip2, simthigh, simforeleg, simcalf = msg.Right_Joints

        realhip1, realhip2, realthigh, realforeleg, realcalf = self.sim2real_angles(
            simhip1, simhip2, simthigh, simforeleg, simcalf, left=False)
        return realhip1, realhip2, realthigh, realforeleg, realcalf

    def move_biped(self, msg):
        RHip1_angle, RHip2_angle, RThigh_angle, RForeleg_angle, RCalf_angle = self.right_leg_callback(
            msg)
        LHip1_angle, LHip2_angle, LThigh_angle, LForeleg_angle, LCalf_angle = self.left_leg_callback(
            msg)

        self.RHip1_servo.angle = RHip1_angle
        self.RHip2_servo.angle = RHip2_angle
        self.RThigh_servo.angle = RThigh_angle
        self.RForeleg_servo.angle = RForeleg_angle
        self.RCalf_servo.angle = RCalf_angle

        self.LHip1_servo.angle = LHip1_angle
        self.LHip2_servo.angle = LHip2_angle
        self.LThigh_servo.angle = LThigh_angle
        self.LForeleg_servo.angle = LForeleg_angle
        self.LCalf_servo.angle = LCalf_angle

        return


if __name__ == '__main__':
    rospy.init_node('biped')

    biped = Biped()

    rate = rospy.Rate(5)  # ROS Rate at 5Hz

    while not rospy.is_shutdown():
        rate.sleep()
