#!/usr/bin/env python3

"""-----------------Instructions--------------------------

Welcome to the kinematics demo of Legolas.
This demo was developed as a result of testing & debugging kinematics.py
by Daniel Kutch, and was later tweaked by me to be compatible with ROS and joystick.

Have fun!
David Ho

------------------------------------------------------- """

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from kinematics import LegList, render_leg, Background
import pygame
import numpy as np

# ---------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Joy
from inverse_kinematics.msg import joint_angles


# ---------------------------------------------------------------------------

d2r = np.deg2rad
r2d = np.rad2deg



class Simulation:
    def __init__(self):
        
        self.display_sim = False        # Enabling this will render the leg in pygame. While this work on faster computer, it slows down considerably on the Pi.

        #init pygame
        if self.display_sim:
            self.BackGround = Background('Legolas_C_Space.png', [0,0])
            pygame.init()
            self.screen = pygame.display.set_mode((800, 800))
            pygame.display.set_caption("Leg Simulator")
            self.clock = pygame.time.Clock()
        
        self.reset()

        sub_topic = 'joy_throttled'	
        pub_topic = 'joint_angles'	
        
        self.JoySub = rospy.Subscriber(sub_topic, Joy , self.run_simulation)

        self.angle_pub = rospy.Publisher(pub_topic, joint_angles, queue_size=1)
        
        self.angle_msg = joint_angles()

        while not rospy.is_shutdown():
            rospy.spin()
    
    def reset(self):
        self.leg_list = LegList()
        self.mouse = self.leg_list.mouse
        self.leg = self.leg_list.legs


    def axes_callback(self, msg):
        return msg.axes

    def buttons_callback(self, msg):
        return msg.buttons
    
    def run_simulation(self, msg):
        if self.display_sim:

            self.clock.tick(100)
            self.screen.fill([255, 255, 255])
            self.screen.blit(self.BackGround.image, self.BackGround.rect)
            
            render_leg(self.leg_list, self.leg[0], self.mouse, self.screen)
            pygame.display.update()

        self.joystick_input(msg)

    def Clamp(num,min,max):
        if(num  < min):
            num=min
        if(num > max):
            num=max
        return num


    def joystick_input(self, msg):
        axes = self.axes_callback(msg)
        buttons = self.buttons_callback(msg)

        if buttons[7]: # Start Button
            self.reset()

        if buttons[6]: # Back Button
            rospy.signal_shutdown("Terminating program")

        toe_coords = self.leg[0].segments[2].motor.connecting_rod_end
        heel_y_coords = toe_coords[1]
        # foot_coords = self.leg[0].segments[2].abs_end_coords()
        # print("Foot:", foot_coords)

        speed = 20

        self.mouse = (self.mouse[0] - speed*np.float16(axes[3]), self.mouse[1] + speed*np.float16(axes[4]))
        hip,knee,calf = self.leg_list.inv_kine_TRUE(self.leg[0], heel_y_coords, self.mouse, True)

        self.angle_msg.hip_pitch = r2d(hip) + 180                  # Virtual: 150 - 90 where 150 is forward;  Real: 90 - 30 where 30 is forward
        self.angle_msg.knee = -r2d(knee) + 155                  # Virtual: 120 - 55 where 120 is full-contract;  Real: 100 - 35 where 35 is full-contract
        self.angle_msg.calf = -r2d(calf) + 180                  # Virtual: (0) 30 - 135 where 30 is toe up; Real: (60) 140 - 45 where 140 is toe up

        rospy.loginfo(self.angle_msg)
        self.angle_pub.publish(self.angle_msg)




if __name__ == '__main__':
    rospy.init_node('joint_angles')
    Sim = Simulation()

    

    # else:
    #     #init pygame
    #     leg_list = LegList()
    #     pygame.init()
    #     screen = pygame.display.set_mode((800, 800))
    #     pygame.display.set_caption("Leg Simulator")
    #     clock = pygame.time.Clock()
    #     running = True

    #     while running:
    #         clock.tick(50)
    #         for event in pygame.event.get():
    #             if event.type == pygame.QUIT:
    #                 running = False
            
    #         screen.fill([255, 255, 255])
    #         # screen.fill((0, 0, 0))
    #         screen.blit(BackGround.image, BackGround.rect)

    #         render_leg(leg_list, leg_list.legs[0], leg_list.mouse, screen)
    #         pygame.display.update()
    #         leg_list.inputs(0, pygame)