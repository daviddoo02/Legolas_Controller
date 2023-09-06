import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from kinematics import Leg, LegList, render_leg, inverse_kinematics, keep_foot_flat, Background
import pygame
import numpy as np
import time
import csv

d2r = np.deg2rad
r2d = np.rad2deg

input_gait = []
with open("Gait1.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        input_gait.append(row)


BackGround = Background('Legolas C-Space.png', [0,0])

leg_list = LegList()
leg = Leg()

#init pygame
pygame.init()
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("Leg Simulator")
clock = pygame.time.Clock()
running = True


hip_angle_prev = d2r(115)
knee_angle_prev = d2r(120)
calf_angle_prev = d2r(85)

while running:
    for coords in input_gait:
        trace_gait = False          # This will draw out every step the leg takes if True
        if not trace_gait:
            print("Here")
            clock.tick(100)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            screen.fill([255, 255, 255])
            # screen.fill((0, 0, 0))
            screen.blit(BackGround.image, BackGround.rect)

        render_leg(leg_list, leg_list.legs[0], leg_list.legs[0].segments[2].abs_end_coords(), screen)
        pygame.display.update()

        toe_coords = leg.segments[2].motor.connecting_rod_end
        heel_y_coords = toe_coords[1]

        angle1, angle2 = inverse_kinematics(leg, coords)
        angle3 = keep_foot_flat(leg, heel_y_coords)         # Tweak Desired Pos

        hip_angle = angle1
        knee_angle = angle2
        calf_angle = angle3

        smoothing = False
        if smoothing:
            smoothing_factor = 0.1

            hip_angle_smoothed = (hip_angle * smoothing_factor) + (hip_angle_prev * (1-smoothing_factor))
            knee_angle_smoothed = (knee_angle * smoothing_factor) + (knee_angle_prev * (1-smoothing_factor))
            calf_angle_smoothed = (calf_angle * smoothing_factor) + (calf_angle_prev * (1-smoothing_factor))

            hip_angle_prev = hip_angle_smoothed
            knee_angle_prev = knee_angle_smoothed
            calf_angle_prev = calf_angle_smoothed
            leg_list.legs[0].move_motor(hip_angle_smoothed, 0, False)
            leg_list.legs[0].move_motor(knee_angle_smoothed, 1, False)
            leg_list.legs[0].move_motor(calf_angle_smoothed, 2, False)
        else:
            leg_list.legs[0].move_motor(hip_angle, 0, False)
            leg_list.legs[0].move_motor(knee_angle, 1, False)
            leg_list.legs[0].move_motor(calf_angle, 2, False)
        
        time.sleep(0.02)
        # print("Hip:", hip_angle_smoothed * 180/pi)
        # print("Knee:", knee_angle_smoothed * 180/pi)
