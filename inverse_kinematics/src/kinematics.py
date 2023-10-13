# This code was written by Daniel Kutch 2023
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from cmath import cos, sin, pi
import numpy as np
from math import atan2
import pygame
from pygame.math import clamp
import math
import typing

d2r = np.deg2rad
r2d = np.rad2deg

class Motor:
    def __init__(self, motor_nub_length :float, angle:float, motor_nub_offset, end_nub_radius:float, end_nub_angle_offset:float, connecting_rod_length:float, connecting_rod_end, bend:bool):
        self.motor_nub_length = motor_nub_length
        self.angle = angle
        self.motor_nub_offset = motor_nub_offset
        self.end_nub_radius = end_nub_radius
        self.end_nub_angle_offset = end_nub_angle_offset
        self.connecting_rod_length = connecting_rod_length
        self.connecting_rod_end = connecting_rod_end
        self.bend = bend
        self.highlighted = 0

class JointAndConnector:
    def __init__(self, origin_coords, end_nub_radius, connecting_rod_length, connecting_rod_end, end_nub_angle_offset, bend):
        self.origin_coords = origin_coords
        self.end_nub_radius = end_nub_radius
        self.connecting_rod_length = connecting_rod_length
        self.connecting_rod_end = connecting_rod_end
        self.end_nub_angle_offset = end_nub_angle_offset
        self.bend = bend

class Motorless:
    def __init__(self):
        self.yeet = False

class Joint:
    def __init__(self):
        self.yeet = False

class LegSegment:
    def __init__(self, motor_type): # all segments contain these values, motor_type contains the unique information
        self.rel_coords = (0.0, 0.0)
        self.abs_coords = (0.0, 0.0)
        self.rel_angle = 0.0
        self.abs_angle = 0.0
        self.length = 60.0
        self.max_angle = 3.5
        self.min_angle = -3.5
        self.motor = motor_type

    def update_position(self, starting_coords, starting_angle):
        #print("updating position")
        self.abs_coords = (
            self.rel_coords[0] + starting_coords[0],
            self.rel_coords[1] + starting_coords[1],
        )
        self.abs_angle = self.rel_angle + starting_angle
        next_leg_angle = self.abs_angle
        next_leg_position = self.abs_end_coords()

        coords1 = self.motor_end_coords()
        coords2 = self.abs_end_coords()
        #print("coords1:", coords1)
        #print("coords2:", coords2)
        if isinstance(self.motor, Motor):
            r1 = self.motor.connecting_rod_length
            r2 = self.motor.end_nub_radius
            connector_end_pt = find_circle_intersections(coords1, coords2, r1, r2, self.motor.bend)
            if connector_end_pt != None:
                self.motor.connecting_rod_end = connector_end_pt

                x_side = coords2[0] - connector_end_pt[0]
                y_side = coords2[1] - connector_end_pt[1]
                angle = atan2(y_side, x_side)

                next_leg_angle = angle + self.motor.end_nub_angle_offset
            else:
                #print("no circle intersection MOTOR!")
                return None
        elif isinstance(self.motor, JointAndConnector):
            r1 = self.motor.connecting_rod_length
            r2 = self.motor.end_nub_radius
            connector_end_pt = find_circle_intersections(self.motor.origin_coords, coords2, r1, r2, self.motor.bend)
            if connector_end_pt != None:
                self.motor.connecting_rod_end = connector_end_pt

                x_side = coords2[0] - connector_end_pt[0]
                y_side = coords2[1] - connector_end_pt[1]
                angle = atan2(y_side, x_side)
                next_leg_angle = angle + self.motor.end_nub_angle_offset
            else:
                #print("no circle intersection JOC!")
                return None

        return (next_leg_position, next_leg_angle)

    def abs_end_coords(self):
        trig_mult = get_trig_mult(-self.abs_angle)
        trans_coords = rotate_coords_f(trig_mult, (self.length, 0.0))
        trans_coords2 = (trans_coords[0] + self.abs_coords[0], trans_coords[1] + self.abs_coords[1])
        return trans_coords2

    # returns the coordinates of the motor head
    def motor_nub_root_coords(self):
        trig_mult = get_trig_mult(- self.abs_angle)
        trans_coords = rotate_coords_f(trig_mult, (self.motor.motor_nub_offset[0], self.motor.motor_nub_offset[1]))
        return (self.abs_coords[0] + trans_coords[0], self.abs_coords[1] + trans_coords[1])

    # returns coordinates of the end of the motor nub
    def motor_end_coords(self):
        if isinstance(self.motor, Motor):
            trig_mult = get_trig_mult(-self.motor.angle - self.abs_angle)
            trans_coords = rotate_coords_f(trig_mult, (self.motor.motor_nub_length, 0.0))
            trans_coords2 = (
                    trans_coords[0] + self.motor_nub_root_coords()[0],
                    trans_coords[1] + self.motor_nub_root_coords()[1],
            )
            return trans_coords2
        else:
            return None

def find_circle_intersections(coords1, coords2, r1, r2, which_root):
    try:
        x1 = coords1[0]
        x2 = coords2[0]
        y1 = coords1[1]
        y2 = coords2[1]
        r1 = r1
        r2 = r2
        d = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        l = (r1**2 - r2**2 + d**2) / (2 * d)
        h = (r1**2 - l**2)**0.5

        if which_root:
            x = l / d * (x2 - x1) + h / d * (y2 - y1) + x1
            y = l / d * (y2 - y1) - h / d * (x2 - x1) + y1
        else:
            x = l / d * (x2 - x1) - h / d * (y2 - y1) + x1
            y = l / d * (y2 - y1) + h / d * (x2 - x1) + y1

        # break if x or y is complex
        if isinstance(x, complex) or isinstance(y, complex):
            #print("-------------IS COMPLEX")
            return None
        elif math.isinf(x) or math.isinf(y):
            #print("--------------IS INF")
            return None
        else:
            #print("---------------returning")
            return (x, y)
    except:
        #print("--------------EXCEPTION")
        return None

def get_trig_mult(angle):
    return (sin(angle).real, cos(angle).real)

def rotate_coords_f(trig_mult, coords):
    return (
        trig_mult[0] * coords[1] + trig_mult[1] * coords[0],
        trig_mult[1] * coords[1] - trig_mult[0] * coords[0]
    )

def rect_to_polar(origin, rect):
    x_side = rect[0] - origin[0]
    y_side = rect[1] - origin[1]
    angle = math.atan2(y_side, x_side)
    radius = math.sqrt((rect[0] - origin[0])**2 + (rect[1] - origin[1])**2)
    return (angle, radius)

class Leg:
    def __init__(self):
        origin_coords1 = (37.5, -16.0)
        end_nub_radius1 = 45.71
        connecting_rod_length1 = 179.0
        connecting_rod_end1 = (100.0, 100.0)
        end_nub_angle_offset1 = d2r(-17.39)
        bend1 = False

        motor_nub_length2 = 32.0
        angle2 = 1.5
        motor_nub_offset2 = (30.83, 19.25) # 30.83, 19.25
        end_nub_radius2 = 49.4
        end_nub_angle_offset2 = d2r(53.75) #53.75 26.9    previous: #0.909 # 52.081864 deg
        connecting_rod_length2 = 115.0
        connecting_rod_end2 = (100.0, 100.0)
        bend2 = False

        self.segments = [
            LegSegment(Joint()), # thigh
            LegSegment(JointAndConnector(origin_coords1, end_nub_radius1, connecting_rod_length1, connecting_rod_end1, end_nub_angle_offset1, bend1)), # foreleg
            LegSegment(Motor(motor_nub_length2, angle2, motor_nub_offset2, end_nub_radius2, end_nub_angle_offset2, connecting_rod_length2, connecting_rod_end2, bend2)), # shin
            LegSegment(Motorless()) # foot
        ]

        # Initialize Starting Angles

        # thigh
        # Hip Pitch, motor[0], init at 115 
        self.segments[0].rel_angle = d2r(-120)
        self.segments[0].length = 80.0
        self.segments[0].max_angle = d2r(-90)
        self.segments[0].min_angle = d2r(-150)

        # foreleg
        # #Knee, motor[1], init at 120
        self.segments[1].rel_angle = d2r(115)
        self.segments[1].length = 211.5
        self.segments[1].min_angle = d2r(60)
        self.segments[1].max_angle = d2r(120)

        # shin
        self.segments[2].length = 169.2
        self.segments[2].motor.angle = d2r(115)
        self.segments[2].min_angle = d2r(30)
        self.segments[2].max_angle = d2r(115)

        # foot
        # Calf, motor[2], init at 120
        self.segments[3].rel_angle = 0.0
        self.segments[3].length = 52.0


        self.update()

    def update(self):
        starting_coords = (0.0, 0.0)
        starting_angle = 0.0
        for leg_segment in self.segments:

            result = leg_segment.update_position(starting_coords, starting_angle)
            if result is None:
                return False
            else:
                starting_coords = result[0]
                starting_angle = result[1]
        return True

    def move_motor(self, angle, leg_segment_index, additive):
        segment = self.segments[leg_segment_index]
        if isinstance(segment.motor, Motor):
            if additive:
                segment.motor.angle += angle
                # print(f"{segment.motor}:", r2d(segment.motor.angle))
            else:
                segment.motor.angle = angle
            segment.motor.angle = clamp(segment.motor.angle, segment.min_angle, segment.max_angle)
        else: # segment is either a joint or motorless
            if additive:
                segment.rel_angle += angle
                # print(f"{segment}:", r2d(segment.rel_angle))
            else:
                segment.rel_angle = angle
            segment.rel_angle = clamp(segment.rel_angle, segment.min_angle, segment.max_angle)
        self.update()

    # updates leg with given motor angles and returns the error between the desired and actuall foot coords
    def update_and_distance(self, position, angle1, angle2):
        self.move_motor(angle1, 0, False)
        self.move_motor(angle2, 1, False)
        self.update
        foot_coords = self.segments[2].abs_end_coords()
        # print("Foot:", foot_coords)
        error = ((foot_coords[0] - position[0])**2 + (foot_coords[1] - position[1])**2)**0.5
        return error
    
    def update_and_distance_foot(self, position, angle1):
        self.move_motor(angle1, 2, False)
        self.update
        toe_coords = self.segments[3].abs_end_coords()[1]

        # print("Toe:", toe_coords)
        # print("Goal:", position)

        error = ((toe_coords - position)**2)**0.5
        return error
    
    # returns the angle of the given motor
    def get_motor_angle(self, leg_segment_index):
        segment = self.segments[leg_segment_index]
        if isinstance(segment.motor, Motor):
            return segment.motor.angle
        else:
            return segment.rel_angle

class LegList:
    def __init__(self):
        #print("-----LegList init")
        leg1 = Leg()

        self.legs = [leg1]

        endeffector_coords = self.legs[0].segments[2].abs_end_coords()
        
        self.mouse = endeffector_coords # (-28.129, -104.88)


    #control leg segment angles with pygame input
    def inputs(self, leg_index, pygame):
        keys = pygame.key.get_pressed()

        leg = self.legs[leg_index]
    
        toe_coords = leg.segments[2].motor.connecting_rod_end
        heel_y_coords = toe_coords[1]
            
        if keys[pygame.K_d]:    # Thigh, motor 0, max 150, min 90,
            leg.move_motor(0.01, 0, True)
        if keys[pygame.K_a]:
            leg.move_motor(-0.01, 0, True)

        if keys[pygame.K_w]:        # Foreleg, motor 1, max 130, min 80,
            leg.move_motor(0.01, 1, True)
        if keys[pygame.K_s]:
            leg.move_motor(-0.01, 1, True)

        if keys[pygame.K_q]:        # Calf, motor 2, max 135, min 40,
            leg.move_motor(0.01, 2, True)
        if keys[pygame.K_e]:
            leg.move_motor(-0.01, 2, True)

        speed = 3

        if keys[pygame.K_LSHIFT]:
            if keys[pygame.K_UP]:
                self.mouse = (self.mouse[0], self.mouse[1] + speed)
                self.inv_kine_TRUE(leg, heel_y_coords, self.mouse, False)
            if keys[pygame.K_DOWN]:
                self.mouse = (self.mouse[0], self.mouse[1] - speed)
                self.inv_kine_TRUE(leg, heel_y_coords, self.mouse, False)
            if keys[pygame.K_LEFT]:
                self.mouse = (self.mouse[0] - speed, self.mouse[1])
                self.inv_kine_TRUE(leg, heel_y_coords, self.mouse, False)
            if keys[pygame.K_RIGHT]:
                self.mouse = (self.mouse[0] + speed, self.mouse[1])
                self.inv_kine_TRUE(leg, heel_y_coords, self.mouse, False)
        
    def inv_kine_TRUE(self, leg, heel_y_coords, mouse, pub):
        angle1, angle2 = inverse_kinematics(leg, mouse)
        angle3 = keep_foot_flat(leg, heel_y_coords)         # Tweak Desired Pos
        leg.move_motor(angle1, 0, False)
        leg.move_motor(angle2, 1, False)
        leg.move_motor(angle3, 2, False)

        if pub:
            return angle1, angle2, angle3

    def update_all_legs(self):
        for leg in self.legs:
            leg.update()



def render_leg(leg_list, leg, mouse, screen):
    scale = 1.25
    # render mouse dot
    # 
    # pygame.draw.circle(screen, (270, 27, 27), (mouse[0]+400, 600-(mouse[1]+300)), 5)
    draw_circle(screen, (255, 255, 255), (mouse), 5, scale)


    for leg in leg_list.legs:
        for leg_segment in leg.segments:
            pt1 = leg_segment.abs_coords
            pt2 = leg_segment.abs_end_coords()
            draw_line(screen, (150, 150, 150), pt1, pt2, scale)

            pt4 = leg_segment.motor_end_coords()
            if leg_segment.motor is not None:

                if isinstance(leg_segment.motor, Motor):
                    pt3 = leg_segment.motor_nub_root_coords()
                    draw_line(screen, (250, 150, 60), pt3, pt4, scale)

                    pt5 = leg_segment.motor.connecting_rod_end
                    draw_line(screen, (250, 150, 60), pt4, pt5, scale)
                elif isinstance(leg_segment.motor, JointAndConnector):
                    pt5 = leg_segment.motor.connecting_rod_end
                    draw_line(screen, (250, 150, 60), leg_segment.motor.origin_coords, pt5, scale)

def draw_line(screen, color, pt1, pt2, scale):
    offset = (357, 475)
    #pygame.draw.line(screen, color, (pt1[0]*2+offset[0], pt1[1]*2+offset[1]), (pt2[0]*2+offset[0], pt2[1]*2+offset[1]), 1)
    pygame.draw.line(screen, color, (pt1[0]*scale+offset[0], 600-(pt1[1]*scale+offset[1])), (pt2[0]*scale+offset[0], 600-(pt2[1]*scale+offset[1])), 3)

def draw_circle(screen, color, pt, radius, scale):
    offset = (357, 475)
    pygame.draw.circle(screen, color, (pt[0]*scale+offset[0], 600-(pt[1]*scale+offset[1])), radius*scale, 3)
    # px, py = pt[0]*scale+offset[0], 600-(pt[1]*scale+offset[1])
    # print(px, py)

def inverse_kinematics(leg, desired_point):
        angle1 = leg.get_motor_angle(0)
        angle2 = leg.get_motor_angle(1)
        step1 = 0.00001
        step2 = 0.00001
        tolerance = 1 # stop when the error is less than this or when weve done max_iterations
        max_iterations = 30
        iterations = 0
        while True:
            # error is the distance between the desired point and our current point. We want to minimize this.
            error = leg.update_and_distance(desired_point, angle1, angle2)
            if abs(error) < tolerance or iterations >= max_iterations:
                # print("iterations:", iterations)
                break
            # step in a direction to find the error of the 2 angles
            error_angle1 = leg.update_and_distance(desired_point, angle1+step1, angle2)
            error_angle2 = leg.update_and_distance(desired_point, angle1, angle2+step2)
            # this is the gradient
            grad1 = (error_angle1 - error) / step1
            grad2 = (error_angle2 - error) / step2
            # step in the direction of the gradient. grad is the slope (rate of change), error is the distance, so in theory grad*error is the distance we want to step to get to the desired point in one step, but lmao no, multiply by really small fraction. 0.00001 is the largest that doesnt overshoot.
            angle1 -= grad1 * error * 0.000005
            angle2 -= grad2 * error * 0.000005
            # every other iteration, step in the opposite direction. When the leg is up against 2 of the 4 walls, the gradient descent will get stuck trying to step into the wall and getting a gradient of 0.
            step1 *= -1
            step2 *= -1
            iterations += 1
        return (angle1, angle2)

def keep_foot_flat(leg, desired_point):
        angle1 = leg.get_motor_angle(2)
        step1 = 0.000005
        tolerance = 1 # stop when the error is less than this or when weve done max_iterations
        max_iterations = 30
        iterations = 0
        while True:
            # error is the distance between the desired point and our current point. We want to minimize this.
            error = leg.update_and_distance_foot(desired_point, angle1)
            if abs(error) < tolerance or iterations >= max_iterations:
                # print("iterations:", iterations)
                break
            # step in a direction to find the error of the 2 angles
            error_angle1 = leg.update_and_distance_foot(desired_point, angle1+step1)
            # this is the gradient
            grad1 = (error_angle1 - error) / step1
            # step in the direction of the gradient. grad is the slope (rate of change), error is the distance, so in theory grad*error is the distance we want to step to get to the desired point in one step, but lmao no, multiply by really small fraction. 0.00001 is the largest that doesnt overshoot.
            angle1 -= grad1 * error * 0.00001 # 0.000005
            # every other iteration, step in the opposite direction. When the leg is up against 2 of the 4 walls, the gradient descent will get stuck trying to step into the wall and getting a gradient of 0.
            step1 *= -1
            iterations += 1
        return (angle1)

class Background(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)  #call Sprite initializer
        self.image = pygame.image.load(image_file)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location


# #--------------------------------------------------
# leg_list = LegList()

# #init pygame
# pygame.init()

# # screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

# screen = pygame.display.set_mode((800, 800))

# pygame.display.set_caption("Leg Simulator")
# clock = pygame.time.Clock()
# running = True
# # make pygame loop
# while running:
#     #print("-----NEW LOOP-----")
#     clock.tick(50)
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     screen.fill((0, 0, 0))
#     render_leg(leg_list, leg_list.legs[0], leg_list.mouse, screen)
#     pygame.display.update()
#     leg_list.inputs(0, pygame)