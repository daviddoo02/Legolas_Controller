# This code was written by David Ho 2023

from kinematics import Leg, inverse_kinematics, keep_foot_flat
import numpy as np
import time
import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import savgol_filter
from kinematics import LegList
from cmath import pi








d2r = np.deg2rad
r2d = np.rad2deg

# ------------------ Visualizing initial leg position -------------------------- #
def get_leg_pos(hip_angle = 115, foreleg_angle = 120):
    """
    This function return the coords for a leg configuration given 2 motor inputs:
        hip_angle = hip motor angle - default is 115 - float
        foreleg_angle = foreleg motor angle - default is 120 - float

        Note that these motor angle is in tandem with the kinematics.py and follows it convention,
        which means that it would be different than the actual servo angles

    """
    leg = Leg()

    motor1_angle_init = d2r(hip_angle)
    motor2_angle_init = d2r(foreleg_angle)

    leg.move_motor(motor1_angle_init, 0, False)
    leg.move_motor(motor2_angle_init, 1, False)
    leg.update()

    thigh = leg.segments[0].abs_end_coords()
    foreleg = leg.segments[1].abs_end_coords()
    shin = leg.segments[2].abs_end_coords()

    leg_x = np.array([0, thigh[0], foreleg[0], shin[0]])
    leg_y = np.array([0, thigh[1], foreleg[1], shin[1]])

    return leg_x, leg_y

# ------------------ Visualizing C-space of leg -------------------------------- #
def c_space(resolution = 200):
    """
    This function generate the configuration space of Legolas.
    A configuration space is basically all the theoretical point a robot manipulator could reach.

    resolution = the resolution of the c-space - default is 200 - int

    return:
    lookup = the c-space which includes x and y positions as well as respective hip and knee angles for each positions
    reject = the rejected positions - maybe useful later

    """
    element = resolution

    leg1 = Leg()

    # Motor Ranges:
    # -------------------------------------------------------- #
    hip_pitch_range = np.linspace(d2r(-90),d2r(-150), element)
    knee_range = np.linspace(d2r(60), d2r(120), element)       
    # -------------------------------------------------------- #

    hip_pitch = []
    knee = []
    pos_x = []
    pos_y = []
    x_rej = []
    y_rej = []

    for index1 in range(len(hip_pitch_range)):
        for index2 in range(len(knee_range)):
            motor1_angle = hip_pitch_range[index1]
            motor2_angle = knee_range[index2]

            leg1.move_motor(motor1_angle, 0, False)
            leg1.move_motor(motor2_angle, 1, False)
            leg1.update()
            result_position = leg1.segments[2].abs_end_coords()
            
            if result_position[1] > -360:       # Rejecting failed poses
                hip_pitch.append(index1)
                knee.append(index2)
                pos_x.append(result_position[0])
                pos_y.append(result_position[1])
            else:
                x_rej.append(result_position[0])
                y_rej.append(result_position[1])

    hip_pitch = np.array(hip_pitch)
    knee = np.array(knee)
    pos_x = np.array(pos_x)
    pos_y = np.array(pos_y)

    lookup = np.stack((pos_x, pos_y, hip_pitch, knee),axis=1)

    x_rej = np.array(x_rej)
    y_rej = np.array(y_rej)

    reject = np.stack((x_rej, y_rej), axis=1)

    return lookup, reject

def unstack(a, axis=0):
    """
    Handy function to unstack arrays, a is for array
    """
    return np.moveaxis(a, axis, 0)

def visualize_c_space(lookup, reject, save, show):
    """
    Visualize the C-Space given pos_x and pos_y as arrays

    Inputs:
    lookup = c-space lookup table
    reject = rejected points
    save = save fig
    show = show fig

    """
    plt.close()

    pos_x, pos_y, hip, knee = unstack(lookup, axis=1)
    x_rej, y_rej = unstack(reject, axis=1)

    my_dpi = 120

    fig = plt.figure(figsize=(800/my_dpi, 800/my_dpi), dpi=my_dpi)

    ax = fig.add_subplot(111)

    # Fake bounding box to equalize aspect ratio
    # ---------------------------------------------------------------------------------------------------- #

    max_range = np.array([pos_x.max()-pos_x.min(), pos_y.max()-pos_y.min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(pos_x.max()+pos_x.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(pos_y.max()+pos_y.min())

    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb in zip(Xb, Yb):
        ax.plot([xb], [yb], 'w')

    # ---------------------------------------------------------------------------------------------------- #

    # plotting
    ax.plot(0, 0, 'o', markersize=2, color='green')
    ax.plot(pos_x, pos_y, 'o', markersize=1, color='darkviolet')
    ax.plot(x_rej, y_rej, 'o', markersize=1, color="lightcoral")
    ax.set_title('Legolas C-space')
    ax.set_aspect('equal')

    plt.grid()
    if show:
        plt.show()
    
    if save:
        fig.savefig('Legolas C-Space.png', dpi=my_dpi)

# ------------------------------------------------------------------------------ #
def trapezoid_gait(h = 125, bba = 230, bbb = 150, tbo = 60, gbo = -360, c = -30):
    """
    Function to generate the key features of a trapezoidal gait
    
    Inputs:
        Key features of the trapezoid:
            h = 150         # height of trapezoid
            bba = 300	    # bottom width back offset
            bbb = 140       # bottom width front offset
            tbo = 60        # top width offset

            # Don't change these 2 unless you know what you're doing
            gbo = -350      # gait base offset
            c = -10         # center of trapezoid

    Outputs:
        A 2D array containing the coordinates (x,y) of the trapezoidal gait
    """

    x = [c, c + bba, c + tbo, c - tbo, c - bbb, c]
    y = [gbo, gbo, h + gbo, h + gbo, gbo, gbo]

    coords = np.stack((x,y),axis=1)
    return coords

def generate_gait(gait_key_features, element = 20):
    x, y = unstack(gait_key_features, axis=1)

    gait_x = []
    gait_y = []

    for index in range(len(x)-1):
        if x[index] < x[index + 1]:
            i = np.linspace(x[index], x[index + 1], element)
        else:
            i = np.linspace(x[index + 1], x[index], element)
            i = i[::-1]

        if y[index] < y[index + 1]:
            j = np.linspace(y[index], y[index + 1], element)
        else:
            j = np.linspace(y[index + 1], y[index], element)
            j = j[::-1]

        gait_x = np.concatenate((gait_x, i), axis=None)
        gait_y = np.concatenate((gait_y, j), axis=None)

    # using a filter to smooth out the data
    win_size = element
    poly = 1

    gait_x = savgol_filter(gait_x, win_size, poly)
    gait_y = savgol_filter(gait_y, win_size, poly)

    return gait_x, gait_y

# hip_pitch_range = np.linspace(90, 150, 10)
# knee_range = np.linspace(70, 120, 10)  

debug = True

if debug:
    leg2_x, leg2_y = get_leg_pos(150, 70)

    lookup_table, reject_pos = c_space(200)

    save_fig = True
    show_fig = True
    
    visualize_c_space(lookup_table, reject_pos, save_fig, show_fig)

visualize_gait = False

if visualize_gait:
    leg = Leg()

    gait_key = trapezoid_gait()

    gait_x, gait_y = generate_gait(gait_key)

    lookup_table, reject_pos = c_space(200)

    visualize_c_space(lookup_table, reject_pos, False, False)

    plt.plot(gait_x, gait_y, marker="o", markersize=2, markeredgecolor="red", markerfacecolor="green")
    plt.show()

    gait = np.column_stack((gait_x, gait_y))

    # Use numpy.savetxt() method to save the list as a CSV file

    save_gait = True
    
    if save_gait:
        np.savetxt("Gait1.csv", 
                gait,
                delimiter =",")  # Set the delimiter as a comma followed by a space

    # From the gait coords, generate the angles

    Hip_pitch = []
    Knee = []
    Calf = []

    toe_coords = leg.segments[2].motor.connecting_rod_end
    heel_y_coords = toe_coords[1]

    for coords in gait:
            angle1, angle2 = inverse_kinematics(leg, coords)
            angle3 = keep_foot_flat(leg, heel_y_coords)
            # Adjusting virtual range to real servo range
            Hip_pitch.append(-r2d(angle1) + 180)                # Virtual: 150 - 90 where 150 is forward;  Real: 90 - 30 where 30 is forward
            Knee.append(-r2d(angle2) + 155)                     # Virtual: 120 - 55 where 120 is full-contract;  Real: 100 - 35 where 35 is full-contract
            Calf.append(-r2d(angle3) + 180)                     # Virtual: (0) 30 - 135 where 30 is toe up; Real: (60) 140 - 45 where 140 is toe up

    Gait_angle = np.column_stack((Hip_pitch,Knee, Calf))

    if save_gait:
        np.savetxt("Gait_angle1.csv", 
                Gait_angle,
                delimiter =",")  # Set the delimiter as a comma followed by a space