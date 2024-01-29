from kinematics import forward_kinematics, plot_leg_config, inverse_kinematics
import numpy as np


# Input
th_1 = 0  # Hip 1
th_2 = 0  # Hip 2
th_3 = 0  # Thigh
th_4 = 0  # Foreleg

desire = np.array([-50, 104, -255])

guess = np.array([th_1, th_2, th_3, th_4])

nth_1, nth_2, nth_3, nth_4 = inverse_kinematics(desire, guess)

joints = forward_kinematics(nth_1, nth_2, nth_3, nth_4)
print(joints)

plot_leg_config(joints)