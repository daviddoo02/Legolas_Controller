from kinematics import Leg, plot_leg_config
import numpy as np

# Input
th_1 = 0  # Hip 1
th_2 = 0  # Hip 2
th_3 = 0  # Thigh
th_4 = 0  # Foreleg
th_5 = 0  # Calf

# For each legs
# lth_1 = 0  # Hip 1
# lth_2 = 0  # Hip 2
# lth_3 = 0  # Thigh
# lth_4 = 0  # Foreleg
# lth_5 = 0  # Calf

# rth_1 = 0  # Hip 1
# rth_2 = 0  # Hip 2
# rth_3 = 0  # Thigh
# rth_4 = 0  # Foreleg
# rth_5 = 0  # Calf

desire_left = np.array([-64.35, 120, -350])
desire_right = np.array([-64.35, -120, -350])

guess = np.array([th_1, th_2, th_3, th_4, th_5])

left_leg = Leg()
right_leg = Leg(False)

lth_1, lth_2, lth_3, lth_4, lth_5 = left_leg.inverse_kinematics(desire_left, guess)
rth_1, rth_2, rth_3, rth_4, rth_5 = right_leg.inverse_kinematics(desire_right, guess)

left_joints = left_leg.forward_kinematics(lth_1, lth_2, lth_3, lth_4, lth_5)
right_joints = right_leg.forward_kinematics(rth_1, rth_2, rth_3, rth_4, rth_5)

# left_joints = left_leg.forward_kinematics(lth_1, lth_2, lth_3, lth_4, lth_5)
# right_joints = right_leg.forward_kinematics(rth_1, rth_2, rth_3, rth_4, rth_5)
print(lth_1, lth_2, lth_3, lth_4, lth_5)
print(left_joints[7])

plot_leg_config(left_joints, right_joints)
