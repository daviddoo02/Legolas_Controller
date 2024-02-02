from kinematics import Leg, plot_leg_config
import numpy as np

# Input
th_1 = -3  # Hip 1
th_2 = 0.5  # Hip 2
th_3 = 1.7  # Thigh
th_4 = -75  # Foreleg
th_5 = -5  # Calf

desire = np.array([-64.35, 120, -346])

guess = np.array([th_1, th_2, th_3, th_4, th_5])

left_leg = Leg()
# right_leg = Leg(False)

nth_1, nth_2, nth_3, nth_4, nth_5 = left_leg.inverse_kinematics(desire, guess)
print(nth_1, nth_2, nth_3, nth_4, nth_5)
joints = left_leg.forward_kinematics(nth_1, nth_2, nth_3, nth_4, nth_5)

# joints = left_leg.forward_kinematics(th_1, th_2, th_3, th_4, th_5)
print(joints)

plot_leg_config(joints)
