import numpy as np


# First closed chain

# Link lengths in centimeters
a1 = 64.35  # Length of hip 1
a2 = 47.5   # Length of hip 2
a3 = 55.34  # Length of thigh
a4 = 105    # Length of foreleg
a5 = 105    # Length of shin

a6 = 55.34  # Length of shin nub
a7 = 105    # Length of linkage 1

# Initialize values for the joint angles (degrees)
theta_1 = 0  # Joint 1
theta_2 = 0  # Joint 2

d2r = np.deg2rad

d_h_table = np.array([[d2r(theta_1), d2r(-90), 0, a1],
                      [d2r(theta_2), 0, a2, ]])


def DH_Transform(theta, alpha, a, d):
    M = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(alpha) * np.sin(theta), a * np.cos(theta)],
                  [np.sin(theta), np.cos(alpha) * np.cos(theta), -
                   np.sin(alpha) * np.cos(theta), a * np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return M
