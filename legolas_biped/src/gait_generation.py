"""
Legolas' gait generator
"""
import csv
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics import Leg, plot_leg_config

__authors__ = "David Ho"
__license__ = "BSD-3-Clause"


class Gait():
    def __init__(self, left=True):
        if left:
            self.left = True
        else:
            self.left = False

    def trapezoid_gait(self):
        """
        Function to generate the key features of a trapezoidal gait

        Outputs:
            A 2D array containing the coordinates (x,y) of the trapezoidal gait
        """

        h = 50         # height of trapezoid
        bba = 50	    # bottom width front offset
        bbb = 50       # bottom width back offset
        tbo = 30        # top width offset

        # Don't change these 2 unless you know what you're doing
        gbo = -350      # gait base offset
        c = -64.35         # center of trapezoid

        x = np.array([c,     c + bba,    c + tbo,
                     c - tbo,    c - bbb,    c])
        z = np.array([gbo,   gbo,        h + gbo,
                     h + gbo,    gbo,        gbo])
        y = np.array([115,   115,        105,
                     105,        115,        115])

        # Set the timing of the gait for each leg:

        if not self.left:
            y *= -1
        return x, y, z

    def generate_gait(self, element=20):
        x, y, z = self.trapezoid_gait()

        gait_x = []
        gait_y = []
        gait_z = []

        # Iterate through each pair of consecutive key points
        for i in range(len(x) - 1):
            start_point_x = x[i]
            end_point_x = x[i + 1]
            start_point_y = y[i]
            end_point_y = y[i + 1]
            start_point_z = z[i]
            end_point_z = z[i + 1]

            # Linear interpolation between the current pair of key points for each dimension
            interpolated_x = np.linspace(
                start_point_x, end_point_x, element + 2)[1:-1]
            interpolated_y = np.linspace(
                start_point_y, end_point_y, element + 2)[1:-1]
            interpolated_z = np.linspace(
                start_point_z, end_point_z, element + 2)[1:-1]

            # Append the interpolated points to the extrapolated arrays
            gait_x.extend(interpolated_x)
            gait_y.extend(interpolated_y)
            gait_z.extend(interpolated_z)

        # Convert the lists to NumPy arrays
        gait_x = np.array(gait_x)
        gait_y = np.array(gait_y)
        gait_z = np.array(gait_z)

        # Apply Savitzky-Golay filter for smoothing
        smoothed_gait_x = savgol_filter(gait_x, window_length=11, polyorder=2)
        smoothed_gait_y = savgol_filter(gait_y, window_length=11, polyorder=2)
        smoothed_gait_z = savgol_filter(gait_z, window_length=11, polyorder=2)

        # Rearrange the gait:
        if self.left:
            start_element = element
        else:
            start_element = - element

        # Rearrange the gait_coordinates to start at the specified element
        smoothed_gait_x = np.concatenate(
            (smoothed_gait_x[start_element:], smoothed_gait_x[:start_element]))
        smoothed_gait_y = np.concatenate(
            (smoothed_gait_y[start_element:], smoothed_gait_y[:start_element]))
        smoothed_gait_z = np.concatenate(
            (smoothed_gait_z[start_element:], smoothed_gait_z[:start_element]))

        return smoothed_gait_x, smoothed_gait_y, smoothed_gait_z


def plot_gait(xl, yl, zl, xr, yr, zr):
    plt.close()
    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(xl, yl, zl)
    ax.scatter(xr, yr, zr)

    # Set labels for each axis
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    ax.axis('equal')
    return


def main():
    # For each legs
    lth_1 = -3.2  # Hip 1
    lth_2 = -0.5  # Hip 2
    lth_3 = -3.1  # Thigh
    lth_4 = 78.5  # Foreleg
    lth_5 = 7.4  # Calf

    rth_1 = -3.2  # Hip 1
    rth_2 = -0.5  # Hip 2
    rth_3 = -3.1  # Thigh
    rth_4 = 78.5  # Foreleg
    rth_5 = 7.4  # Calf

    left_gait = Gait()
    right_gait = Gait(False)

    left_leg = Leg()
    right_leg = Leg(False)

    xl, yl, zl = left_gait.generate_gait(10)
    xr, yr, zr = right_gait.generate_gait(10)

    # plot_gait(xl, yl, zl, xr, yr, zr)
    # plt.show()

    # Initialize empty arrays to store joint angles
    left_leg_joint_angles = []
    right_leg_joint_angles = []

    for i in range(len(xl)):
        # Get the gait points for the current step
        left_gait_point = np.array([xl[i], yl[i], zl[i]])
        right_gait_point = np.array([xr[i], yr[i], zr[i]])

        # Get the current guess for joint angles
        current_guess_l = np.array([lth_1, lth_2, lth_3, lth_4, lth_5])
        current_guess_r = np.array([rth_1, rth_2, rth_3, rth_4, rth_5])

        # Calculate inverse kinematics for both legs
        lth_1, lth_2, lth_3, lth_4, lth_5 = left_leg.inverse_kinematics(
            left_gait_point, current_guess_l)
        rth_1, rth_2, rth_3, rth_4, rth_5 = right_leg.inverse_kinematics(
            right_gait_point, current_guess_r)

        # Append the joint angles for the current step to the arrays
        left_leg_joint_angles.append([lth_1, lth_2, lth_3, lth_4, lth_5])
        right_leg_joint_angles.append([rth_1, rth_2, rth_3, rth_4, rth_5])
        print(left_leg_joint_angles[i])

    # Specify the file path for the CSV file
    csv_file_path = 'gait_joint_angles.csv'

    # Open the CSV file in write mode
    with open(csv_file_path, mode='w', newline='') as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)

        # Write the joint angles for each step
        for i in range(len(xl)):
            # Get the joint angles for the current step
            left_joint_angles = left_leg_joint_angles[i]
            right_joint_angles = right_leg_joint_angles[i]

            # Write the data to the CSV file
            csv_writer.writerow(left_joint_angles + right_joint_angles)

    print(
        f'Joint angles data has been successfully written to {csv_file_path}')

    # desire_left = np.array([-64.35, 120, -350])
    # desire_right = np.array([-64.35, -120, -350])

    # guess = np.array([lth_1, lth_2, lth_3, lth_4, lth_5])

    # lth_1, lth_2, lth_3, lth_4, lth_5 = left_leg.inverse_kinematics(desire_left, guess)
    # rth_1, rth_2, rth_3, rth_4, rth_5 = right_leg.inverse_kinematics(desire_right, guess)

    # left_joints = left_leg.forward_kinematics(lth_1, lth_2, lth_3, lth_4, lth_5)
    # right_joints = right_leg.forward_kinematics(rth_1, rth_2, rth_3, rth_4, rth_5)

    # print(lth_1, lth_2, lth_3, lth_4, lth_5)
    # print(left_joints[7])
    # print()
    # print(rth_1, rth_2, rth_3, rth_4, rth_5)
    # print(right_joints[7])

    # plot_leg_config(left_joints, right_joints)

    return


if __name__ == '__main__':
    main()
