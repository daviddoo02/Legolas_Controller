import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple


def DH_Transform(th, alp, r, d):
    theta = np.deg2rad(th)
    alpha = np.deg2rad(alp)
    M = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(alpha) * np.sin(theta), r * np.cos(theta)],
                  [np.sin(theta), np.cos(alpha) * np.cos(theta), -
                   np.sin(alpha) * np.cos(theta), r * np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return M


def pol2cart(l: float, th: float):
    """
    Convert polar information to cartesian translation

    Parameters:
        l (float): linkage's length in mm
        th (float): angle in degrees

    Returns:
        Tuple[float, float]: Converted cartesian translation
    """

    th = np.deg2rad(th)
    x = l*np.cos(th)
    y = l*np.sin(th)

    return np.array([x, y], dtype=float)


def circle_equations(params: Tuple[float, float], center1: np.ndarray, r1: float, center2: np.ndarray, r2: float) -> Tuple[float, float]:
    """
    System of equations representing the circles.

    Parameters:
        params (Tuple[float, float]): Coordinates (x, y) of the point to evaluate.
        center1 (np.ndarray): Center of the first circle.
        r1 (float): Radius of the first circle.
        center2 (np.ndarray): Center of the second circle.
        r2 (float): Radius of the second circle.

    Returns:
        Tuple[float, float]: Residuals for the system of equations.
    """
    x, y = params
    eq1 = (x - center1[0])**2 + (y - center1[1])**2 - r1**2
    eq2 = (x - center2[0])**2 + (y - center2[1])**2 - r2**2
    return np.array([eq1, eq2], dtype=float)


def find_circle_intersection(center1: np.ndarray, r1: float, center2: np.ndarray, r2: float) -> np.ndarray:
    """
    Find the intersection points of two circles using the Levenberg-Marquardt algorithm.

    Parameters:
        center1 (np.ndarray): Center of the first circle.
        r1 (float): Radius of the first circle.
        center2 (np.ndarray): Center of the second circle.
        r2 (float): Radius of the second circle.

    Returns:
        np.ndarray: Coordinates of the intersection points.
    """

    # Assuming furthest to the right -> start guessing there
    rightmost_point = center2[0] - r2
    current = np.array([rightmost_point, center2[1]])

    # Use Levenberg-Marquardt to find the roots
    result = least_squares(circle_equations, current,
                           args=(center1, r1, center2, r2))

    return result.x


def angle_between_vectors(v1, v2):
    """
    Find the angle (in degrees) between two vectors.

    Parameters:
        v1 (numpy.ndarray): First vector.
        v2 (numpy.ndarray): Second vector.

    Returns:
        float: Angle between the two vectors in degrees.
    """
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    cosine_angle = dot_product / (norm_v1 * norm_v2)
    angle_in_radians = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    angle_in_degrees = np.degrees(angle_in_radians)

    # Calculate the cross product to determine the direction
    cross_product = np.cross(v1, v2)

    # If the cross product is positive --> clockwise
    if cross_product > 0:
        angle_in_degrees *= -1

    return angle_in_degrees

def plot_leg_single(joint_positions, ax):
    # Plotting the links

    # Scatter plot of joints
    ax.scatter(joint_positions[:, 0], joint_positions[:, 1],
               joint_positions[:, 2], c='b', marker='', label='Joints')

    j = 0
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    j = 1
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    j = 2
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    j = 3
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    j = 4
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    j = 5
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='')

    ax.plot([joint_positions[5, 0], joint_positions[7, 0]],
            [joint_positions[5, 1], joint_positions[7, 1]],
            [joint_positions[5, 2], joint_positions[7, 2]], color='b', linestyle='-', marker='')

    ax.plot([joint_positions[3, 0], joint_positions[6, 0]],
            [joint_positions[3, 1], joint_positions[6, 1]],
            [joint_positions[3, 2], joint_positions[6, 2]], color='g', linestyle='-', marker='')

    j = 8
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='r', linestyle='-', marker='')

    j = 9
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='g', linestyle='-', marker='')

    j = 10
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='r', linestyle='-', marker='')

    ax.plot([joint_positions[6, 0], joint_positions[8, 0]],
            [joint_positions[6, 1], joint_positions[8, 1]],
            [joint_positions[6, 2], joint_positions[8, 2]], color='gray', linestyle='-', marker='')

    ax.plot([joint_positions[8, 0], joint_positions[7, 0]],
            [joint_positions[8, 1], joint_positions[7, 1]],
            [joint_positions[8, 2], joint_positions[7, 2]], color='gray', linestyle='-', marker='')

    ax.plot([joint_positions[7, 0], joint_positions[11, 0]],
            [joint_positions[7, 1], joint_positions[11, 1]],
            [joint_positions[7, 2], joint_positions[11, 2]], color='r', linestyle='-', marker='')

    ax.plot([joint_positions[7, 0], joint_positions[10, 0]],
            [joint_positions[7, 1], joint_positions[10, 1]],
            [joint_positions[7, 2], joint_positions[10, 2]], color='r', linestyle='-', marker='')

    # Specific names for each point
    point_names = ['imu - 0', 'Hip 1 - 1', 'Hip 2 - 2',
                   'Thigh - 3', 'Foreleg - 4', '5', '6', 'Ankle - 7', 'Calf - 8', '9', '10', '11']

    # Label each point with specific names
    for i, txt in enumerate(joint_positions):
        ax.text(txt[0], txt[1], txt[2], point_names[i],
                color='red', fontsize=8)
    
    return


def plot_leg_config(left_joint_positions, right_joint_positions):
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plot_leg_single(left_joint_positions, ax)
    plot_leg_single(right_joint_positions, ax)

    # Setting labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set the view to isometric and rotate 180 degrees in the z-direction
    ax.view_init(elev=10, azim=70)

    # Make axis ticks equal
    ax.axis('equal')

    plt.show()
    return


class Leg:
    def __init__(self, left=True):
        # # Define angle limits in degrees
        
        self.angle2_min, self.angle2_max = -20, 20
        self.angle3_min, self.angle3_max = -45, 65      # -65, 45
        self.angle4_min, self.angle4_max = 0, 95        # -95, 0
        self.angle5_min, self.angle5_max = -45, 60      # -60, 45
        
        self.left = left

        if self.left:
            self.y01 = 104       # y distance to imu
            self.angle1_min, self.angle1_max = -15, 5
            
        else:
            self.y01 = -104
            self.angle1_min, self.angle1_max = -5, 15

    def forward_kinematics(self, hip1, hip2, thigh, foreleg, calf):

        # Initialize values for the joint angles (degrees)
        if self.left:
            th_1 = hip1
            th_2 = -hip2
        else:
            th_1 = -hip1
            th_2 = hip2

        th_3 = -thigh + 65
        th_4 = -foreleg + 135
        th_5 = -calf - 15

        # Link lengths in centimeters -- From CAD
        z01 = 67.74         # z distance to imu
        x01 = 5.09          # x distance to imu
        y01 = self.y01      # y distance to imu

        l1 = 64.35      # Length of hip 1
        l2 = 47.5       # Length of hip 2
        l3 = 55.34      # Length of thigh
        l4 = 105        # Length of foreleg
        l5 = 105        # Length of shin
        l6 = 105        # Length of linkage 1
        l7 = 55.34      # Length of shin nub

        l8 = 25         # Offset from shin center to calf motor
        l9 = 24.5       # Calf servo arm length
        l10 = 108.73    # Length of linkage 2
        l11 = 24.50     # Length of foot hypothenus
        l12 = 36.21     # Length of foot

        # Calculating everything from the thigh down wrt the thigh's coordinate frame

        # < First closed chain:

        j3 = np.array([0, 0])                           # Thigh
        j4 = j3 + pol2cart(l3, th_3)                    # Foreleg
        j5 = j4 + pol2cart(l4, th_3 + th_4)             # Shin center

        # Closing the loop, where shin nub meets linkage:
        j7 = find_circle_intersection(j3, l6, j5, l7)

        th_s = 180 + np.rad2deg(np.arctan2(j7[1] - j5[1], j7[0] - j5[0]))

        # end-effector position:
        j6 = j5 + pol2cart(l5, th_s)

        # End of first closed chain >

        # ---------------------------------------

        # < Second closed chain:

        j8 = j5 + pol2cart(l8, th_s + 91.87)
        j9 = j8 + pol2cart(l9, th_s + 91.87 + th_5)

        # Closing the loop, where linkage 2 meets foot
        j10 = find_circle_intersection(j9, l10, j6, l11)

        th_f = 180 + np.rad2deg(np.arctan2(j10[1] - j9[1], j10[0] - j9[0])
                                ) + angle_between_vectors(j6-j10, j9-j10) + 42.35

        j11 = j10 + pol2cart(l12, th_f)

        # End of second closed chain >

        # Now package them neatly ;)

        j3_6 = np.array([j3, j4, j5, j7, j6, j8, j9, j10, j11]).round(4)
        j3_6 = np.hstack([j3_6, np.zeros((j3_6.shape[0], 1)),
                          np.ones((j3_6.shape[0], 1))])

        # ---------------------------------------

        # Now calculate the rest:

        # D_H Table from the imu to thigh
        d_h_table = np.array([[-90, 90, -y01, -z01],
                              [0 + th_1, 90, 0, l1],
                              [-90 + th_2, 90, 0, l2]])

        joint_positions = [[0, 0, 0]]     # Position of IMU
        m00 = np.eye(4)         # Just identity matrix

        # Go through each rows of dh table
        i = 0
        m01 = DH_Transform(d_h_table[i, 0], d_h_table[i, 1],
                           d_h_table[i, 2], d_h_table[i, 3])
        m01 = m00 @ m01
        joint_position = m01[:3, 3].round(4)
        joint_position[0] += x01        # x offset
        joint_positions.append(joint_position)

        i = 1
        m12 = DH_Transform(d_h_table[i, 0], d_h_table[i, 1],
                           d_h_table[i, 2], d_h_table[i, 3])
        m02 = m00 @ m01 @ m12
        joint_position = m02[:3, 3].round(4)
        joint_positions.append(joint_position)

        i = 2
        m23 = DH_Transform(d_h_table[i, 0], d_h_table[i, 1],
                           d_h_table[i, 2], d_h_table[i, 3])
        m03 = m00 @ m01 @ m12 @ m23
        joint_position = m03[:3, 3].round(4)
        joint_positions.append(joint_position)

        p3_6 = ((m03 @ j3_6.T).T)[:, :3]

        joint_positions = np.concatenate((joint_positions, p3_6[1:]), axis=0)

        return joint_positions

    def clamp_angle(self, angle, min_angle, max_angle):
        """
        Helper function to clamp an angle within a specified range.
        """
        return max(min(angle, max_angle), min_angle)

    def inverse_kinematics(self, desired: np.ndarray, angles_guess: Tuple[float, float], max_iterations: int = 100) -> Tuple[float, float]:
        """
        Perform inverse kinematics using gradient descent.

        Parameters:
            desired (np.ndarray): Target end-effector position [x, y].
            angles_guess (Tuple[float, float]): current joint angles.
            max_iterations (int): Maximum number of iterations for gradient descent.
            learning_rate (float): Learning rate for gradient descent.

        Returns:
            Tuple[float, float]: Joint angles (th1, th2).
        """

        def error_function(desired_pos: np.ndarray, current: np.ndarray) -> float:
            th1, th2, th3, th4, th5 = current
            current_pos = self.forward_kinematics(th1, th2, th3, th4, th5)[7]
            error = np.linalg.norm(current_pos - desired_pos)
            return error

        def error_function_foot(current: np.ndarray) -> float:
            th1, th2, th3, th4, th5 = current
            current_pos = self.forward_kinematics(
                th1, th2, th3, th4, th5)[11][2]
            desired_pos = self.forward_kinematics(
                th1, th2, th3, th4, th5)[10][2]
            error = np.linalg.norm(current_pos - desired_pos)
            return error

        angle1, angle2, angle3, angle4, angle5 = angles_guess

        step1 = 0.1
        step2 = 0.1
        step3 = 0.1
        step4 = 0.1
        step5 = 0.1

        learning_rate = 0.05
        learning_rate_foot = 0.1

        tolerance = 0.5  # stop when the error is less than this or when weve done max_iterations
        iter = 0

        for _ in range(max_iterations):
            # error is the distance between the desired point and our current point. We want to minimize this.
            error = error_function(desired, np.array(
                [angle1, angle2, angle3, angle4, angle5]))

            error_foot = error_function_foot(np.array(
                [angle1, angle2, angle3, angle4, angle5]))

            # step in a direction to find the error of the 4 angles
            error_angle1 = error_function(desired, np.array(
                [angle1+step1,  angle2,         angle3,         angle4,             angle5]))
            error_angle2 = error_function(desired, np.array(
                [angle1,        angle2+step2,   angle3,         angle4,             angle5]))
            error_angle3 = error_function(desired, np.array(
                [angle1,        angle2,         angle3+step3,   angle4,             angle5]))
            error_angle4 = error_function(desired, np.array(
                [angle1,        angle2,         angle3,         angle4+step4,       angle5]))

            error_angle5 = error_function_foot(np.array(
                [angle1,        angle2,         angle3,         angle4,             angle5+step5]))

            # this is the gradient
            grad1 = (error_angle1 - error) / step1
            grad2 = (error_angle2 - error) / step2
            grad3 = (error_angle3 - error) / step3
            grad4 = (error_angle4 - error) / step4

            grad5 = (error_angle5 - error_foot) / step5

            # step in the direction of the gradient. grad is the slope (rate of change), error is the distance, so in theory grad*error is the distance we want to step to get to the desired point in one step, but lmao no, multiply by really small fraction. 0.00001 is the largest that doesnt overshoot.
            angle1 -= grad1 * error * learning_rate
            angle2 -= grad2 * error * learning_rate
            angle3 -= grad3 * error * learning_rate
            angle4 -= grad4 * error * learning_rate

            angle5 -= grad5 * error_foot * learning_rate_foot

            # Adjust to be in range
            angle1 = self.clamp_angle(angle1, self.angle1_min, self.angle1_max)
            angle2 = self.clamp_angle(angle2, self.angle2_min, self.angle2_max)
            angle3 = self.clamp_angle(angle3, self.angle3_min, self.angle3_max)
            angle4 = self.clamp_angle(angle4, self.angle4_min, self.angle4_max)
            angle5 = self.clamp_angle(angle5, self.angle5_min, self.angle5_max)

            iter += 1
            # print(iter)

            # every other iteration, step in the opposite direction. When the leg is up against 2 of the 4 walls, the gradient descent will get stuck trying to step into the wall and getting a gradient of 0.
            step1 *= -1
            step2 *= -1
            step3 *= -1
            step4 *= -1
            step5 *= -1

            print("Leg error: ", error)
            print("Foot error: ", error_foot)

            if (abs(error) < tolerance) and (abs(error_foot) < tolerance):
                print("")
                print("Breaking")
                print("")
                print("Leg error: ", error)
                print("Foot error: ", error_foot)
                break

        return angle1, angle2, angle3, angle4, angle5
