import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from mpl_toolkits.mplot3d import Axes3D
from mplcursors import cursor as mpl_cursor
from scipy.signal import savgol_filter
from matplotlib.widgets import Slider
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


def find_circle_intersection(center1: np.ndarray, r1: float, center2: np.ndarray, r2: float, shin=True) -> np.ndarray:
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


def forward_kinematics(hip1, hip2, thigh, foreleg, calf):

    # Link lengths in centimeters -- From CAD
    y01 = 104       # y distance to imu
    z01 = 67.74     # z distance to imu
    x01 = 5.09      # x distance to imu

    # Initialize values for the joint angles (degrees)
    th_1 = hip1
    th_2 = hip2
    th_3 = thigh + 60
    th_4 = foreleg + 118.66

    # Calculating everything from the thigh down wrt the thigh's coordinate frame

    # < First closed chain:

    l1 = 64.35      # Length of hip 1
    l2 = 47.5       # Length of hip 2
    l3 = 55.34      # Length of thigh
    l4 = 105        # Length of foreleg
    l5 = 105        # Length of shin
    l6 = 105        # Length of linkage 1
    l7 = 55.34      # Length of shin nub

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

    l8 = 25         # Offset from shin center to calf motor
    l9 = 24.5       # Calf servo arm length
    l10 = 108.73    # Length of linkage 2
    l11 = 24.50     # Length of foot hypothenus
    l12 = 36.21     # Length of foot

    th_5 = calf

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


def plot_leg_config(joint_positions):
    # Plotting the manipulator configuration
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the links

    # Scatter plot of joints
    ax.scatter(joint_positions[:, 0], joint_positions[:, 1],
               joint_positions[:, 2], c='b', marker='o', label='Joints')

    j = 0
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    j = 1
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    j = 2
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    j = 3
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    j = 4
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    j = 5
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    ax.plot([joint_positions[5, 0], joint_positions[7, 0]],
            [joint_positions[5, 1], joint_positions[7, 1]],
            [joint_positions[5, 2], joint_positions[7, 2]], color='b', linestyle='-', marker='o')

    ax.plot([joint_positions[3, 0], joint_positions[6, 0]],
            [joint_positions[3, 1], joint_positions[6, 1]],
            [joint_positions[3, 2], joint_positions[6, 2]], color='g', linestyle='-', marker='o')

    j = 8
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='r', linestyle='-', marker='o')

    j = 9
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='g', linestyle='-', marker='o')

    j = 10
    ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
            [joint_positions[j, 1], joint_positions[j + 1, 1]],
            [joint_positions[j, 2], joint_positions[j + 1, 2]], color='r', linestyle='-', marker='o')

    ax.plot([joint_positions[6, 0], joint_positions[8, 0]],
            [joint_positions[6, 1], joint_positions[8, 1]],
            [joint_positions[6, 2], joint_positions[8, 2]], color='gray', linestyle='-', marker='o')

    ax.plot([joint_positions[8, 0], joint_positions[7, 0]],
            [joint_positions[8, 1], joint_positions[7, 1]],
            [joint_positions[8, 2], joint_positions[7, 2]], color='gray', linestyle='-', marker='o')

    ax.plot([joint_positions[7, 0], joint_positions[11, 0]],
            [joint_positions[7, 1], joint_positions[11, 1]],
            [joint_positions[7, 2], joint_positions[11, 2]], color='r', linestyle='-', marker='o')

    ax.plot([joint_positions[7, 0], joint_positions[10, 0]],
            [joint_positions[7, 1], joint_positions[10, 1]],
            [joint_positions[7, 2], joint_positions[10, 2]], color='r', linestyle='-', marker='o')

    # Specific names for each point
    point_names = ['imu - 0', 'Hip 1 - 1', 'Hip 2 - 2',
                   'Thigh - 3', 'Foreleg - 4', '5', '6', 'Ankle - 7', 'Calf - 8', '9', '10', '11']

    # Label each point with specific names
    for i, txt in enumerate(joint_positions):
        ax.text(txt[0], txt[1], txt[2], point_names[i],
                color='red', fontsize=8)

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


def plot_leg_config_sliders(joint_positions):
    # Plotting the manipulator configuration
    plt.close()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the links

    # Scatter plot of joints
    scatter = ax.scatter(joint_positions[:, 0], joint_positions[:, 1],
                         joint_positions[:, 2], c='b', marker='o', label='Joints')

    for j in range(len(joint_positions)-1):
        ax.plot([joint_positions[j, 0], joint_positions[j + 1, 0]],
                [joint_positions[j, 1], joint_positions[j + 1, 1]],
                [joint_positions[j, 2], joint_positions[j + 1, 2]], color='b', linestyle='-', marker='o')

    ax.plot([joint_positions[5, 0], joint_positions[7, 0]],
            [joint_positions[5, 1], joint_positions[7, 1]],
            [joint_positions[5, 2], joint_positions[7, 2]], color='b', linestyle='-', marker='o')

    ax.plot([joint_positions[3, 0], joint_positions[6, 0]],
            [joint_positions[3, 1], joint_positions[6, 1]],
            [joint_positions[3, 2], joint_positions[6, 2]], color='g', linestyle='-', marker='o')

    # Specific names for each point
    point_names = ['imu - 0', 'Hip 1 - 1', 'Hip 2 - 2',
                   'Thigh - 3', 'Foreleg - 4', '5', '6', 'Foot - 7']

    # Label each point with specific names
    for i, txt in enumerate(joint_positions):
        ax.text(txt[0], txt[1], txt[2], point_names[i],
                color='red', fontsize=8)

    # Add sliders for adjusting a point's coordinates
    slider_ax_x = plt.axes([0.9, 0.65, 0.03, 0.25],
                           facecolor='lightgoldenrodyellow')
    slider_ax_y = plt.axes([0.92, 0.65, 0.03, 0.25],
                           facecolor='lightgoldenrodyellow')
    slider_ax_z = plt.axes([0.94, 0.65, 0.03, 0.25],
                           facecolor='lightgoldenrodyellow')

    slider_x = Slider(
        slider_ax_x, 'X', joint_positions[0, 0], joint_positions[-1, 0], valinit=joint_positions[0, 0])
    slider_y = Slider(
        slider_ax_y, 'Y', joint_positions[0, 1], joint_positions[-1, 1], valinit=joint_positions[0, 1])
    slider_z = Slider(
        slider_ax_z, 'Z', joint_positions[0, 2], joint_positions[-1, 2], valinit=joint_positions[0, 2])

    def update(val):
        joint_positions[0, 0] = slider_x.val
        joint_positions[0, 1] = slider_y.val
        joint_positions[0, 2] = slider_z.val
        scatter.set_offsets(joint_positions[:, :2])
        scatter.set_3d_properties(joint_positions[:, 2])

    slider_x.on_changed(update)
    slider_y.on_changed(update)
    slider_z.on_changed(update)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.view_init(elev=10, azim=70)

    ax.axis('equal')

    plt.show()
    return


def clamp_angle(angle, min_angle, max_angle):
    """
    Helper function to clamp an angle within a specified range.
    """
    return max(min(angle, max_angle), min_angle)

# Gradient Descent


def error_function(desired_pos: np.ndarray, current: np.ndarray) -> float:
    th1, th2, th3, th4, th5 = current
    current_pos = forward_kinematics(th1, th2, th3, th4, th5)[7]
    error = np.linalg.norm(current_pos - desired_pos)
    # print(error)
    return error


def inverse_kinematics(desired: np.ndarray, angles_guess: Tuple[float, float], max_iterations: int = 100, learning_rate: int = 0.1) -> Tuple[float, float]:
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
    angle1, angle2, angle3, angle4, _ = angles_guess

    step1 = 0.1
    step2 = 0.1
    step3 = 0.1
    step4 = 0.1

    tolerance = 0.01  # stop when the error is less than this or when weve done max_iterations
    iter = 0

    # # Define angle limits in degrees
    # angle1_min, angle1_max = -180.0, 180.0
    # angle2_min, angle2_max = -180.0, 180.0
    # angle3_min, angle3_max = -180.0, 180.0
    # angle4_min, angle4_max = -180.0, 180.0

    for _ in range(max_iterations):
        # error is the distance between the desired point and our current point. We want to minimize this.
        error = error_function(desired, np.array(
            [angle1, angle2, angle3, angle4, _]))

        # step in a direction to find the error of the 4 angles
        error_angle1 = error_function(desired, np.array(
            [angle1+step1,  angle2,         angle3,         angle4, _]))
        error_angle2 = error_function(desired, np.array(
            [angle1,        angle2+step2,   angle3,         angle4, _]))
        error_angle3 = error_function(desired, np.array(
            [angle1,        angle2,         angle3+step3,   angle4, _]))
        error_angle4 = error_function(desired, np.array(
            [angle1,        angle2,         angle3,         angle4+step4, _]))

        # this is the gradient
        grad1 = (error_angle1 - error) / step1
        grad2 = (error_angle2 - error) / step2
        grad3 = (error_angle3 - error) / step3
        grad4 = (error_angle4 - error) / step4

        # step in the direction of the gradient. grad is the slope (rate of change), error is the distance, so in theory grad*error is the distance we want to step to get to the desired point in one step, but lmao no, multiply by really small fraction. 0.00001 is the largest that doesnt overshoot.
        # angle1 = clamp_angle(angle1 - grad1 * error * learning_rate, angle1_min, angle1_max)
        # angle2 = clamp_angle(angle2 - grad2 * error * learning_rate, angle2_min, angle2_max)
        # angle3 = clamp_angle(angle3 - grad3 * error * learning_rate, angle3_min, angle3_max)
        # angle4 = clamp_angle(angle4 - grad4 * error * learning_rate, angle4_min, angle4_max)

        angle1 -= grad1 * error * learning_rate
        angle2 -= grad2 * error * learning_rate
        angle3 -= grad3 * error * learning_rate
        angle4 -= grad4 * error * learning_rate

        iter += 1

        # print(angle1, angle2, angle3, angle4)
        # print(grad1, grad2, grad3, grad4)
        # print(iter)

        # every other iteration, step in the opposite direction. When the leg is up against 2 of the 4 walls, the gradient descent will get stuck trying to step into the wall and getting a gradient of 0.
        step1 *= -1
        step2 *= -1
        step3 *= -1
        step4 *= -1

        if abs(error) < tolerance:
            break

    return angle1, angle2, angle3, angle4


def unstack(a, axis=0):
    """
    Handy function to unstack arrays, a is for array
    """
    return np.moveaxis(a, axis, 0)


def trapezoid_gait(h=125, bba=230, bbb=150, tbo=60, gbo=-360, c=-30):
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

    coords = np.stack((x, y), axis=1)
    return coords


def generate_gait(gait_key_features, element=20):
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
