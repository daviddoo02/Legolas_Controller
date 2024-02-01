import numpy as np
from scipy.signal import savgol_filter


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