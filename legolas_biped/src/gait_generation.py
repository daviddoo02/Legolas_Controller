"""
Legolas' gait generator
"""
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

__authors__ = "David Ho"
__license__ = "BSD-3-Clause"


class Gait():
    def __init__(self, left=True):
        if left:
            self.left = 1
        else:
            self.left = -1
    
    def trapezoid_gait(self):
        """
        Function to generate the key features of a trapezoidal gait

        Outputs:
            A 2D array containing the coordinates (x,y) of the trapezoidal gait
        """

        h = 50         # height of trapezoid
        bba = 50	    # bottom width back offset
        bbb = 50       # bottom width front offset
        tbo = 30        # top width offset

        # Don't change these 2 unless you know what you're doing
        gbo = -350      # gait base offset
        c = -64.35         # center of trapezoid
        
        x = np.array([c,     c + bba,    c + tbo,    c - tbo,    c - bbb,    c])
        z = np.array([gbo,   gbo,        h + gbo,    h + gbo,    gbo,        gbo])
        y = np.array([115,   115,        105,        105,        115,        115])

        return x, y, z
        
    def generate_gait(self, element=20):
        x, y, z = self.trapezoid_gait()

        gait_x = []
        gait_y = []
        gait_z = []

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

            if z[index] < z[index + 1]:
                k = np.linspace(z[index], z[index + 1], element)
            else:
                k = np.linspace(z[index + 1], z[index], element)
                k = k[::-1]

            gait_x = np.concatenate((gait_x, i), axis=None)
            gait_y = np.concatenate((gait_y, j), axis=None)
            gait_z = np.concatenate((gait_z, k), axis=None) 

        # using a filter to smooth out the data
        win_size = element
        poly = 1

        gait_x = savgol_filter(gait_x, win_size, poly)
        gait_y = savgol_filter(gait_y, win_size, poly) * self.left
        gait_z = savgol_filter(gait_z, win_size, poly)

        return gait_x, gait_y, gait_z


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

# def main():
#     left_gait = Gait()
#     right_gait = Gait(False)

#     xl, yl, zl = left_gait.generate_gait(5)
#     xr, yr, zr = right_gait.generate_gait(5)

#     plot_gait(xl, yl, zl, xr, yr, zr)
#     plt.show()
#     return

# if __name__ == '__main__':
#     main()
