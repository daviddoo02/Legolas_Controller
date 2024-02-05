import sympy as sp
import numpy as np
import numpy as np
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d   
from sympy.physics.mechanics import dynamicsymbols

def leg_config():
    DH = pd.read_excel("DH-Parameters.xlsx", 
                    dtype = {"theta (deg)": float,
                                "alpha (deg)":float,
                                "r (mm)": float,
                                "d (mm)": float})

    theta_dic = DH["theta (deg)"]
    alpha_dic = DH["alpha (deg)"]
    r_dic = DH["r (mm)"]
    d_dic = DH["d (mm)"]

    d2r = np.deg2rad

    theta, alpha, r, d = dynamicsymbols('theta alpha r d')

    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = dynamicsymbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7')

    alpha1 = d2r(alpha_dic[0])
    alpha2 = d2r(alpha_dic[1])
    alpha3 = d2r(alpha_dic[2])
    alpha4 = d2r(alpha_dic[3])
    alpha5 = d2r(alpha_dic[4])
    alpha6 = d2r(alpha_dic[5])
    alpha7 = d2r(alpha_dic[6])

    r1 = r_dic[0]
    r2 = r_dic[1]
    r3 = r_dic[2]
    r4 = r_dic[3]
    r5 = r_dic[4]
    r6 = r_dic[5]
    r7 = r_dic[6]

    d1 = d_dic[0]
    d2 = d_dic[1]
    d3 = d_dic[2]
    d4 = d_dic[3]
    d5 = d_dic[4]
    d6 = d_dic[5]
    d7 = d_dic[6]

    # FIXED Angles
    theta6s = d2r(theta_dic[5])

    # VARIABLE Angles
    theta1s = d2r(theta_dic[0])                # Hip 1
    theta2s = d2r(theta_dic[1])                # Hip 2
    theta3s = d2r(theta_dic[2])                # Thigh
    theta4s = d2r(theta_dic[3])                # Foreleg
    theta5s = d2r(theta_dic[4])                # Knee
    theta7s = d2r(theta_dic[6])                # Feet

    xs = [0]
    ys = [0]
    zs = [0]


    rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha)],
                    [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha)],
                    [0, sp.sin(alpha), sp.cos(alpha)]])

    trans = sp.Matrix([r*sp.cos(theta), r*sp.sin(theta),d])

    last_row = sp.Matrix([[0, 0, 0, 1]])

    m = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)


    # m01 = m.subs({alpha:alpha1, r:r1, theta:theta1, d:d1})

    # m12 = m.subs({alpha:alpha2, r:r2, theta:theta2, d:d2})

    m23 = m.subs({alpha:alpha3, r:r3, theta:theta3, d:d3})

    m34 = m.subs({alpha:alpha4, r:r4, theta:theta4, d:d4})

    m45 = m.subs({alpha:alpha5, r:r5, theta:theta5, d:d5})

    m56 = m.subs({alpha:alpha6, r:r6, theta:theta6, d:d6})

    tf = [m23, m34, m45, m56]

    m_prev = np.identity(4)

    for m in tf:
        i = m_prev * m
        px = i[0,3]
        py = i[1,3]
        pz = i[2,3]

        # Lambdify the sympy expression into numpy function:
        fx = sp.lambdify((theta1, theta2, theta3, theta4, theta5, theta6, theta7), px, 'numpy')
        fy = sp.lambdify((theta1, theta2, theta3, theta4, theta5, theta6, theta7), py, 'numpy')
        fz = sp.lambdify((theta1, theta2, theta3, theta4, theta5, theta6, theta7), pz, 'numpy')

        # Tip position in relative to frame 0

        x = fx(theta1s, theta2s, theta3s, theta4s, theta5s, theta6s, theta7s)
        y = fy(theta1s, theta2s, theta3s, theta4s, theta5s, theta6s, theta7s)
        z = fz(theta1s, theta2s, theta3s, theta4s, theta5s, theta6s, theta7s)

        xs.append(x)
        ys.append(y)
        zs.append(z)

        m_prev = i

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)

    return xs, ys, zs


# plt.close()
 
# fig = plt.figure()
 
# # syntax for 3-D projection
# ax = fig.add_subplot(111, projection='3d')
# ax.set_aspect('equal')

# xs = np.array(xs)
# ys = np.array(ys)
# zs = np.array(zs)

# # Fake bounding box to equalize aspect ratio
# # ---------------------------------------------------------------------------------------------------- #

# max_range = np.array([xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min()]).max()
# Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(xs.max()+xs.min())
# Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(ys.max()+ys.min())
# Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(zs.max()+zs.min())
# # Comment or uncomment following both lines to test the fake bounding box:
# for xb, yb, zb in zip(Xb, Yb, Zb):
#    ax.plot([xb], [zb], [yb], 'w')

# # ---------------------------------------------------------------------------------------------------- #
 
# # plotting
# ax.plot3D(xs, zs, ys, 'green')
# ax.set_title('')

# plt.grid()
# plt.show()