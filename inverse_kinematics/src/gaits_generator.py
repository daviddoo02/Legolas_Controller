# This code was written by David Ho 2023

from C_space import *

# gait_key = trapezoid_gait()

# gait_x, gait_y = generate_gait(gait_key)

# lookup_table, reject_pos = c_space(200)

# visualize_c_space(lookup_table, reject_pos, False, False)

# plt.plot(gait_x, gait_y, marker="o", markersize=2, markeredgecolor="red", markerfacecolor="green")
# plt.show()

# gait = np.column_stack((gait_x, gait_y))

# # Use numpy.savetxt() method to save the list as a CSV file
# np.savetxt("Gait1.csv", 
#            gait,
#            delimiter =",")  # Set the delimiter as a comma followed by a space

# leg_list = LegList()

# # From the gait coords, generate the angles

# angle = leg_list.inverse_kinematics(gait[0], 0)

# Hip_pitch=[]
# Knee=[]

# for coords in gait:
#         angle = leg_list.inverse_kinematics(coords, 0)
#         if angle is not None:
#         # Adjusting virtual range to real servo range
#                 Hip_pitch.append(-(angle[0] * 180 / pi) + 180)            # Virtual: 150 - 90 where 150 is forward;  Real: 90 and 30 where 30 is forward
#                 Knee.append(-(angle[1] * 180 / pi) + 155)                 # Virtual: 120 - 55 where 120 is full-contract;  Real: 100 and 35 where 35 is full-contract
#         else:
#                 print(coords)

# Gait_angle = np.column_stack((Hip_pitch,Knee))

# np.savetxt("Gait_angle1.csv", 
#            Gait_angle,
#            delimiter =",")  # Set the delimiter as a comma followed by a space
        
        
