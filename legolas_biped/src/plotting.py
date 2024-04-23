from kinematics import forward_kinematics, plot_leg_config, inverse_kinematics
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

# Input
th_1 = 0  # Hip 1
th_2 = 0  # Hip 2
th_3 = 0  # Thigh
th_4 = 0  # Foreleg

desire = np.array([-50, 100, -250])
guess = np.array([th_1, th_2, th_3, th_4])


nth_1, nth_2, nth_3, nth_4 = inverse_kinematics(desire, guess, learning_rate=0.1)
joints = forward_kinematics(nth_1, nth_2, nth_3, nth_4)
print(joints)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Make a vertically oriented slider to control the amplitude
axamp = fig.add_axes([0.05, 0.25, 0.0225, 0.63])
slider_x = Slider(
    ax=axamp,
    label="x",
    valmin=-50.0,
    valmax=50.0,
    valinit=0.0,
    orientation="vertical"
)
axamp = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
slider_y = Slider(
    ax=axamp,
    label="y",
    valmin=50.0,
    valmax=150.0,
    valinit=100.0,
    orientation="vertical"
)
axamp = fig.add_axes([0.15, 0.25, 0.0225, 0.63])
slider_z = Slider(
    ax=axamp,
    label="z",
    valmin=-300.0,
    valmax=-200.0,
    valinit=-250.0,
    orientation="vertical"
)


# The function to be called anytime a slider's value changes
def slider_event_x(val):
    print("updated :)")
    desire[0] = val
    nth_1, nth_2, nth_3, nth_4 = inverse_kinematics(desire, guess, learning_rate=0.1)
    joints = forward_kinematics(nth_1, nth_2, nth_3, nth_4)
    plot_leg_config(joints, fig, ax)
def slider_event_y(val):
    print("updated :)")
    desire[1] = val
    nth_1, nth_2, nth_3, nth_4 = inverse_kinematics(desire, guess, learning_rate=0.1)
    joints = forward_kinematics(nth_1, nth_2, nth_3, nth_4)
    plot_leg_config(joints, fig, ax)
def slider_event_z(val):
    print("updated :)")
    desire[2] = val
    nth_1, nth_2, nth_3, nth_4 = inverse_kinematics(desire, guess, learning_rate=0.1)
    joints = forward_kinematics(nth_1, nth_2, nth_3, nth_4)
    plot_leg_config(joints, fig, ax)


slider_x.on_changed(slider_event_x)
slider_y.on_changed(slider_event_y)
slider_z.on_changed(slider_event_z)

plot_leg_config(joints, fig, ax)
