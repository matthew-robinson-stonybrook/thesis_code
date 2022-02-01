#!/usr/bin/env python

import numpy as np
import math as m
import matplotlib.pyplot as plt

import csv

from Motion_Planner import Motion_Planner
from manipdkin_specified_point import manipdkin_specified_point as mdkp

baxter_joint_ranges = [(-2.461, 0.89), (-2.147, 1.047), (-3.028, 3.038), (-0.052, 2.618), (-3.059, 3.059), (-1.571, 2.098), (-3.059, 3.059)]

# Creates Baxter at the initial config, iterates through the motion planner method until final position and orientaiton
# is achieved, and then plots the path and final config of baxter
def motion_iterator(motion_planner):
    # Baxter's first link starts at origin
    initial_points = [[0, 0, 0]]

    # Iterate through points defined for baxter links and run each point through manipdkin_specified_point (mdkp)
    # mdkp is same as manipdkin except assumes point has followed a certain number of joint axes
    for index in range(1, len(baxter_initial_config_points)):
        point = baxter_initial_config_points[index]
        g0 = [[1, 0, 0, point[0]], [0, 1, 0, point[1]], [0, 0, 1, point[2]], [0, 0, 0, 1]]
        initial_points.append(mdkp(g0, axis_joints, q_joints, type_joints, motion_planner.config, index)[0])

    #Plot the initial config using points just calculated
    initial_points = np.array(initial_points)
    x = initial_points[:, 0]
    y = initial_points[:, 1]
    z = initial_points[:, 2]
    ax.plot3D(x, y, z, 'yellow', label = 'Initial Position')

    g_x_axis = [[1, 0, 0, x_axis[0]], [0, 1, 0, x_axis[1]], [0, 0, 1, x_axis[2]], [0, 0, 0, 1]]
    g_y_axis = [[1, 0, 0, y_axis[0]], [0, 1, 0, y_axis[1]], [0, 0, 1, y_axis[2]], [0, 0, 0, 1]]
    g_z_axis = [[1, 0, 0, z_axis[0]], [0, 1, 0, z_axis[1]], [0, 0, 1, z_axis[2]], [0, 0, 0, 1]]

    # Plot x, y, and z axes of initial end-effector to show orientation
    x_axis_initial = mdkp(g_x_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    y_axis_initial = mdkp(g_y_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    z_axis_initial = mdkp(g_z_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_initial[0]], [y[-1], x_axis_initial[1]], [z[-1], x_axis_initial[2]], "red", label = 'End-Effector x-axis')
    ax.plot3D([x[-1], y_axis_initial[0]], [y[-1], y_axis_initial[1]],  [z[-1], y_axis_initial[2]], "green", label = 'End-Effector y-axis')
    ax.plot3D([x[-1], z_axis_initial[0]], [y[-1], z_axis_initial[1]], [z[-1], z_axis_initial[2]], "blue", label = 'End-Effector z-axis')

    # All data points for motion path to be plotted
    xdata = []
    ydata = []
    zdata = []

    # While value "r" is greater than a certain value, iterate through iterative motion planning algorithm
    # "r" is root-sum-square difference between each element of the current and desired transformation matrix
    r = 100
    i = 0
    while abs(r) >= 0.01:
        for b in range(0, int(1 / motion_planner.beta)):
            xdata.append(motion_planner.g[0, 3])
            ydata.append(motion_planner.g[1, 3])
            zdata.append(motion_planner.g[2, 3])

            motion_planner.iteration()
            motion_planner.update_values()

            # Find the current error between the rotation matrices
            r = 0
            for row in range(0,3):
                for column in range(0,4):
                    r += (motion_planner.gstf[row][column] - motion_planner.g[row][column]) ** 2

            r = m.sqrt(r)
            i += 1
    
    print(i)
    for joint in range(0,7):
        angle = motion_planner.config[joint]
        if not baxter_joint_ranges[joint][0] <= angle <= baxter_joint_ranges[joint][1]:
            print('WARNING! BAXTER JOINT ' + str(joint) + ' IS OUT OF RANGE!')

    # Plot data points for motion path
    ax.scatter3D(xdata, ydata, zdata, cmap='Greens')

    # Determine final baxter config using same method as initial
    final_points = [[0,0,0]]
    for index in range(1,len(baxter_initial_config_points)):
        point = baxter_initial_config_points[index]
        g0 = [[1, 0, 0, point[0]], [0, 1, 0, point[1]], [0, 0, 1, point[2]], [0, 0, 0, 1]]
        final_points.append(mdkp(g0, axis_joints, q_joints, type_joints, motion_planner.config, index)[0])

    final_points = np.array(final_points)
    x = final_points[:, 0]
    y = final_points[:, 1]
    z = final_points[:, 2]
    ax.plot3D(x, y, z, 'orange', label = 'Final Position')

    # Plot x, y, and z axes of final end-effector to show orientation
    x_axis_final = mdkp(g_x_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    y_axis_final = mdkp(g_y_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    z_axis_final = mdkp(g_z_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_final[0]], [y[-1], x_axis_final[1]], [z[-1], x_axis_final[2]], "red")
    ax.plot3D([x[-1], y_axis_final[0]], [y[-1], y_axis_final[1]],  [z[-1], y_axis_final[2]], "green")
    ax.plot3D([x[-1], z_axis_final[0]], [y[-1], z_axis_final[1]], [z[-1], z_axis_final[2]], "blue")

# Baxter link lengths (check drawing reference)
l1 = 270.35
l2 = 69
l3 = 364.35
l4 = 69
l5 = 374.29
l6 = 10
l7 = 374.42
l8 = 229.525


# Joint axes
theta_W0 = m.atan(l6/l5)
uS0 = [0, 0, 1]
uS1 = [-1/m.sqrt(2), 1/m.sqrt(2), 0]
uE0 = [1/m.sqrt(2), 1/m.sqrt(2), 0]
uE1 = uS1
uW0 = uE0
uW1 = uS1
uW2 = uE0
axis_joints = [uS0, uS1, uE0, uE1, uW0, uW1, uW2]

# Q Joints (Vector from base frame to joint axes)
qS0 = [0, 0, 0]
qS1 = [l2 * m.cos(m.pi / 4), l2 * m.sin(m.pi / 4), l1]
qE0 = [0, 0, l1]
qE1 = [(l2 + l3) * m.cos(m.pi / 4), (l2 + l3) * m.sin(m.pi/4), l1 - l4]
qW0 = qE1
qW1 = [(l2 + l3 + l5) * m.cos(m.pi / 4), (l2 + l3 + l5) * m.sin(m.pi / 4), l1 - l4 - l6]
qW2 = qW1
q_joints = [qS0, qS1, qE0, qE1, qW0, qW1, qW2]

# Type Joints (all revolute)
type_joints = ["R", "R", "R", "R", "R", "R", "R"]

# Initial position of Baxter where ALL thetas = 0
gst0 = [[1/m.sqrt(2), 1/m.sqrt(2), 0, (l2 + l3 + l5 + l8) * m.cos(m.pi / 4)],
     [-1/m.sqrt(2), 1/m.sqrt(2), 0, (l2 + l3 + l5 + l8) * m.sin(m.pi / 4)],
     [0, 0, 1, l1 - l4 - l6],
     [0, 0, 0, 1]]

# All thetas = 0
config0 = np.zeros(7)

#Position and orienation homogenous matrix of cup
g_handle= [[1, 0, 0, 0],
      [0, 1, 0, (l2 + l3 + l5 + l8) - 300],
      [0, 0, 1, l1 - l4 - l6],
      [0, 0, 0, 1]]

# Transformation Matrix to turn knob
g_turn = [[0, 0, -1, 0],
     [0, 1, 0, 0],
     [1, 0, 0, 0],
     [0, 0, 0, 1]]

g_turn = np.dot(g_handle, g_turn)
print(g_turn)

# Open door position will be same as initial position, but will be rotated 90 degrees about z
g_open = [[0, 0, -1, (l2 + l3 + l5 + l8 - 400) * m.cos(3 * m.pi / 8)],
          [0, 1, 0, (l2 + l3 + l5 + l8 - 400) * m.sin(3 * m.pi / 8)],
          [1, 0, 0, l1 - l4 - l6],
          [0, 0, 0, 1]]


# Points of initial position of Baxter to define linkage start and end points
cti = m.cos(m.pi / 4)
sti = m.sin(m.pi / 4)
p0 = [0, 0, l1]
p1 = [l2 * cti, l2 * sti, l1]
p2 = [(l2 + l3) * cti, (l2 + l3) * sti, l1]
p3 = [(l2 + l3) * cti, (l2 + l3) * sti, l1 - l4]
p4 = [(l2 + l3 + l5) * cti, (l2 + l3 + l5) * sti, l1 - l4]
p5 = p4
p5[2] = l1 - l4 - l6
p6 = [(l2 + l3 + l5 + l8) * cti, (l2 + l3 + l5 + l8) * sti, l1 - l4 - l6]

axis_length = 100
x_axis = [(l2 + l3 + l5 + l8 + axis_length) * cti, (l2 + l3 + l5 + l8 - axis_length) * sti, l1 - l4 - l6]
y_axis = [(l2 + l3 + l5 + l8 + axis_length) * cti, (l2 + l3 + l5 + l8 + axis_length) * sti, l1 - l4 - l6]
z_axis = [(l2 + l3 + l5 + l8) * cti, (l2 + l3 + l5 + l8) * sti, l1 - l4 - l6 + axis_length]

baxter_initial_config_points = np.array([[0, 0, 0], p0, p1, p2, p3, p4, p5, p6])


# Create a stupid figure
fig1 = plt.figure(1)
ax = plt.axes(projection='3d')
plt.title("Motion from Neutral Position to Door Knob")

# Motion planner for going to handle
baxter_to_handle = Motion_Planner(gst0, gst0, config0, g_handle, axis_joints, q_joints, type_joints)
# Plot them points
motion_iterator(baxter_to_handle)

print("Tao: " + str(baxter_to_handle.tau))
print("Beta: " + str(baxter_to_handle.beta))
print(" ")
print("Initial Configuration (gst0): ")
print(baxter_to_handle.gst0)
print('- - - - - - -')
print(" ")

print("Requested Handle Configuration: ")
print(baxter_to_handle.gstf)
print(" - - - - - - -  ")
print(" ")
print("Achieved Handle Configration: ")
print(baxter_to_handle.g)
print('- - - - - - -')
print(" ")

ax.legend()
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')
plt.show()


fig2 = plt.figure(2)
ax = plt.axes(projection='3d')
plt.title("Motion of Twisting Door Knob")

# Motion planner for twisting handle
baxter_handle_turn = Motion_Planner(gst0, baxter_to_handle.g, baxter_to_handle.config, g_turn, axis_joints, q_joints, type_joints)
# Plot them points
motion_iterator(baxter_handle_turn)

print("Requested Turned Configuration: ")
print(baxter_handle_turn.gstf)
print(" - - - - - - -  ")
print(" ")
print("Achieved Turned Configration: ")
print(baxter_handle_turn.g)
print('- - - - - - -')
print(" ")

ax.legend()
plt.show()

fig3 = plt.figure(3)
ax = plt.axes(projection='3d')
plt.title("Motion of Opening Door")

'''
# Motion planner for opening the door
baxter_open = Motion_Planner(gst0, baxter_handle_turn.g, baxter_handle_turn.config, g_open, axis_joints, q_joints, type_joints)
# Plot them points
motion_iterator(baxter_open)

print("Requested Opened Configuration (Gf): ")
print(baxter_open.gstf)
print(" - - - - - - -  ")
print(" ")
print("Achieved Turned Configration: ")
print(baxter_open.g)
print('- - - - - - -')
print(" ")

ax.legend()
plt.show()
'''


