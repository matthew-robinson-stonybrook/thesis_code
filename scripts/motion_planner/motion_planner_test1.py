#!/usr/bin/env python

import numpy as np
import math as m
import matplotlib.pyplot as plt

import csv

from Motion_Planner import Motion_Planner
from manipdkin_specified_point import manipdkin_specified_point as mdkp
from baxter import Baxter
from motion_iterator import motion_iterator


# Baxter link lengths (check drawing reference)
baxter = Baxter()

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
baxter_to_handle = Motion_Planner(baxter.gst0, baxter.gst0, config0, g_handle, baxter.axis_joints, baxter.q_joints, baxter.type_joints)
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
baxter_handle_turn = Motion_Planner(baxter.gst0, baxter_to_handle.g, baxter_to_handle.config, g_turn, axis_joints, q_joints, type_joints)
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


