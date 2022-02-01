#!/usr/bin/env python

import numpy as np
import math as m
import matplotlib.pyplot as plt

import csv

from motion_planner_class import Motion_Planner
from baxter import Baxter
import motion_iterator as mit

# Baxter link lengths (check drawing reference)
baxter = Baxter()

# All thetas = 0
config0 = np.zeros(7)

#Position and orienation homogenous matrix of cup
g_handle= [[1, 0, 0, 0],
      [0, 1, 0, (baxter.l1 + baxter.l2 + baxter.l4 + baxter.l7) - 300],
      [0, 0, 1, baxter.l0 + baxter.l3 + baxter.l5],
      [0, 0, 0, 1]]

# Transformation Matrix to turn knob
g_turn = [[0, 0, -1, 0],
     [0, 1, 0, 0],
     [1, 0, 0, 0],
     [0, 0, 0, 1]]

g_turn = np.dot(g_handle, g_turn)

# Open door position will be same as initial position, but will be rotated 90 degrees about z
g_open = [[0, 0, -1, (baxter.l1 + baxter.l2 + baxter.l4 + baxter.l7 - 400) * m.cos(3 * m.pi / 8)],
          [0, 1, 0, (baxter.l1 + baxter.l2 + baxter.l4 + baxter.l7 - 400) * m.sin(3 * m.pi / 8)],
          [1, 0, 0, baxter.l0 + baxter.l3 + baxter.l5],
          [0, 0, 0, 1]]

# Create a stupid figure
fig1 = plt.figure(1)
ax = plt.axes(projection='3d')
plt.title("Motion from Neutral Position to Door Knob")

# Motion planner for going to handle
baxter_to_handle = Motion_Planner(baxter, g_handle)
# Plot them points
mit.motion_iterator(baxter_to_handle)

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

'''
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


