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
      [0, 0, 1, baxter.l0 - baxter.l3 - baxter.l5],
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
          [1, 0, 0, baxter.l0 - baxter.l3 - baxter.l5],
          [0, 0, 0, 1]]



# Create a stupid figure
fig1 = plt.figure(1)
ax = plt.axes(projection='3d')
plt.title("Motion from Neutral Position to Door Knob")

# Motion planner for going to handle
baxter_to_handle = Motion_Planner(baxter, g_handle)
# Plot them points
mit.motion_iterator(baxter_to_handle, ax)


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

# Create a figure
fig2 = plt.figure(2)
plt.title("Joint 1 Path")

joint1_data = np.array(baxter_to_handle.joint_path)[:,0].tolist()
joint2_data = np.array(baxter_to_handle.joint_path)[:,1].tolist()
joint3_data = np.array(baxter_to_handle.joint_path)[:,2].tolist()
joint4_data = np.array(baxter_to_handle.joint_path)[:,3].tolist()
joint5_data = np.array(baxter_to_handle.joint_path)[:,4].tolist()
joint6_data = np.array(baxter_to_handle.joint_path)[:,5].tolist()
joint7_data = np.array(baxter_to_handle.joint_path)[:,6].tolist()

l = len(joint1_data)
plt.plot(joint1_data, np.linspace(0, l, l), color = 'red', label = "Joint 1")
plt.plot(joint2_data, np.linspace(0, l, l), color = 'orange', label = "Joint 2")
plt.plot(joint3_data, np.linspace(0, l, l), color = 'yellow', label = "Joint 3")
plt.plot(joint4_data, np.linspace(0, l, l), color = 'green', label = "Joint 4")
plt.plot(joint5_data, np.linspace(0, l, l), color = 'blue', label = "Joint 5")
plt.plot(joint6_data, np.linspace(0, l, l), color = 'purple', label = "Joint 6")
plt.plot(joint7_data, np.linspace(0, l, l), color = 'black', label = "Joint 7")
plt.show()

'''
# Motion planner for twisting handle
fig2 = plt.figure(2)
ax = plt.axes(projection='3d')
plt.title("Motion of Twisting Door Knob")

baxter_handle_turn = Motion_Planner(baxter, g_turn)
# Plot them points
mit.motion_iterator(baxter_handle_turn, ax)

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
baxter_open = Motion_Planner(baxter, g_open)
# Plot them points
mit.motion_iterator(baxter_open, ax)

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


