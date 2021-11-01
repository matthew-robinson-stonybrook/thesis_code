#!/usr/bin/env python

import numpy as np
import math as m
import matplotlib.pyplot as plt

from Motion_Planner import Motion_Planner
from manipdkin_specified_point import manipdkin_specified_point as mdkp

# Baxter link lengths (check drawing reference)
l1 = 270.35
l2 = 69
l3 = 364.35
l4 = 69
l5 = 374.29
l6 = 10
l7 = 374.42
l8 = 229.525

# Initial position of Baxter where ALL thetas = 0
gst0 = [[1/m.sqrt(2), 1/m.sqrt(2), 0, (l2 + l3 + l5 + l8) * m.cos(m.pi / 4)],
     [-1/m.sqrt(2), 1/m.sqrt(2), 0, (l2 + l3 + l5 + l8) * m.sin(m.pi / 4)],
     [0, 0, 1, l1 - l4 - l6],
     [0, 0, 0, 1]]

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

thetas = [0, 0, 0, 0, 0, 0, 0]

final_pos, final_config = mdkp(gst0, axis_joints, q_joints, type_joints, thetas, 7)

print("Gst0: ")
print(gst0)
print("Final Configuration: ")
print(final_config)
print("Final Position: ")
print(final_pos)
