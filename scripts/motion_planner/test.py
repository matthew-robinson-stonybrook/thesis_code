import numpy as np
import math as m
from quat_math import Quat_Math

n1 = np.array([1,3])
n2 = np.array([2,5])
v1 = np.array([[0.1, 0.3, 0.5], [0.2, 0.4, 0.6]])
v2 = np.array([[0.7, 0.9, 1.1], [0.8, 1.0, 1.2]])

a = m.sqrt(n1[0]**2 + v1[0][0]**2 + v1[0][1]**2 + v1[0][2]**2)
b = m.sqrt(n1[1]**2 + v1[1][0]**2 + v1[1][1]**2 + v1[1][2]**2)
c = m.sqrt(n2[0]**2 + v2[0][0]**2 + v1[0][1]**2 + v1[0][2]**2)
d = m.sqrt(n2[1]**2 + v2[1][0]**2 + v1[1][1]**2 + v1[1][2]**2)

dq1 = np.array([[n1[0]/a, v1[0][0]/a, v1[0][1]/a, v1[0][2]/a], [n1[1]/b, v1[1][0]/b, v1[1][1]/b, v1[1][2]/b]])
dq2 = np.array([[n2[0]/c, v2[0][0]/c, v2[0][1]/c, v2[0][2]/c], [n2[1]/d, v2[1][0]/d, v2[1][1]/d, v2[1][2]/d]])

g = np.array([[0.707, -0.707, 0, 5], [0.707, 0.707, 0, 7], [0, 0, 1, 9.3], [0, 0, 0, 1]])
quat_math = Quat_Math()
print("g to dual quat")
print(quat_math.g_to_dual_quat(g))

