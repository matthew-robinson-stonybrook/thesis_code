# Import modules.
import math as m
import numpy as np

# Convert rotation matrix to axis of rotation and angle.
def Rot_to_AxisAngle(R):
    R = np.array(R)
    angle = m.acos((np.trace(R)-1)/2)
    arr = np.array([(R[2][1]-R[1][2]), (R[0][2]-R[2][0]), (R[1][0]-R[0][1])])
    axis = (1/(2*m.sin(angle)))*arr
    return [axis, angle]