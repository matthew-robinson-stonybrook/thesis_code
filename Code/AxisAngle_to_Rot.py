# Import modules.
import numpy as np
import math as m
from skew import skew

# Convert axis of rotation and angle to rotation matrix.
def AxisAngle_to_Rot(axis, angle):
    w_hat = skew(axis)
    I = np.identity(3)
    R = I + w_hat*m.sin(angle) + (np.dot(w_hat, w_hat))*(1-m.cos(angle))
    return R