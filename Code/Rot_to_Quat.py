# Import modules.
import numpy as np

# Convert rotation matrix to unit quaternion.
def Rot_to_Quat(R):
    # Solve system, find index of max value.
    # Max value will determine which equations to use.
    A = np.array([[1, 1, -1, -1],
                  [1, -1, 1, -1],
                  [1, -1, -1, 1],
                  [1, 1, 1, 1]])
    b = np.array([R[0][0], R[1][1], R[2][2], 1])
    x = np.dot(np.linalg.inv(A), b)
    index = np.argmax(x) # index number is the quaternion component index.

    # Compute quaternions based on which value is max.
    if index==0:
        q_0 = x[index]**(1/2)
        q_1 = (R[2][1]-R[1][2])/(4*q_0)
        q_2 = (R[0][2]-R[2][0])/(4*q_0)
        q_3 = (R[1][0]-R[0][1])/(4*q_0)
    elif index==1:
        q_1 = x[index]**(1/2)
        q_0 = (R[2][1]-R[1][2])/(4*q_1)
        q_2 = (R[0][1]+R[1][0])/(4*q_1)
        q_3 = (R[0][2]+R[2][0])/(4*q_1)
    elif index==2:
        q_2 = x[index]**(1/2)
        q_0 = (R[0][2]-R[2][0])/(4*q_2)
        q_1 = (R[0][1]+R[1][0])/(4*q_2)
        q_3 = (R[1][2]+R[2][1])/(4*q_2)
    else:
        q_3 = x[index]**(1/2)
        q_0 = (R[1][0]-R[0][1])/(4*q_3)
        q_1 = (R[0][2]+R[2][0])/(4*q_3)
        q_2 = (R[1][2]+R[2][1])/(4*q_3)
        
    Q = [q_0, q_1, q_2, q_3]
    return Q