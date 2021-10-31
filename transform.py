#!/usr/bin/env python

import numpy as np

# Convert unit quaternion to rotation matrix.
def Quat_to_Rot(Q):
    # Unpack quaternion components.
    q_0 = Q[0]
    q_1 = Q[1]
    q_2 = Q[2]
    q_3 = Q[3]
    
    # Compute R components.
    r_11 = q_0**2 + q_1**2 - q_2**2 - q_3**2
    r_12 = 2*(q_1*q_2 - q_0*q_3)
    r_13 = 2*(q_1*q_3 + q_0*q_2)
    
    r_21 = 2*(q_1*q_2 + q_0*q_3)
    r_22 = q_0**2 + q_2**2 - q_1**2 - q_3**2
    r_23 = 2*(q_2*q_3 - q_0*q_1)
    
    r_31 = 2*(q_1*q_3 - q_0*q_2)
    r_32 = 2*(q_2*q_3 + q_0*q_1)
    r_33 = q_0**2 + q_3**2 - q_1**2 - q_2**2
    
    # Combine into one R matrix.
    R = np.array([[r_11, r_12, r_13],
                  [r_21, r_22, r_23],
                  [r_31, r_32, r_33]])
    return R


g_sh = np.array([
  [0, -1/np.sqrt(2), 1/np.sqrt(2), .73338640471],
  [0, 1/np.sqrt(2), 1/np.sqrt(2), .73338640471],
  [1, 0, 0, .19135],
  [0, 0, 0, 1]
])

Q = [0.6507136500236983, -0.26414863893185586, 0.6567014923656346, 0.27484612449156426]
R = Quat_to_Rot(Q)

P = [0.9115866572901011, 1.1017962979289726, 0.30925253255776314]

g_bh = np.array([
  [R[0][0], R[0][1], R[0][2], P[0]],
  [R[1][0], R[1][1], R[1][2], P[1]],
  [R[2][0], R[2][1], R[2][2], P[2]],
  [0, 0, 0, 1]
])

g_bs = np.dot(np.linalg.inv(g_sh), g_bh)
print("R = ")
print(R)
print("g_bs = ")
print(g_bs)



'''
R1 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
R2 = np.array([[1, 0, 0], [0, 0.707, 0.707], [0, -0.707, 0.707]])
print(np.dot(R1, R2))

#  Point(x=0.9115866572901011, y=1.1017962979289726, z=0.30925253255776314), 'orientation': Quaternion(x=-0.26414863893185586, y=0.6567014923656346, z=0.27484612449156426, w=0.6507136500236983)}
'''



