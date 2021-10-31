# Import modules.
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