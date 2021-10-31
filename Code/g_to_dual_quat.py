# Import modules.
import numpy as np
from DualQuaternion import DualQuaternion
from Quaternion import Quaternion
from Rot_to_Quat import Rot_to_Quat

# Function to convert transformation matrix into dual quaternion.
def g_to_dual_quat(g):
    g = np.array(g)
    D_r_arr = Rot_to_Quat(g[0:3, 0:3])
    D_d_arr = [0, g[0, 3], g[1, 3], g[2, 3]]
    D_r = Quaternion(D_r_arr)
    P = Quaternion(D_d_arr)
    D_d = 0.5*(P*D_r).Q
    return DualQuaternion(D_r.Q, D_d)