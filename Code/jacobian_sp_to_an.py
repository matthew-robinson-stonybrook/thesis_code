# Import modules.
import numpy as np
from AdjointOfg import AdjointOfg
from skew import skew

# Function to convert spatial jacobian to analytic jacobian.
def jacobian_sp_to_an(J_sp, g_st):
    # Make sure inputs are numpy arrays.
    J_sp = np.array(J_sp)
    g_st = np.array(g_st)

    # Pull P from g_st and compute skew.
    P = g_st[0:3, 3]
    P_hat = skew(P)

    # Compute matrix to convert jacobian.
    mat = np.identity(6)
    mat[0:3, 3:6] = -P_hat

    # Convert and return analytic jacobian.
    return np.matmul(mat, J_sp)