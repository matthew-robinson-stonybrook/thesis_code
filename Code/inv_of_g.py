# Import modules.
import numpy as np

# Function to calculate the inverse of a transformation matrix.
def inv_of_g(g):

    # Make sure g is a numpy array.
    g = np.array(g)

    # Unpack R and P.
    R = g[0:3, 0:3]
    R_trans = np.transpose(R)
    P = g[0:3, 3]

    # Calculate and return inverse of g.
    g_inv = np.zeros((4, 4))
    g_inv[0:3, 0:3] = R_trans
    g_inv[0:3, 3] = -np.matmul(R_trans, P)
    g_inv[3, 3] = 1
    return g_inv