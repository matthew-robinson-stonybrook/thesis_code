# Import modules.
import numpy as np
from skew import skew

# Function to compute the adjoint matrix of g.
def AdjointOfg(g):
    
    # Verify g is a numpy array, and unpack.
    g = np.array(g)
    R = g[0:3, 0:3]
    P = g[0:3, 3]

    # Compute product of P skew symmetric matrix and rotation matrix.
    P_hat = skew(P)
    prod = np.matmul(P_hat, R)

    # Create adjoint matrix and return.
    Ad_g = np.zeros((6, 6))
    Ad_g[0:3, 0:3] = R
    Ad_g[0:3, 3:6] = prod
    Ad_g[3:6, 3:6] = R
    return Ad_g