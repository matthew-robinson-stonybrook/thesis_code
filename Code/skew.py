# Import modules.
import numpy as np

# Skew symmetrix matrix function.
def skew(w):
    w = np.array(w)
    w_hat = np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])
    return w_hat