# Import modules.
import numpy as np

# Function to compute the twist coordinates.
def twist(u, q, joint_type):

    # Verify inputs are numpy arrays.
    u = np.array(u)
    q = np.array(q)

    # Compute based on joint type.
    xi = np.zeros(6)
    if joint_type.upper() == 'R':
        xi[0:3] = -np.cross(u, q)
        xi[3:6] = u
    elif joint_type.upper() == 'P':
        xi[0:3] = u
    else:
        print("Please specify 'R' or 'P' as joint type.")
        return 0
    
    # Return value of xi.
    return xi