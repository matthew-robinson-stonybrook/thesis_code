# Import modules.
import numpy as np
from skew import skew
from AxisAngle_to_Rot import AxisAngle_to_Rot

# Function to form a list of transformation matrices for the manipulator.
def get_g_list(gst0, axis_joints, q_joints, type_joints, theta):
    
    # Compute g of every frame conversion and append to list.
    g_list = []
    for i in range(0, len(type_joints)):

        # Unpack joint type and define I.
        joint_type = type_joints[i].upper()
        I = np.identity(3)
        
        # Revolute joint.
        if joint_type=='R':
            # Unpack values.
            w = axis_joints[:,i]
            q = q_joints[:,i]

            # Compute R and P
            R = AxisAngle_to_Rot(w, theta[i])
            P = np.dot((I-R), q)
            
        # Prismatic joint.    
        elif joint_type=='P':
            # Unpack v vector.
            v = axis_joints[:,i]

            # Assign R as identity and compute P.
            R = I
            P = v*theta[i]
        
        # Inform user if joint type string is incorrect.            
        else:
            print("Error! Joint type must be 'R' or 'P'.")
    
        # Create g matrix with R and P and append to list.
        g = np.zeros((4, 4))
        g[0:3, 0:3] = R
        g[0:3, 3] = P
        g[3, 3] = 1
        g_list.append(g)

    # Compute product of transformation matrices T in list g.
    return np.array(g_list)