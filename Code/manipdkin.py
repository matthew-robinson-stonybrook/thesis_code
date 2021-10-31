# Import modules.
import math as m
import numpy as np
from skew import skew
from AxisAngle_to_Rot import AxisAngle_to_Rot


# Manipulator direct kinematics function.
def manipdkin(gst0, axis_joints, q_joints, type_joints, theta):
    I = np.identity(3) # 3x3 identity matrix.
    g = [] # Initialize list of T matrices.
    
    # Loop over every joint and compute T of the joint.
    for i in range(len(type_joints)):
        T = np.zeros((4,4)) # Initialize T.
        joint_type = type_joints[i].upper() # Unpack joint type.
        
        # Revolute joint.
        if joint_type=='R':
            # Unpack values and compute R.
            w = axis_joints[:,i]
            q = q_joints[:,i]
            R = AxisAngle_to_Rot(w, theta[i])
            
            # Place computed R 3x3 matrix into T 4x4 matrix.
            rows, cols = np.shape(R)
            for row in range(rows):
                for col in range(cols):
                    T[row, col] = R[row, col]
            
            # Place computed P 3x1 vector into T 4x4 matrix.
            P = np.dot((I-R), q)
            for row in range(len(P)):
                T[row, cols] = P[row]
                
            # Place 1x3 zero vector and 1 scalar into T 4x4 matrix.
            T[cols,:] = [0, 0, 0, 1]
            
            # Append computed T for this joint into list of all T matrices.
            g.append(T)
            
        # Prismatic joint.    
        elif joint_type=='P':
            # Unpack v vector.
            v = axis_joints[:,i]
            
            # Place identity 3x3 matrix into T 4x4 matrix.
            rows, cols = np.shape(I)
            for row in range(rows):
                for col in range(cols):
                    T[row, col] = I[row, col]
            
            # Place computed P 3x1 vector into T 4x4 matrix.
            P = v*theta[i]
            for row in range(len(P)):
                T[row, cols] = P[row]
            
            # Place 1x3 zero vector and 1 scalar into T 4x4 matrix.
            T[cols,:] = [0, 0, 0, 1]
            
            # Append computed T for this joint into list of all T matrices.
            g.append(T)
        
        # Inform user if joint type string is incorrect.            
        else:
            print("Error! Joint type must be 'R' or 'P'.")
    
    # Compute product of transformation matrices T in list g.
    g = np.array(g)
    prod = np.linalg.multi_dot(g)
    
    # Direct kinematics solution is the product of all the T matrices and gst0.
    g_base_tool = np.dot(prod, gst0)
    
    # Return transformation matrix of {T} w.r.t. {S}.
    return g_base_tool