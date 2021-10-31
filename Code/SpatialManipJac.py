# Import modules.
import math as m
import numpy as np
from twist import twist
from manipdkin import manipdkin
from AdjointOfg import AdjointOfg
from get_g_list import get_g_list
from mult_range import mult_range

# Function to compute the spatial manipulator jacobian.
def SpatialManipJac(gst0, axis_joints, q_joints, type_joints, theta):
    # Verify all inputs are numpy arrays.
    gst0 = np.array(gst0)
    axis_joints = np.array(axis_joints)
    q_joints = np.array(q_joints)
    type_joints = np.array(type_joints)
    theta = np.array(theta)

    # Generate list of transformation matrices for the manipulator.
    g_list = get_g_list(gst0, axis_joints, q_joints, type_joints, theta)
    
    # Function to compute xi prime.
    def xi_prime(xi, n):
        # Verify inputs are numpy arrays.
        xi = np.array(xi)

        # Compute g from 1 to n-1 and its adjoint.
        g_r = mult_range(g_list, 0, n)
        Ad_g_r = AdjointOfg(g_r)

        # Compute and return xi_prime.
        xi_prime = np.matmul(Ad_g_r, xi)
        return xi_prime

    # Compute values of xi_prime.
    xi_prime_list = []
    for i in range(1, len(type_joints)):
        # Compute twist coordinates.
        u = axis_joints[0:3, i]
        q = q_joints[0:3, i]
        joint_type = type_joints[i]
        xi = twist(u, q, joint_type)

        # Compute xi_prime and append to list.
        xi_prime_val = xi_prime(xi, i)
        xi_prime_list.append(xi_prime_val)

    # Create manipulator jacobian matrix.
    # First column.
    J = np.zeros((6, len(type_joints)))
    J[0:6, 0] = twist(axis_joints[0:3, 0], q_joints[0:3, 0], type_joints[0])

    # Other columns.
    for j in range(len(xi_prime_list)):
        J[0:6, (j+1)] = xi_prime_list[j]

    # Return value.
    return J