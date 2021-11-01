import numpy as np
import math as m


def manipdkin_specified_point(gst0, axis_joints, q_joints, type_joints, thetas, previous_joints):
    # Identity Matrix
    I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Creates a skew matrix for each axis
    skew_axis = []
    for axis in axis_joints:
        skew_axis.append(np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]))

    transformation_matrices = []

    # Computes each g matrix
    for index in range(previous_joints):
        omega = axis_joints[index]
        q = q_joints[index]
        t = thetas[index]
        # Revolute Joints
        if type_joints[index] == "R":
            rot = I + skew_axis[index] * m.sin(t) + np.dot(skew_axis[index], skew_axis[index]) * (1 - m.cos(t))
            a = (I - rot)
            trans = np.dot(a, q)

            t1 = np.append(rot[0], trans[0])
            t2 = np.append(rot[1], trans[1])
            t3 = np.append(rot[2], trans[2])
            transformation = np.array([t1.tolist(), t2.tolist(), t3.tolist(), [0, 0, 0, 1]])

        # Prismatic
        elif type_joints[index] == "P":
            transformation = np.array([[1, 0, 0, omega[0] * t], [0, 1, 0, omega[1] * t], [0, 0, 1, omega[2] * t], [0, 0, 0, 1]])

        transformation_matrices.append(transformation)

    transformation_matrices.append(gst0)
    # Muitiplies all g matrices to obtain final transformation
    g_final = np.identity(4)
    for transformation_matrix in transformation_matrices:
        g_final = np.dot(g_final, transformation_matrix)

    p_final = g_final[0:3, 3]
    return p_final, g_final
