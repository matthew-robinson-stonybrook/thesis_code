import numpy as np
import math as m
from Quat_Math import Quat_Math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class Motion_Planner(Quat_Math):
    def __init__(self, gst0, g0, config0, gstf, axis_joints, q_joints, type_joints):
        self.gst0 = np.array(gst0)
        self.g0 = np.array(g0)
        self.A0 = self.g_to_dual_quat(self.g0)
        self.g = np.array(g0)
        self.config = config0
        self.gstf = np.array(gstf)
        self.axis_joints = axis_joints
        self.q_joints = q_joints
        self.type_joints = type_joints

        self.Af = self.g_to_dual_quat(self.gstf)
        self.tau = 0.01
        self.beta = 1

        # Step 1: Convert g into gamma and dual unit quat At
        self.At = 0
        self.gamma = 0

        # Step 2: Get A(t + 1)
        self.AtAf = 0
        self.At_prime = 0

        # Step 3: Get gamma(t+1) from A(t+1)
        self.gamma_prime = 0

        # Step 4: Get next joint configuration (theta(t + 1))
        self.spatial_jac = 0
        self.B = 0
        self.config_prime = 0

        # Step 5: Get next end-effector orientation (g(t+1))
        self.g_prime = 0

    def iteration(self):
        # Step 1: Convert g into gamma and dual unit quat At
        self.At = self.g_to_dual_quat(self.g)
        self.gamma = self.dual_quat_to_gamma(self.At)

        # Step 2: Get A(t + 1)
        self.AtAf = self.dual_quat_product(self.dual_conjugate(self.At), self.Af)
        self.At_prime = self.dual_quat_product(self.At, self.power_of_dual_quat(self.AtAf, self.tau))

        # Step 3: Get gamma(t+1) from A(t+1)
        self.gamma_prime = self.dual_quat_to_gamma(self.At_prime)

        # Step 4: Get next joint configuration (theta(t + 1))
        self.spatial_jac = self.spatial_manip_jac(self.config)
        self.B = self.B_matrix()
        self.config_prime = self.config + self.beta * np.dot(self.B, (self.gamma_prime - self.gamma))

        # Step 5: Get next end-effector orientation (g(t+1))
        self.g_prime = self.manipdkin(self.config_prime)

    def update_values(self):
        self.g = self.g_prime
        self.config = self.config_prime

        A0Af = self.dual_quat_product(self.dual_conjugate(self.A0), self.Af)
        thetai = 2 * np.arccos(A0Af[0,0])
        pi = self.dual_quat_to_p(A0Af)
        pi = m.sqrt(pi[0]**2 + pi[1]**2 + pi[2]**2)

        AtAf = self.dual_quat_product(self.dual_conjugate(self.At_prime), self.Af)
        theta = 2 * np.arccos(AtAf[0,0])
        p = self.dual_quat_to_p(AtAf)
        p = m.sqrt(p[0]**2 + p[1]**2 + p[2]**2)

        dtheta = abs(theta / thetai)
        dp = abs(p/pi)
        d = (dtheta + dp) / 2

        if self.tau < 0.2:
            self.tau += (1/d) * self.beta * 0.001

        if self.tau >= 0.2:
            self.tau = 0.2


    def spatial_manip_jac(self, thetas):
        n = len(self.axis_joints)

        # Step 1: Calculate joint twists for each joint
        joint_twists = []

        # For step 2
        g1_is = [np.identity(4)]
        for joint in range(n):
            uj = np.array(self.axis_joints[joint])
            qj = np.array(self.q_joints[joint])
            tj = np.array(self.type_joints[joint])
            t = thetas[joint]

            if tj == "R":
                cross = np.cross(-uj, qj)
                twist = np.hstack((cross, uj))
                joint_twists.append(twist)
            else:
                twist = np.hstack((uj, np.array([0, 0, 0])))
                joint_twists.append(twist)

            #Step 2: Calculate transformation matrices from g1 to current joint
            transformation = self.transformation(uj, qj, t)
            g1_is.append(np.dot(g1_is[joint], transformation))

        # Step 3: Compute Adjoint matrices and ith column of Jacobian = Ad_(g1,i-1) * zi
        g1_is.pop()
        jacobian = []
        for element in range(len(g1_is)):
            adjoint = self.adjoint_of_g(g1_is[element])
            j_column = np.dot(adjoint, joint_twists[element])
            jacobian.append(j_column.tolist())

        jacobian = np.array(jacobian)
        jacobian = np.transpose(jacobian)
        jacobian = np.around(jacobian, decimals = 4)
        return jacobian

    def transformation(self, axis_joint, q_joint, theta):
        I = np.identity(3)
        t = theta
        skew = np.array([[0, -axis_joint[2], axis_joint[1]], [axis_joint[2], 0, -axis_joint[0]],
                         [-axis_joint[1], axis_joint[0], 0]])
        rot = I + skew * m.sin(t) + np.dot(skew, skew) * (1 - m.cos(t))
        a = (I - rot)
        trans = np.dot(a, q_joint)
        t1 = np.append(rot[0], trans[0])
        t2 = np.append(rot[1], trans[1])
        t3 = np.append(rot[2], trans[2])
        transformation = np.array([t1.tolist(), t2.tolist(), t3.tolist(), [0, 0, 0, 1]])
        return transformation

    def adjoint_of_g(self, g):
        g = np.array(g)
        # Rotation Matrix
        R = g[0:3, 0:3]
        # Translation
        p = g[0:3, 3]
        # Skew Symetric Translation
        p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
        pR = np.dot(p_hat, R)
        Ad_g = np.hstack((np.vstack((R, np.zeros((3, 3)))), np.vstack((pR, R))))

        return Ad_g

    def B_matrix(self):
        J1 = self.quat_skew(self.At[0])
        p_hat = self.skew(self.g[0:3, 3])
        top_right_3by3 = 2 * np.dot(p_hat, J1)
        J2 = np.vstack((np.hstack((np.identity(3), top_right_3by3)), np.hstack((np.zeros((3,3)), 2 * J1))))
        B = np.dot(np.dot(self.spatial_jac.transpose(), np.linalg.inv(np.dot(self.spatial_jac, self.spatial_jac.transpose()))), J2)
        return B



    def manipdkin(self, thetas):
        # Identity Matrix
        I = np.identity(3)

        # Creates a skew matrix for each axis
        skew_axis = []
        for axis in self.axis_joints:
            skew_axis.append(self.skew(axis))

        transformation_matrices = []

        # Computes each g matrix
        for index in range(len(thetas)):
            omega = self.axis_joints[index]
            q = self.q_joints[index]
            t = thetas[index]

            # Revolute Joints
            if self.type_joints[index] == "R":
                rot = I + skew_axis[index] * m.sin(t) + np.dot(skew_axis[index], skew_axis[index]) * (1 - m.cos(t))
                a = (I - rot)
                trans = np.dot(a, q)

                t1 = np.append(rot[0], trans[0])
                t2 = np.append(rot[1], trans[1])
                t3 = np.append(rot[2], trans[2])
                transformation = np.array([t1.tolist(), t2.tolist(), t3.tolist(), [0, 0, 0, 1]])

            # Prismatic
            elif self.type_joints[index] == "P":
                transformation = np.array(
                    [[1, 0, 0, omega[0] * t], [0, 1, 0, omega[1] * t], [0, 0, 1, omega[2] * t], [0, 0, 0, 1]])

            transformation_matrices.append(transformation)

        transformation_matrices.append(self.gst0)

        # Muitiplies all g matrices to obtain final transformation
        g_theta = np.identity(4)
        for transformation_matrix in transformation_matrices:
            g_theta = np.dot(g_theta, transformation_matrix)

        return g_theta

