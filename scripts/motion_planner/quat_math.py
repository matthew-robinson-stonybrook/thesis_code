import numpy as np
import math as m

class Quat_Math:
    def skew(self, x):
        return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

    def quat_skew(self, Q):
        q0, q1, q2, q3 = Q
        return np.array([[-q1, q0, q3, -q2],
                        [-q2, -q3, q0, q1],
                        [-q3, q2, -q1, q0]])


    def conjugate(self, A):
        return np.array([A[0], -A[1], -A[2], -A[3]])

    def dual_conjugate(self, A):
        return np.array([[A[0][0], -A[0][1], -A[0][2], -A[0][3]],
                         [A[1][0], -A[1][1], -A[1][2], -A[1][3]]])

    def quat_product(self, q, p):
        bottom_3by3 = q[0] * np.identity(3) + self.skew(q[1:4])
        top_row = np.array([[q[0], -q[1], -q[2], -q[3]]])
        Q_plus = np.vstack((top_row.tolist(),
                            np.hstack((q[1], bottom_3by3[0])).tolist(),
                            np.hstack((q[2], bottom_3by3[1])).tolist(),
                            np.hstack((q[3], bottom_3by3[2])).tolist()))

        return np.dot(Q_plus, p)

    # The Product of 2 dual numbers
    def dual_num_product(self, D1, D2):
        a1 = D1[0]
        b1 = D1[1]
        a2 = D2[0]
        b2 = D2[1]
        return np.vstack(((a1 * a2), (a1 * b2 + a2 * b1)))

    # The product of 2 dual vectors
    def dual_vec_product(self, E, F):
        EF1 = self.dual_num_product(E[0:2, 0], F[0:2, 0])
        EF2 = self.dual_num_product(E[0:2, 1], F[0:2, 1])
        EF3 = self.dual_num_product(E[0:2, 2], F[0:2, 2])
        return EF1 + EF2 + EF3

    # The product of 2 dual quaternions
    def dual_quat_product(self, A, B):
        # Scalar Component
        As = A[0:2, 0]
        Bs = B[0:2, 0]
        Av = A[0:2, 1:4]
        Bv = B[0:2, 1:4]
        AsBs = self.dual_num_product(As, Bs)
        ABs = AsBs - self.dual_vec_product(Av, Bv)

        # Vector Component
        AsBv = self.dual_num_dual_vector(As, Bv)
        BsAv = self.dual_num_dual_vector(Bs, Av)
        AvBv = self.dual_vector_cross(Av, Bv)
        ABv = (AsBv + BsAv) + AvBv
        AB = np.hstack((ABs, ABv))
        return AB

    def dual_vector_cross(self, E, F):
        E1 = E[0:, 0]
        E2 = E[0:, 1]
        E3 = E[0:, 2]
        F1 = F[0:, 0]
        F2 = F[0:, 1]
        F3 = F[0:, 2]
        row1 = self.dual_num_product(E2, F3) - self.dual_num_product(E3, F2)
        row2 = self.dual_num_product(E3, F1) - self.dual_num_product(E1, F3)
        row3 = self.dual_num_product(E1, F2) - self.dual_num_product(E2, F1)
        EF = np.hstack((row1, row2, row3))
        return EF

    def dual_num_dual_vector(self, D, E):
        DE1 = self.dual_num_product(D, E[0:2, 0]).tolist()
        DE2 = self.dual_num_product(D, E[0:2, 1]).tolist()
        DE3 = self.dual_num_product(D, E[0:2, 2]).tolist()
        DE = np.hstack((DE1, DE2, DE3))
        return DE

    def power_of_dual_quat(self, A, tau):
        Ar = A[0, 0:4]
        theta = 2 * np.arccos(Ar[0])
        u = (Ar[1:4] / np.sin(theta / 2))
        p = self.dual_quat_to_p(A)
        d = np.dot(p, u)
        m = (0.5) * (np.cross(p, u) + ((p - (d * u)) * (1 / np.tan(theta / 2))))
        u_hat = np.vstack((u, m))
        sin = np.vstack((np.sin((tau * theta) / 2), ((((tau * d) / 2)) * np.cos((tau * theta) / 2))))
        cos = np.vstack((np.cos((tau * theta) / 2), -((((tau * d) / 2)) * np.sin((tau * theta) / 2))))
        D_vector = self.dual_num_dual_vector(sin, u_hat)
        D_tau = np.hstack((cos, D_vector))

        return D_tau

    def g_to_dual_quat(self, g):
        R = g[0:3, 0:3]
        p = g[0:3, 3]
        p_quat = np.hstack(([0], p))
        Ar = self.rot_to_quat(R)
        Ad = 0.5 * self.quat_product(p_quat, Ar)
        A = np.vstack((Ar, Ad))
        return A

    def g_to_gamma(self, g):
        R = g[0:3, 0:3]
        p = g[0:3, 3]
        Ar = self.rot_to_quat(R)
        gamma = np.hstack((p[0:3], Ar))

        return gamma

    def dual_quat_to_gamma(self, A):
        Ar = A[0]
        Ad = A[1]
        p = 2 * self.quat_product(Ad, self.conjugate(Ar))
        gamma = np.hstack([p[1:4], Ar])
        return gamma


    def rot_to_quat(self, R):
        r11 = R[0][0]
        r12 = R[0][1]
        r13 = R[0][2]

        r21 = R[1][0]
        r22 = R[1][1]
        r23 = R[1][2]

        r31 = R[2][0]
        r32 = R[2][1]
        r33 = R[2][2]

        M = np.array([[1, 1, 1, 1], [1, -1, -1, 1],
                      [-1, 1, -1, 1], [-1, -1, 1, 1]])
        D = np.array([r11, r22, r33, 1])
        Q = (1 / 4) * np.dot(M, D)

        qMax = max(Q[0], Q[1], Q[2], Q[3])
        qMaxIndex = np.where(Q == qMax)[0][0]

        if qMaxIndex == 0:
            q0 = m.sqrt(qMax)
            q1 = (r32 - r23) / (4 * q0)
            q2 = (r13 - r31) / (4 * q0)
            q3 = (r21 - r12) / (4 * q0)

        elif qMaxIndex == 1:
            q1 = m.sqrt(qMax)
            q0 = (r32 - r23) / (4 * q1)
            q2 = (r12 + r21) / (4 * q1)
            q3 = (r13 + r31) / (4 * q1)

        elif qMaxIndex == 2:
            q2 = m.sqrt(qMax)
            q0 = (r13 - r31) / (4 * q2)
            q1 = (r12 + r21) / (4 * q2)
            q3 = (r23 + r32) / (4 * q2)

        elif qMaxIndex == 3:
            q3 = m.sqrt(qMax)
            q0 = (r21 - r12) / (4 * q3)
            q1 = (r13 + r31) / (4 * q3)
            q2 = (r23 + r32) / (4 * q3)

        return np.array([q0, q1, q2, q3])

    def dual_quat_to_p(self, A):
        Ar = A[0, 0:4]
        Ar_conj = self.conjugate(Ar)
        Ad = A[1, 0:4]
        p_4x1 = self.quat_product((2 * Ad), Ar_conj)
        p = p_4x1[1:4]
        return p
