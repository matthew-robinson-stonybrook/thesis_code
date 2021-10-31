# Import modules.
import numpy as np
from skew import skew
from Quat_to_Rot import Quat_to_Rot

# Quaternion class.
class Quaternion:

    # Initialize object.
    def __init__(self, Q):
        self.Q = np.array(Q)

    # Quaternion representaion.
    def __repr__(self):
        str_1 = 'Quaternion:'
        str_2 = f'\n    Q = {self.Q}'
        return str_1 + str_2

    # Quaternion addition.
    def __add__(self, _):
        return Quaternion(self.Q + _.Q)

    # Dual quaternion subtraction.
    def __sub__(self, _):
        return Quaternion(self.Q - _.Q)

    # Dual quaternion multiplication.
    def __mul__(self, _):
        Q_p = np.zeros((4,4))
        Q_p[0, 0] = self.Q[0]
        Q_p[1:4, 0] = self.Q[1:4]
        Q_p[0, 1:4] = -self.Q[1:4]
        Q_p[1:4, 1:4] =  self.Q[0]*np.identity(3) + skew(self.Q[1:4])
        prod = np.matmul(Q_p, _.Q)
        return Quaternion(prod)

    # Division of quaternion by a scalar.
    def __truediv__(self, _):
        return Quaternion(self.Q / _)

    # Power of a quaternion.
    def __pow__(self, n):
        pass

    # Conjugate of a quaternion.
    def conjugate(self):
        conj = np.zeros(4)
        conj[0] = self.Q[0]
        conj[1:4] = -self.Q[1:4]
        return Quaternion(conj)

    # Magnitude of a quaternion.
    def magnitude(self):
        return np.linalg.norm(self.Q)

    # Inverse of a quaternion.
    def inverse(self):
        return Quaternion(self.conjugate() / self.magnitude())

    # Normalize quaternion.
    def norm(self):
        return self / self.magnitude()

    # Get associated rotation matrix.
    def rot_mat(self):
        return Quat_to_Rot(self.Q)