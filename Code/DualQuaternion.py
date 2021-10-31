# Import modules.
import math as m
import numpy as np
from DualNumber import DualNumber
from DualVector import DualVector
from Quaternion import Quaternion
from Rot_to_AxisAngle import Rot_to_AxisAngle

# Dual quaternion data type.
class DualQuaternion:
    
    # Initialize object.
    def __init__(self, arg_1=None, arg_2=[None]):
        if not arg_2[0]==None:
            self.real = Quaternion(arg_1)
            self.dual = Quaternion(arg_2)
        else:
            self.real = Quaternion([arg_1[0].a, arg_1[1].a, arg_1[2].a, arg_1[3].a])
            self.dual = Quaternion([arg_1[0].b, arg_1[1].b, arg_1[2].b, arg_1[3].b])

    # Dual quaternion representaion.
    def __repr__(self):
        str_1 = 'Dual Quaternion:'
        str_2 = f'\n    D_real = {self.real.Q}'
        str_3 = f'\n    D_dual = {self.dual.Q}'
        return str_1 + str_2 + str_3

    # Dual quaternion addition.
    def __add__(self, _):
        sum_r = self.real + _.real
        sum_d = self.dual + _.dual
        return DualQuaternion(sum_r.Q, sum_d.Q)

    # Dual quaternion subtraction.
    def __sub__(self, _):
        diff_r = self.real - _.real
        diff_d = self.dual - _.dual
        return DualQuaternion(diff_r.Q, diff_d.Q)

    # Dual quaternion multiplication.
    def __mul__(self, _):
        prod_r = self.real * _.real
        prod_d = self.dual*_.real + self.real*_.dual
        return DualQuaternion(prod_r.Q, prod_d.Q)

    # Power of a dual quaternion.
    def __pow__(self, n):
        # Get axis, angle, and P.
        T = self.trans_mat()
        P = T[0:3, 3]
        q = self.real.Q
        theta = 2*m.acos(q[0])
        const = m.sin(theta/2)
        u = np.array([q[1], q[2], q[3]])/const

        # Calculate variables for alternative representation.
        d = np.dot(P, u)
        M = 0.5*(np.cross(P, u) + (P - d*u)*(1/m.tan(theta/2)))
        u_bar = DualVector(u, M)

        # Compute cos and sin terms.
        const_1 = n*theta/2
        const_2 = n*d/2 
        cos = DualNumber(m.cos(const_1), -const_2*m.sin(const_1))
        sin = DualNumber(m.sin(const_1), const_2*m.cos(const_1))

        # Compute dual quaternion.
        D_1 = u_bar.DN_1 * sin
        D_2 = u_bar.DN_2 * sin
        D_3 = u_bar.DN_3 * sin
        D = [cos, D_1, D_2, D_3]
        return DualQuaternion(D)


    # Magnitude of a dual quaternion.
    def magnitude(self):
        return self.real.magnitude()

    # Convert to unit dual quaternion.
    def norm(self):
        mag = self.magnitude()
        norm_r = self.real / mag
        norm_d = self.dual / mag
        return DualQuaternion(norm_r.Q, norm_d.Q)

    # Conjugate of dual quaternion.
    def conjugate(self):
        conj_r = self.real.conjugate()
        conj_d = self.dual.conjugate()
        return DualQuaternion(conj_r.Q, conj_d.Q)

    # Associated transformation matrix of dual quaternion.
    def trans_mat(self):
        R = self.real.rot_mat()
        P = 2 * (self.dual * self.real.conjugate()).Q
        T = np.zeros((4, 4))
        T[0:3, 0:3] = R
        T[0:3, 3] = P[1:4]
        T[3, 3] = 1
        return T
        
    # Screw linear interpolation.
    def ScLERP(self, D_f, tau):
        return (self*((self.conjugate() * D_f)**tau))

    # Convert to position & orientation quaternion concatination representation.
    def gamma(self):
        T = self.trans_mat()
        arr = np.zeros(7)
        arr[0:3] = T[0:3, 3]
        arr[3:8] = self.real.Q
        return arr