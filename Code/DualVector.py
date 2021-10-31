# Import modules.
import numpy as np
from DualNumber import DualNumber

# Dual vector data type.
class DualVector:

    # Initialize attributes.
    def __init__(self, arg_1=None, arg_2=None, arg_3=None):

        if not isinstance(arg_1, DualNumber):
            # Specified as two vectors.
            if arg_3==None:
                self.DN_1 = DualNumber(arg_1[0], arg_2[0])
                self.DN_2 = DualNumber(arg_1[1], arg_2[1])
                self.DN_3 = DualNumber(arg_1[2], arg_2[2])

            # Specified as three dual number lists.
            else:
                self.DN_1 = DualNumber(arg_1[0], arg_1[1])
                self.DN_2 = DualNumber(arg_2[0], arg_2[1])
                self.DN_3 = DualNumber(arg_3[0], arg_3[1])
        else:
            # Specified as three dual numbers.
            self.DN_1 = arg_1
            self.DN_2 = arg_2
            self.DN_3 = arg_3

    # Representation of a dual vector.
    def __repr__(self):
        str_1 = 'Dual Vector:'
        str_2 = f'\n    D_1 = ({self.DN_1.a}, {self.DN_1.b})'
        str_3 = f'\n    D_2 = ({self.DN_2.a}, {self.DN_2.b})'
        str_4 = f'\n    D_3 = ({self.DN_1.a}, {self.DN_1.b})'
        return str_1 + str_2 + str_3 + str_4


    # Dual vector addition.
    def __add__(self, other):
        sum_1 = self.DN_1 + other.DN_1
        sum_2 = self.DN_2 + other.DN_2
        sum_3 = self.DN_3 + other.DN_3
        return DualVector(sum_1, sum_2, sum_3)

    # Dual vector subtraction.
    def __sub__(self, other):
        diff_1 = self.DN_1 - other.DN_1
        diff_2 = self.DN_2 - other.DN_2
        diff_3 = self.DN_3 - other.DN_3
        return DualVector(diff_1, diff_2, diff_3)

    # Product of dual number and dual vector.
    def DN_times_DV(self, DN):
        prod_1 = DN * self.DN_1
        prod_2 = DN * self.DN_2
        prod_3 = DN * self.DN_3
        return DualVector(prod_1, prod_2, prod_3)

    # Dot product of two dual vectors.
    def dot(self, other):
        prod_1 = self.DN_1 * other.DN_1
        prod_2 = self.DN_2 * other.DN_2
        prod_3 = self.DN_3 * other.DN_3
        return prod_1 + prod_2 + prod_3

    # Cross product of two dual vectors.
    def cross(self, other):
        temp_1 = (self.DN_2*other.DN_3) - (self.DN_3*other.DN_2)
        temp_2 = (self.DN_3*other.DN_1) - (self.DN_1*other.DN_3)
        temp_3 = (self.DN_1*other.DN_2) - (self.DN_2*other.DN_1)
        return DualVector(temp_1, temp_2, temp_3)