# Import modules.
import numpy as np

# Dual number data type.
class DualNumber:

    # Initialize attributes.
    def __init__(self, a, b):
        self.a = a
        self.b = b

    # Representation of a dual number.
    def __repr__(self):
        str_1 = 'Dual Number:'
        str_2 = f'\n    a = {self.a}'
        str_3 = f'\n    b = {self.b}'
        return str_1 + str_2 + str_3


    # Dual number addition.
    def __add__(self, other):
        sum_a = self.a + other.a
        sum_b = self.b + other.b
        return DualNumber(sum_a, sum_b)

    # Dual number subtraction.
    def __sub__(self, other):
        diff_a = self.a - other.a
        diff_b = self.b - other.b
        return DualNumber(diff_a, diff_b)

    # Dual number multiplication.
    def __mul__(self, other):
            prod_a = self.a * other.a
            prod_b = (self.a * other.b) + (self.b * other.a)
            return DualNumber(prod_a, prod_b)
    
    # Dual number inverse.
    def inv(self):
        if self.a==0:
            print('Inverse of Dual Number DNE.')
        else:
            inv_a = (1/self.a)
            inv_b = -(1/self.a)*(self.b/self.a)
        return DualNumber(inv_a, inv_b)

    # Dual number conjugate.
    def conjugate(self):
        return DualNumber(self.a, -self.b)