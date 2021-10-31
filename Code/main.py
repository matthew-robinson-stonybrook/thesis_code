#!/usr/bin/env python

import math as m
import numpy as np
from DualNumber import DualNumber
from DualVector import DualVector
from Quaternion import Quaternion
from DualQuaternion import DualQuaternion
from g_to_dual_quat import g_to_dual_quat
from BaxterArm import BaxterArm
    
def main():
    # Initialize arm.
    arm = BaxterArm()
    arm.angles = (m.pi/4)*np.ones(7)
    arm.update()

    # Compute desired poses.
    g_0 = arm.g
    g_1 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    g_2 = np.array([
        [0, 1, 0, 0.1],
        [1, 0, 0, 0.1],
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])


    # Run code.
    print('\n-----Pose 0-----')
    print(arm)
    arm.motion_plan(g_0, g_1)
    print('\n-----Pose 1-----')
    print(arm)
    arm.motion_plan(g_1, g_2)
    print('\n-----Pose 2-----')
    print(arm)


if __name__=="__main__":
    main()
