# Import modules.
import math as m
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from manipdkin import manipdkin
from g_to_dual_quat import g_to_dual_quat
from SpatialManipJac import SpatialManipJac
from skew import skew


# Baxter arm class.
class BaxterArm:

    # Link lengths and joint types.
    link_lengths = np.array([m.sqrt(270.35**2 + 69**2), 370.82, 374.42, 229.525])/1000
    d = np.array([270.35, 69, 364.35, 69, 374.29, 10, 229.525])/1000
    type_joints = ['R', 'R', 'R', 'R', 'R', 'R', 'R']

    # Joint axis.
    w_1 = [0, 0, 1]
    w_2 = [1, 0, 0]
    w_3 = [0, 1, 0]
    w_4 = w_2
    w_5 = w_3
    w_6 = w_2
    w_7 = w_3
    axis_joints = np.transpose(np.array([w_1, w_2, w_3, w_4, w_5, w_6, w_7]))

    # Points on joint axis.
    q_S0 = [0, 0, d[0]]
    q_S1 = [0, d[1], d[0]]
    q_E0 = q_S1
    q_E1 = [0, d[1]+d[2], d[0]-d[3]]
    q_W0 = q_E1
    q_W1 = [0, d[1]+d[2]+d[4], d[0]-d[3]-d[5]]
    q_W2 = q_W1
    q_joints = np.transpose(np.array([q_S0, q_S1, q_E0, q_E1, q_W0, q_W1, q_W2]))

    # Reference configuration of EE.
    gst_0 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, d[1]+d[2]+d[4]+d[6]],
        [0, 0, 1, d[0]-d[3]-d[5]],
        [0, 0, 0, 1]
    ])

    # Initialize object.
    def __init__(self, angles=np.zeros(7)):
        self.angles = np.array(angles)
        self.g = None
        self.A = None
        self.gamma = None
        self.all_g = []

    # Represent robot parameters.
    def __repr__(self):
        self.update()
        str_1 = '---Baxter 7-DoF Robotic Arm---'
        str_2 = f'\nJoint Angles:\n{self.angles.round(2)}'
        str_3 = f'\nTransformation Matrix:\n{self.g.round(3)}'
        return str_1+str_2+str_3

    def update(self):
        self.g = self.dkin()
        self.A = g_to_dual_quat(self.g)
        self.gamma = self.A.gamma()

    # Forward kinematics.
    def dkin(self):
        return manipdkin(self.gst_0, self.axis_joints, self.q_joints, self.type_joints, self.angles)

    # Compute spatial manipulator jacobian.
    def spatial_jac(self):
        return SpatialManipJac(self.gst_0, self.axis_joints, self.q_joints, self.type_joints, self.angles)

    # Plot mechanism end effector position.
    def plot(self, path, animate, show):

        # Initialize plot.
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X Position [m]')
        ax.set_ylabel('Y Position [m]')
        ax.set_zlabel('Z Position [m]')
        plt.title('End-Effector Path Animation (Pose 1 to Pose 5)')
        ax.axes.set_xlim3d(left=-0.375, right=0.375) 
        ax.axes.set_ylim3d(bottom=0.125, top=1) 
        ax.axes.set_zlim3d(bottom=0, top=0.3)
        ax.view_init(elev=30, azim=45)

        # Create list of transformation matrices to plot.
        n_paths = len(self.all_g)
        g_list = []
        if path=='all':
            for i in range(0, n_paths):
                for j in range(0, len(self.all_g[i])):
                    g_list.append(self.all_g[i][j])
        elif path=='task':
            for i in range(1, n_paths):
                for j in range(0, len(self.all_g[i])):
                    g_list.append(self.all_g[i][j])
        else:
            for i in range(0, len(self.all_g[path])):
                g_list.append(self.all_g[path][i])
        size = len(g_list)
        
        # Function to draw frame of animation/static plot.
        def draw_frame(frame):
            for a in range(frame):
                ax.scatter(g_list[a][0, 3], g_list[a][1, 3], g_list[a][2, 3], c='red', s=10)
        
        # Render static plot or animation.
        if animate:
            anim = FuncAnimation(fig, draw_frame, frames=np.arange(0, size, 10), save_count = 10000)
            anim.save('path3.gif', fps=20)
            print('Save successful!')
        else:
            draw_frame(size)

        # Show plot.
        if show:
            plt.show()

    # Compute B matrix.
    def B_mat(self):
        # Compute transformation matrix, dual quaternion, and orientation quaternion.
        g = self.dkin()
        P = g[0:3, 3]
        A = g_to_dual_quat(g)
        q = A.real.Q

        # Compute jacobians.
        J_s = self.spatial_jac()
        J_1 = np.array([
            [-q[1], q[0], q[3], -q[2]],
            [-q[2], -q[3], q[0], q[1]],
            [-q[3], q[2], -q[1], q[0]]
        ])
        J_2 = np.zeros((6, 7))
        J_2[0:3, 0:3] = np.identity(3)
        J_2[0:3, 3:7] = 2*np.matmul(skew(P), J_1)
        J_2[3:6, 3:7] = 2*J_1

        # Compute and return B.
        J_s_trans = np.transpose(J_s)
        temp_1 = np.linalg.inv(np.matmul(J_s, J_s_trans))
        temp_2 = np.matmul(temp_1, J_2)
        return np.matmul(J_s_trans, temp_2)

    # Motion planning algorithm.
    def motion_plan(self, g_0, g_f):
        # Initialize configuration representation.
        A_0 = g_to_dual_quat(g_0)
        A_f = g_to_dual_quat(g_f)
        gamma_0 = A_0.gamma()
        gamma_f = A_f.gamma()
        g_bar_list = [g_0]
        A_list = [A_0]
        gamma_list = [gamma_0]
        angle_list = [self.angles]

        # Initialize distance checking.
        q_f = A_f.real.Q
        p_f = np.zeros(3)
        for i in range(3):
            p_f[i] = g_f[i, 3]

        # Initialize algorithm parameters.
        tau = 0.01
        beta = 1
        i = 0
        run = True

        # Run algorithm until an exit condition is met.
        while run:

            # Interpolate current and final pose w/ ScLERP and convert to gamma.
            A_list.append(A_list[-1].ScLERP(A_f, tau))
            gamma_list.append(A_list[-1].gamma())

            # Compute new angles.
            B = self.B_mat()
            gamma_diff = gamma_list[-1] - gamma_list[-2]
            angle_list.append(angle_list[-1] + beta*np.matmul(B, gamma_diff))
            max_angle_diff = np.max(np.abs(angle_list[-1] - angle_list[-2]))

            # Recalculate pose of robot with computed approximate angles.
            self.angles = angle_list[-1]
            self.update()
            g_bar_list.append(self.g)
            
            # Compute euclidean distance between EE positions.
            p = np.zeros(3)
            for j in range(3):
                p[j] = self.g[j, 3]
            dist = m.sqrt((p_f[0]-p[0])**2 + (p_f[1]-p[1])**2 + (p_f[2]-p[2])**2)

            # Compute euclidean distance between rotation quaternions.
            q = A_list[-1].real.Q
            mag_1 = np.abs(np.linalg.norm(q-q_f))
            mag_2 = np.abs(np.linalg.norm(q+q_f))
            phi = min(mag_1, mag_2)

            # Adjust tau.
            max_angle_diff = np.max(np.abs(self.angles - angle_list[-2]))
            if tau==1:
                exit_cond = 'tau=1'
                run = False
            elif max_angle_diff<0.01:
                tau *= 1.5
                if tau>1:
                    tau = 1
            
            # Exit loop if it has reached the final pose, or if it diverges.
            cond = 0.01
            if (phi<cond) and (dist<cond):
                exit_cond = 'converges'
                run = False
            elif i == 2000:
                print('Solution diverges!')
                exit_cond = 'diverges'
                run = False
            i += 1

        print('---Motion Plan Results---')
        print(f'Iterations: {i}')
        print(f'Exit Condition: {exit_cond}')
        self.g_list = g_bar_list
        self.angles_list = angle_list
        self.all_g.append(self.g_list)