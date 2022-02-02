import numpy as np
import math as m


def manipdkin_sp(manip, previous_joints):
    # Identity Matrix
    I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Creates a skew matrix for each axis
    skew_axis = []
    for axis in manip.axis_joints:
        skew_axis.append(np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]))

    transformation_matrices = []

    # Computes each g matrix
    for index in range(previous_joints):
        omega = manip.axis_joints[index]
        q = manip.q_joints[index]
        t = manip.thetas[index]
        # Revolute Joints
        if manip.type_joints[index] == "R":
            rot = I + skew_axis[index] * m.sin(t) + np.dot(skew_axis[index], skew_axis[index]) * (1 - m.cos(t))
            a = (I - rot)
            trans = np.dot(a, q)

            t1 = np.append(rot[0], trans[0])
            t2 = np.append(rot[1], trans[1])
            t3 = np.append(rot[2], trans[2])
            transformation = np.array([t1.tolist(), t2.tolist(), t3.tolist(), [0, 0, 0, 1]])

        # Prismatic
        elif manip.type_joints[index] == "P":
            transformation = np.array([[1, 0, 0, omega[0] * t], [0, 1, 0, omega[1] * t], [0, 0, 1, omega[2] * t], [0, 0, 0, 1]])

        transformation_matrices.append(transformation)

    transformation_matrices.append(manip.gst0)
    # Muitiplies all g matrices to obtain final transformation
    g_final = np.identity(4)
    for transformation_matrix in transformation_matrices:
        g_final = np.dot(g_final, transformation_matrix)

    p_final = g_final[0:3, 3]
    return p_final, g_final


# Creates Baxter at the initial config, iterates through the motion planner method until final position and orientaiton
# is achieved, and then plots the path and final config of baxter
def plot_manip(manip, graph, title, color):
    # Baxter's first link starts at origin
    points = [[0, 0, 0]]

    # Iterate through points defined for baxter links and run each point through manipdkin_specified_point (manipdkin_sp)
    # manipdkin_sp is same as manipdkin except assumes point has followed a certain number of joint axes
    for index in range(1, len(manip.initial_config_points)):
        point = manip.initial_config_points[index]
        g0 = [[1, 0, 0, point[0]], [0, 1, 0, point[1]], [0, 0, 1, point[2]], [0, 0, 0, 1]]
        points.append(manipdkin_sp(manip, index)[0])

    #Plot the initial config using points just calculated
    points = np.array(points)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    graph.plot3D(x, y, z, color, label = title)
    
    '''
    g_x_axis = [[1, 0, 0, x_axis[0]], [0, 1, 0, x_axis[1]], [0, 0, 1, x_axis[2]], [0, 0, 0, 1]]
    g_y_axis = [[1, 0, 0, y_axis[0]], [0, 1, 0, y_axis[1]], [0, 0, 1, y_axis[2]], [0, 0, 0, 1]]
    g_z_axis = [[1, 0, 0, z_axis[0]], [0, 1, 0, z_axis[1]], [0, 0, 1, z_axis[2]], [0, 0, 0, 1]]

    # Plot x, y, and z axes of initial end-effector to show orientation
    x_axis_initial = manipdkin_sp(g_x_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    y_axis_initial = manipdkin_sp(g_y_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    z_axis_initial = manipdkin_sp(g_z_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_initial[0]], [y[-1], x_axis_initial[1]], [z[-1], x_axis_initial[2]], "red", label = 'End-Effector x-axis')
    ax.plot3D([x[-1], y_axis_initial[0]], [y[-1], y_axis_initial[1]],  [z[-1], y_axis_initial[2]], "green", label = 'End-Effector y-axis')
    ax.plot3D([x[-1], z_axis_initial[0]], [y[-1], z_axis_initial[1]], [z[-1], z_axis_initial[2]], "blue", label = 'End-Effector z-axis')
    '''
    
def motion_iterator(motion_planner, graph=0):
    
    if (graph != 0):
        plot_manip(motion_planner.manip, graph, "Initial Config", 'yellow')
        
    # All data points for motion path to be plotted
    xdata = []
    ydata = []
    zdata = []

    # While value "r" is greater than a certain value, iterate through iterative motion planning algorithm
    # "r" is root-sum-square difference between each element of the current and desired transformation matrix
    r = 100
    i = 0
    while abs(r) >= 0.01:
        for b in range(0, int(1 / motion_planner.beta)):
            xdata.append(motion_planner.g[0, 3])
            ydata.append(motion_planner.g[1, 3])
            zdata.append(motion_planner.g[2, 3])

            motion_planner.iteration()
            motion_planner.update_values()
            motion_planner.joint_path.append(motion_planner.manip.thetas)

            # Find the current error between the rotation matrices
            r = 0
            for row in range(0,3):
                for column in range(0,4):
                    r += (motion_planner.gstf[row][column] - motion_planner.g[row][column]) ** 2

            r = m.sqrt(r)
            i += 1
    
    print(i)
    for joint in range(0,7):
        angle = motion_planner.config[joint]
        if not motion_planner.manip.joint_ranges[joint][0] <= angle <= motion_planner.manip.joint_ranges[joint][1]:
            print('WARNING! BAXTER JOINT ' + str(joint) + ' IS OUT OF RANGE!')

    # Plot data points for motion path
    if (graph != 0):
      graph.scatter3D(xdata, ydata, zdata, cmap='Greens')
      plot_manip(motion_planner.manip, graph, "Final Config",  'orange')

    '''
    # Plot x, y, and z axes of final end-effector to show orientation
    x_axis_final = manipdkin_sp(g_x_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    y_axis_final = manipdkin_sp(g_y_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    z_axis_final = manipdkin_sp(g_z_axis, manip.axis_joints, manip.q_joints, manip.type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_final[0]], [y[-1], x_axis_final[1]], [z[-1], x_axis_final[2]], "red")
    ax.plot3D([x[-1], y_axis_final[0]], [y[-1], y_axis_final[1]],  [z[-1], y_axis_final[2]], "green")
    ax.plot3D([x[-1], z_axis_final[0]], [y[-1], z_axis_final[1]], [z[-1], z_axis_final[2]], "blue")
   '''
