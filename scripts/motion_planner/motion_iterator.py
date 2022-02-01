# Creates Baxter at the initial config, iterates through the motion planner method until final position and orientaiton
# is achieved, and then plots the path and final config of baxter
def motion_iterator(motion_planner):
    # Baxter's first link starts at origin
    initial_points = [[0, 0, 0]]

    # Iterate through points defined for baxter links and run each point through manipdkin_specified_point (mdkp)
    # mdkp is same as manipdkin except assumes point has followed a certain number of joint axes
    for index in range(1, len(baxter_initial_config_points)):
        point = baxter_initial_config_points[index]
        g0 = [[1, 0, 0, point[0]], [0, 1, 0, point[1]], [0, 0, 1, point[2]], [0, 0, 0, 1]]
        initial_points.append(mdkp(g0, axis_joints, q_joints, type_joints, motion_planner.config, index)[0])

    #Plot the initial config using points just calculated
    initial_points = np.array(initial_points)
    x = initial_points[:, 0]
    y = initial_points[:, 1]
    z = initial_points[:, 2]
    ax.plot3D(x, y, z, 'yellow', label = 'Initial Position')

    g_x_axis = [[1, 0, 0, x_axis[0]], [0, 1, 0, x_axis[1]], [0, 0, 1, x_axis[2]], [0, 0, 0, 1]]
    g_y_axis = [[1, 0, 0, y_axis[0]], [0, 1, 0, y_axis[1]], [0, 0, 1, y_axis[2]], [0, 0, 0, 1]]
    g_z_axis = [[1, 0, 0, z_axis[0]], [0, 1, 0, z_axis[1]], [0, 0, 1, z_axis[2]], [0, 0, 0, 1]]

    # Plot x, y, and z axes of initial end-effector to show orientation
    x_axis_initial = mdkp(g_x_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    y_axis_initial = mdkp(g_y_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    z_axis_initial = mdkp(g_z_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_initial[0]], [y[-1], x_axis_initial[1]], [z[-1], x_axis_initial[2]], "red", label = 'End-Effector x-axis')
    ax.plot3D([x[-1], y_axis_initial[0]], [y[-1], y_axis_initial[1]],  [z[-1], y_axis_initial[2]], "green", label = 'End-Effector y-axis')
    ax.plot3D([x[-1], z_axis_initial[0]], [y[-1], z_axis_initial[1]], [z[-1], z_axis_initial[2]], "blue", label = 'End-Effector z-axis')

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
        if not baxter_joint_ranges[joint][0] <= angle <= baxter_joint_ranges[joint][1]:
            print('WARNING! BAXTER JOINT ' + str(joint) + ' IS OUT OF RANGE!')

    # Plot data points for motion path
    ax.scatter3D(xdata, ydata, zdata, cmap='Greens')

    # Determine final baxter config using same method as initial
    final_points = [[0,0,0]]
    for index in range(1,len(baxter_initial_config_points)):
        point = baxter_initial_config_points[index]
        g0 = [[1, 0, 0, point[0]], [0, 1, 0, point[1]], [0, 0, 1, point[2]], [0, 0, 0, 1]]
        final_points.append(mdkp(g0, axis_joints, q_joints, type_joints, motion_planner.config, index)[0])

    final_points = np.array(final_points)
    x = final_points[:, 0]
    y = final_points[:, 1]
    z = final_points[:, 2]
    ax.plot3D(x, y, z, 'orange', label = 'Final Position')

    # Plot x, y, and z axes of final end-effector to show orientation
    x_axis_final = mdkp(g_x_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    y_axis_final = mdkp(g_y_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    z_axis_final = mdkp(g_z_axis, axis_joints, q_joints, type_joints, motion_planner.config, 7)[0]
    ax.plot3D([x[-1], x_axis_final[0]], [y[-1], x_axis_final[1]], [z[-1], x_axis_final[2]], "red")
    ax.plot3D([x[-1], y_axis_final[0]], [y[-1], y_axis_final[1]],  [z[-1], y_axis_final[2]], "green")
    ax.plot3D([x[-1], z_axis_final[0]], [y[-1], z_axis_final[1]], [z[-1], z_axis_final[2]], "blue")
