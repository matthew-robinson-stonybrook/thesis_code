#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
import numpy as np

def set_joint(limb, joint_name, theta):
    joint_command = {joint_name: theta}
    limb.set_joint_positions(joint_command)

def set_joint_vel(limb, joint_name, delta):
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: current_position + delta}
    limb.set_joint_positions(joint_command)

# Sets joints to specified configuration defined by joint_positions
# Returns 1 if all joint errors meet desired errors condition and 0 if they dont
# Some "pos_index" variable should be defined outside function and added to function
# to keep track of joint position index
def iterate_joint_positions(limb, joint_positions, error_desired, pos_index):
    
   #Define the joint name list for specified limb (lj = ['left_s0', 'left_s1', 'left_e0', 'left_e1','left_w0', 'left_w1', 'left_w2'],
    jn = limb.joint_names()
    
    thetas_current1 = limb.joint_angles()

    thetas_current = np.zeros(7)
    for joint in range(7):
      thetas_current[joint] = thetas_current1[jn[joint]]

    for i in range(7):
      set_joint(limb, jn[i], joint_positions[pos_index][i])


    thetas_error = abs(thetas_current - joint_positions[pos_index])
    for joint_index in range(7):
      if thetas_error[joint_index] <= error_desired:
        if joint_index == 6: 
          return 1
        else:
          pass
      else:
        return 0


def robinson_joint_control_ex1():
  print("Initializing node... ")
  rospy.init_node("robinson_joint_control_ex1")
  print("Getting robot state... ")
  rs = baxter_interface.RobotEnable(CHECK_VERSION)
  init_state = rs.state().enabled

  def clean_shutdown():
      print("\nExiting example...")
      if not init_state:
          print("Disabling robot...")
          rs.disable()

  rospy.on_shutdown(clean_shutdown)

  print("Enabling robot... ")
  rs.enable()

  #Define both limb objects
  left = baxter_interface.Limb('left')
  right = baxter_interface.Limb('right')
  
  #thetas_desired = np.array([[0, 0, 0, 0, 0, 0, 0], [0, 0, 0.3, 0.4, 0.5, 0.6, 0.7]])
  thetas_desired = np.array([[0, 0, 0, 0, 0, 0, 0]])
  #thetas_desired = np.array([[-.5, -.3, .3, .5, .5, 1, .5]])

  error_desired = 0.02
  pos_index = 0

  done = False
  while not done and not rospy.is_shutdown():
    pos_index += iterate_joint_positions(left, thetas_desired, error_desired, pos_index)
    print(left._cartesian_pose)
    
    
if __name__ == '__main__':
  try:
    robinson_joint_control_ex1()
  except rospy.ROSInterruptException:
    pass

