#!/usr/bin/env python

import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def set_joint(limb, joint_name, theta):
    joint_command = {joint_name: theta}
    limb.set_joint_positions(joint_command)

def set_joint_vel(limb, joint_name, delta):
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: current_position + delta}
    limb.set_joint_positions(joint_command)

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

  #Define the joint name list for both limbs (lj = ['left_s0', 'left_s1', 'left_e0', 'left_e1',
  #'left_w0', 'left_w1', 'left_w2'],
  lj = left.joint_names()
  rj = right.joint_names()
  
  while not rospy.is_shutdown():
    set_joint(left, lj[6], 1)


if __name__ == '__main__':
  try:
    robinson_joint_control_ex1()
  except rospy.ROSInterruptException:
    pass

