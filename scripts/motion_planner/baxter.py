import math as m

class Baxter:
   def __init__(self):
      self.joints = 7
      self.l0 = 0.27035;
      self.l1= 0.069;
      self.l2 = 0.36459;
      self.l3 = 0.069;
      self.l4 = 0.37429;
      self.l5 = 0.010;
      self.l6 = 0.37442;
      self.l7 = 0.22953;
      
      self.axis_joints = [
      [0, 0, 1],
      [-1 / m.sqrt(2), 1 / m.sqrt(2), 0],
      [1 / m.sqrt(2), 1 / m.sqrt(2), 0],
      [-1 / m.sqrt(2), 1 / m.sqrt(2), 0],
      [1 / m.sqrt(2), 1 / m.sqrt(2), 0],
      [-1 / m.sqrt(2), 1 / m.sqrt(2), 0],
      [1 / m.sqrt(2), 1 / m.sqrt(2), 0]
      ]
      
      self.q_joints = [
      [0, 0, 0],
      [self.l1 * m.cos(m.pi/4), self.l1 * m.sin(m.pi/4), self.l0],
      [0, 0, self.l0],
      [(self.l1 + self.l2) * m.cos(m.pi/4), (self.l1 + self.l2) * m.sin(m.pi/4), self.l0-self.l3],
      [(self.l1 + self.l2) * m.cos(m.pi/4), (self.l1 + self.l2) * m.sin(m.pi/4), self.l0-self.l3],
      [(self.l1 + self.l2 + self.l4) * m.cos(m.pi/4), (self.l1 + self.l2 + self.l4) * m.sin(m.pi/4), self.l0-self.l3-self.l5],
      [(self.l1 + self.l2 + self.l4) * m.cos(m.pi/4), (self.l1 + self.l2 + self.l4) * m.sin(m.pi/4), self.l0-self.l3-self.l5]
      ] 
      
      self.type_joints = ["R", "R", "R", "R", "R", "R", "R"]
      
      # Initial position of Baxter where ALL thetas = 0
      self.gst0 = [[1/m.sqrt(2), 1/m.sqrt(2), 0, (self.l2 + self.l3 + self.l5 + self.l8) * m.cos(m.pi / 4)],
     [-1/m.sqrt(2), 1/m.sqrt(2), 0, (self.l2 + self.l3 + self.l5 + self.l8) * m.sin(m.pi / 4)],
     [0, 0, 1, self.l1 - self.l4 - self.l6],
     [0, 0, 0, 1]]
     
     self.thetas = [0,0,0,0,0,0,0]
     
     self.joint_ranges = [(-2.461, 0.89), (-2.147, 1.047), (-3.028, 3.038), (-0.052, 2.618), (-3.059, 3.059), (-1.571, 2.098), (-3.059, 3.059)]
   	
