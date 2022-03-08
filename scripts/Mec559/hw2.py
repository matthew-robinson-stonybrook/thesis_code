#!/usr/bin/env python

import numpy as np

rho = 0.1
L = 0.4
h = 0.01
T = 1
e = 0.25

ut = np.array([10, 10])

A = np.array([[1,0,0], [0,1,0],[0,0,1]])
Q = np.array([[0.001,0,0], [0,0.001,0],[0,0,0.005]])

# theta is the most previous theta state, or theta_t-1
def calc_B(theta):
  B = np.array([[(rho/2)*np.cos(theta), (rho/2)*np.cos(theta)],
  [(rho/2)*np.sin(theta), (rho/2)*np.sin(theta)],
  [rho/L, -rho/L]])
  
  return B

def calc_xt(x_t_1, ut, wt):
  B = calc_B(x_t_1[2])
  xt = A.dot(x_t_1) + B.dot(ut) + wt
  return xt
  
# Method 1
t = 0
n = 10
k = 5

x_pdf = []
w_pdf = []


# Initial n poses and k noises
for i in range(0,n):
  x = np.array([np.random.normal(0, 1), np.random.normal(0, 1), np.random.normal(0, 1)])
  x = Q.dot(x)
  x_pdf.append(x)
  
for j in range(0,k):
  w = np.array([np.random.normal(0, 1), np.random.normal(0, 1), np.random.normal(0, 1)])
  w = Q.dot(x)
  w_pdf.append(w)

while t <= 1:
  if (t % 0.25 <= 0.00001):
    w_pdf = []
    for j in range(0,k):
      w = np.array([np.random.normal(0, 1), np.random.normal(0, 1), np.random.normal(0, 1)])
      w = Q.dot(w)
      w_pdf.append(w)
      
    for x in x_pdf:
      pass
      
  else:
    for i in range(0,len(x_pdf)):
      xt = calc_xt(x_pdf[i], h*ut, 0)
      x_pdf[i] = xt 
  
  print(x_pdf[0]) 
  t += h

  
