#ifndef _BAXTER_H_
#define _BAXTER_H_

#include<iostream>
#include<vector>

class Baxter {
   public:
      double pi = 3.14159265;
      
      int joints = 7;
      
      double l0 = 270.35;
      double l1= 69;
      double l2 = 364.59;
      double l3 = 69;
      double l4 = 374.29;
      double l5 = 10;
      double l6 = 374.42;
      double l7 = 229.53;
      
      Matrix<double, 1, 7> thetas {0, 0, 0, 0, 0, 0, 0};
   
   
      Matrix<double, 7, 3> axis_joints {
   	   {0, 0, 1}, 
   	   {-1 / sqrt(2), 1 / sqrt(2), 0},
   	   {1 / sqrt(2), 1 / sqrt(2), 0},
   	   {-1 / sqrt(2), 1 / sqrt(2), 0},
   	   {1 / sqrt(2), 1 / sqrt(2), 0},
  	   {-1 / sqrt(2), 1 / sqrt(2), 0},
   	   {1 / sqrt(2), 1 / sqrt(2), 0},
   	   };  
   	
      Matrix<double, 7, 3> q_joints {
      {0, 0, 0},
      {l1 * cos(pi/4), l1 * sin(pi/4), l0},
      {0, 0, l0},
      {(l1 + l2) * cos(pi/4), (l2 + l3) * sin(pi/4), l0-l3},
      {(l1 + l2) * cos(pi/4), (l2 + l3) * sin(pi/4), l0-l3},
      {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5},
      {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5}
      };
      
      // NOT DONE
      Matrix<double, 7, 3> p_links {
      {0, 0, l0/2},
      {(l1 + l2/2) * cos(pi/4), (l1 + l2/2) * sin(pi/4), l0},
      {(l1 + l2/2) * cos(pi/4), (l1 + l2/2) * sin(pi/4), l0},
      {(l1 + l2) * cos(pi/4), (l2 + l3) * sin(pi/4), l0-l3},
      {(l1 + l2) * cos(pi/4), (l2 + l3) * sin(pi/4), l0-l3},
      {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5},
      {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5}
      };

};

#endif
