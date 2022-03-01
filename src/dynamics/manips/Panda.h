#ifndef _PANDA_H_
#define _PANDA_H_

#include<iostream>
#include<vector>

#include "manip.h"
#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Panda: public Manip{
   public:
      const double pi = 3.14159265;
      int joints = 7;
      
      Panda();
      
      double l0 = 0.27035;
      double l1= 0.069;
      double l2 = 0.36459;
      double l3 = 0.069;
      double l4 = 0.37429;
      double l5 = 0.010;
      double l6 = 0.37442;
      double l7 = 0.22953;
      
      double d1 = 0.333;
      double d3 = 0.316;
      double a4 = 0.0825;
      double d5 = 0.384;
      double a7 = 0.088;
      double d8 = 0.107;

      double Il1xx = 0.70337;
      double Il1xy = -0.000139;
      double Il1xz = 0.006772;
      double Il1yy = 0.70661;
      double Il1yz = 0.019169;
      double Il1zz = 0.009117;
      
      Matrix3d Il1 {
      {Il1xx, Il1xy, Il1xz},
      {Il1xy, Il1yy, Il1yz},
      {Il1xz, Il1yz, Il1zz}
      };
      
      double Il2xx = 0.007962;
      double Il2xy = -0.003925;
      double Il2xz = 0.010254;
      double Il2yy = 0.02811;
      double Il2yz = 0.000704;
      double Il2zz = 0.025995;
      
      Matrix3d Il2 {
      {Il2xx, Il2xy, Il2xz},
      {Il2xy, Il2yy, Il2yz},
      {Il2xz, Il2yz, Il2zz}
      };
      
      double Il3xx = 0.037242;
      double Il3xy = -0.004761;
      double Il3xz = -0.011396;
      double Il3yy = 0.036155;
      double Il3yz = -0.012805;
      double Il3zz = 0.01083;
      
      Matrix3d Il3 {
      {Il3xx, Il3xy, Il3xz},
      {Il3xy, Il3yy, Il3yz},
      {Il3xz, Il3yz, Il3zz}
      };
                      
      double Il4xx = 0.025853;
      double Il4xy = 0.007796;
      double Il4xz = -0.001332;
      double Il4yy = 0.019552;
      double Il4yz = 0.008641;
      double Il4zz = 0.028323;
      
      Matrix3d Il4 {
      {Il4xx, Il4xy, Il4xz},
      {Il4xy, Il4yy, Il4yz},
      {Il4xz, Il4yz, Il4zz}
      };
                      
      double Il5xx = 0.035549;
      double Il5xy = -0.002117;
      double Il5xz = -0.004037
      double Il5yy = 0.029474;
      double Il5yz = 0.000229;
      double Il5zz = 0.008627;
      
      Matrix3d Il5 {
      {Il5xx, Il5xy, Il5xz},
      {Il5xy, Il5yy, Il5yz},
      {Il5xz, Il5yz, Il5zz}
      };
                      
      double Il6xx = 0.001964;
      double Il6xy = 0.000109;
      double Il6xz = -0.001158;
      double Il6yy = 0.004354;
      double Il6yz = 0.000341;
      double Il6zz = 0.005433;
      
      Matrix3d Il6 {
      {Il6xx, Il6xy, Il6xz},
      {Il6xy, Il6yy, Il6yz},
      {Il6xz, Il6yz, Il6zz}
      };
                      
      double Il7xx = 0.012516;
      double Il7xy = -0.000428;
      double Il7xz = -0.001196;
      double Il7yy = 0.010027;
      double Il7yz = -0.000741;
      double Il7zz = 0.004815;
      
      Matrix3d Il7 {
      {Il7xx, Il7xy, Il7xz},
      {Il7xy, Il7yy, Il7yz},
      {Il7xz, Il7yz, Il7zz}
      };
      
      // Link and motor inertia tensors
      vector <Matrix3d> Ils {Il1, Il2, Il3, Il4, Il5, Il6, Il7};
      vector <Matrix3d> Ims {Il1, Il2, Il3, Il4, Il5, Il6, Il7};
      
      // Link and motor masses
      vector <double> mls {4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 0.735522};
      vector <double> mms{mls};
      vector <double> krs {1,1,1,1,1,1,1};
                
      Matrix<double,7,1> thetas {0,0,0,0,0,0,0};
      Matrix<double,7,1> theta_dots {0,0,0,0,0,0,0};
      Matrix<double,7,1> theta_ddots {0,0,0,0,0,0,0};
      
      MatrixXd axis_joints {
   	   {0, 0, 1}, 
   	   {0, 1, 0},
   	   {0, 0, 1},
   	   {0, -1, 0},
   	   {0, 0, 1},
  	   {0, -1, 0},
   	   {0, 0, -1},
   	   };  
   	
      MatrixXd q_joints {
         {0, 0, 0},
         {0, 0, d1},
         {0, 0, 0},
         {a4, 0, d1 + d3},
         {0, 0, 0},
         {0, 0, d1 + d3 + d5},
         {a7, 0, d1 + d3 + d5}
      };
      
      Matrix<double, 7, 6> twist_coords;
      
      MatrixXd p_links {
         {0, 0, l0/2},
         {l1 * cos(pi/4), l1 * sin(pi/4), l0},
         {(l1 + l2/2) * cos(pi/4), (l1 + l2/2) * sin(pi/4), l0},
         {(l1 + l2) * cos(pi/4), (l1 + l2) * sin(pi/4), l0-l3},
         {(l1 + l2 + l4/2) * cos(pi/4), (l1 + l2 + l4/2) * sin(pi/4), l0-l3},
         {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5}, 
         {(l1 + l2 + l4 + l7/2) * cos(pi/4), (l1 + l2 + l4 + l7/2) * sin(pi/4), l0-l3-l5}
      };
      
      Matrix<double, 7, 3> p_motors {p_links};
      
      Matrix4d gst0 {
         {1, 0, 0, (l1 + l2 + l4 + l7) * cos(pi/4)},
         {0, -1, 0, (l1 + l2 + l4 + l7) * sin(pi/4)},
         {0, 0, -1, l0 - l3 - l5},
         {0, 0, 0, 1}
      };
      
      virtual ~Panda() {cout << "Panda:: destructor" << endl;}
      
      // Returns total joints
      virtual int get_joints() {return joints;}
      
      // Returns link and motor inertia tensors
      virtual vector <Matrix3d> get_Ils() {return Ils;}
      virtual vector <Matrix3d> get_Ims() {return Ims;}
      
      // Returns link and motor masses and transmission ratios
      virtual vector <double> get_mls() {return mls;}
      virtual vector <double> get_mms() {return mms;}
      virtual vector <double> get_krs() {return krs;}
      
      // Returns joint space position, velocity, and accel
      virtual MatrixXd get_thetas() {return thetas;}
      virtual MatrixXd get_theta_dots() {return theta_dots;}
      virtual MatrixXd get_theta_ddots() {return theta_ddots;}
      
      // Returns joint configs, vector to joint axes, and vec to link CoM
      virtual MatrixXd get_axis_joints() {return axis_joints;}
      virtual MatrixXd get_q_joints() {return q_joints;}
      virtual MatrixXd get_twist_coords() {return twist_coords;}
      
      virtual MatrixXd get_p_links() {return p_links;}
      
      // Return initial end-effector transformation matirx
      virtual Matrix4d get_gst0() {return gst0;}
      
      Matrix4d forward_kin(MatrixXd q);
      

};

Panda::Panda(){
   cout << "Panda:: constructor" << endl;
   Vector3d cross;
   Vector3d uj;
   Vector3d qj;
   int t;
   
   for (int joint = 0; joint < joints; joint++) {
      uj = axis_joints.row(joint);
      qj = q_joints.row(joint);
      cross = (-1 * uj).cross(qj);
      twist_coords(joint, seq(0,2)) = cross;
      twist_coords(joint, seq(3,5)) = uj;
   }
};

Matrix4d Panda::forward_kin(MatrixXd q) {
   Matrix4d ge = linalg::eye4;
   double t;
   for (int joint {0}; joint < joints; joint++) {
      t = q(joint);
      ge *= g::twist(twist_coords.row(joint), t);
   }
   ge *= get_gst0();
   return ge;
}

#endif
