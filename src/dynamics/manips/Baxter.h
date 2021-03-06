#ifndef _BAXTER_H_
#define _BAXTER_H_

#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>
#include<vector>

#include "manip.h"
#include "../transformation.h"
#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Baxter: public Manip{
   public:
      const double pi = 3.14159265;
      int joints = 7;
      
      Baxter();
      
      double l0 = 0.27035;
      double l1= 0.069;
      double l2 = 0.36459;
      double l3 = 0.069;
      double l4 = 0.37429;
      double l5 = 0.010;
      double l6 = 0.37442;
      double l7 = 0.22953;

      double Il1xx = 0.0470910226;
      double Il1xy = -0.00614870039;
      double Il1xz = 0.00012787556;
      double Il1yy = 0.0359598847;
      double Il1yz = -0.00078086899;
      double Il1zz = 0.03766976455;
      
      Matrix3d Il1 {
      {Il1xx, Il1xy, Il1xz},
      {Il1xy, Il1yy, Il1yz},
      {Il1xz, Il1yz, Il1zz}
      };
      
      double Il2xx = 0.0278859752;
      double Il2xy = -0.00018821993;
      double Il2xz = 0.000300963979;
      double Il2yy = 0.0207874929;
      double Il2yz = 0.00207675762;
      double Il2zz = 0.01175209419;
      
      Matrix3d Il2 {
      {Il2xx, Il2xy, Il2xz},
      {Il2xy, Il2yy, Il2yz},
      {Il2xz, Il2yz, Il2zz}
      };
      
      double Il3xx = 0.02661733557;
      double Il3xy = -0.00392189887;
      double Il3xz = 0.00029270634;
      double Il3yy = 0.0124800832;
      double Il3yz = -0.0010838933;
      double Il3zz = 0.02844355207;
      
      Matrix3d Il3 {
      {Il3xx, Il3xy, Il3xz},
      {Il3xy, Il3yy, Il3yz},
      {Il3xz, Il3yz, Il3zz}
      };
                      
      double Il4xx = 0.01318227876;
      double Il4xy = -0.00019663418;
      double Il4xz = 0.00036036173;
      double Il4yy = 0.0092685206;
      double Il4yz = 0.0007459496;
      double Il4zz = 0.00711582686;
      
      Matrix3d Il4 {
      {Il4xx, Il4xy, Il4xz},
      {Il4xy, Il4yy, Il4yz},
      {Il4xz, Il4yz, Il4zz}
      };
                      
      double Il5xx = 0.01667742825;
      double Il5xy = -0.00018657629;
      double Il5xz = 0.00018403705;
      double Il5yy = 0.0037463115;
      double Il5yz = 0.00064732352;
      double Il5zz = 0.01675457264;
      
      Matrix3d Il5 {
      {Il5xx, Il5xy, Il5xz},
      {Il5xy, Il5yy, Il5yz},
      {Il5xz, Il5yz, Il5zz}
      };
                      
      double Il6xx = 0.00700537914;
      double Il6xy = 0.00015348067;
      double Il6xz = -0.00044384784;
      double Il6yy = 0.0055275524;
      double Il6yz = -0.00021115038;
      double Il6zz = 0.00387607152;
      
      Matrix3d Il6 {
      {Il6xx, Il6xy, Il6xz},
      {Il6xy, Il6yy, Il6yz},
      {Il6xz, Il6yz, Il6zz}
      };
                      
      double Il7xx = 0.00081621358;
      double Il7xy = 0.00012844010;
      double Il7xz = 0.000189698911;
      double Il7yy = 0.00087350127;
      double Il7yz = 0.00010577265;
      double Il7zz = 0.00054941487; 
      
      Matrix3d Il7 {
      {Il7xx, Il7xy, Il7xz},
      {Il7xy, Il7yy, Il7yz},
      {Il7xz, Il7yz, Il7zz}
      };
      
      // Link and motor inertia tensors
      vector <Matrix3d> Ils {Il1, Il2, Il3, Il4, Il5, Il6, Il7};
      vector <Matrix3d> Ims {Il1, Il2, Il3, Il4, Il5, Il6, Il7};
      
      // Link and motor masses
      vector <double> mls {5.70044, 3.22698, 4.31272, 2.07206, 2.24665, 1.60979, 0.35093+0.19125};  
      vector <double> mms{5.70044, 3.22698, 4.31272, 2.07206, 2.24665, 1.60979, 0.35093+0.19125};
      vector <double> krs {1,1,1,1,1,1,1};
                
      Matrix<double,7,1> thetas {0,0,0,0,0,0,0};
      Matrix<double,7,1> theta_dots {0,0,0,0,0,0,0};
      Matrix<double,7,1> theta_ddots {0,0,0,0,0,0,0};
      
      vector <Matrix<double, 7, 1>> joint_path;
      
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
         {(l1 + l2) * cos(pi/4), (l1 + l2) * sin(pi/4), l0-l3},
         {(l1 + l2) * cos(pi/4), (l1 + l2) * sin(pi/4), l0-l3},
         {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5},
         {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5}
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
      
      Matrix<double, 7, 3> p_motors {
         {0, 0, 0},
         {l1 * cos(pi/4), l1 * sin(pi/4), l0},
         {(l1 + l2/4) * cos(pi/4), (l1 + l2/4) * sin(pi/4), l0},
         {(l1 + l2) * cos(pi/4), (l1 + l2) * sin(pi/4), l0-l3},
         {(l1 + l2 + l4/4) * cos(pi/4), (l1 + l2 + l4/4) * sin(pi/4), l0-l3},
         {(l1 + l2 + l4) * cos(pi/4), (l1 + l2 + l4) * sin(pi/4), l0-l3-l5}, 
         {(l1 + l2 + l4 + l7/4) * cos(pi/4), (l1 + l2 + l4 + l7/4) * sin(pi/4), l0-l3-l5}
      };
      
      Matrix4d gst0 {
         {1/sqrt(2), 1/sqrt(2), 0, (l1 + l2 + l4 + l7) * cos(pi/4)},
         {-1/sqrt(2), 1/sqrt(2), 0, (l1 + l2 + l4 + l7) * sin(pi/4)},
         {0, 0, 1, l0 - l3 - l5},
         {0, 0, 0, 1}
      };
      
      virtual ~Baxter() {cout << "Baxter:: destructor" << endl;}
      
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
      void set_joint_path(string);
      
};

Baxter::Baxter(){
   cout << "Baxter:: constructor" << endl;
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

void Baxter::set_joint_path(string file_name) {
   string line, theta;
   fstream file (file_name, ios::in);
   if(file.is_open()) {
      while(getline(file, line)) {
         stringstream str(line);
         
         Matrix<double, 7, 1> config {0, 0, 0, 0, 0, 0, 0};
         int joint {0};
         while(getline(str, theta, ',')) {
            config(joint) = stod(theta);
            joint += 1;
         }
         joint_path.push_back(config);
      }
   }
}

Matrix4d Baxter::forward_kin(MatrixXd q) {
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
