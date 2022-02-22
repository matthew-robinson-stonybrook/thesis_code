#ifndef _THREE_LINK_H_
#define _THREE_LINK_H_

#include<iostream>
#include<vector>

#include "manip.h"
#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Three_link: public Manip{
   public:
      Three_link();
      int joints = 3;
      
      // Link and motor inertia tensors
      
      double Ix1 = 3.3;
      double Iy1 = 3.6;
      double Iz1 = 3.9;
      Matrix3d Il1 {
         {Ix1, 0, 0},
         {0, Iy1, 0},
         {0, 0, Iz1}
      };
      
      double Ix2 = 2.2;
      double Iy2 = 2.4;
      double Iz2 = 2.6;
      Matrix3d Il2 {
         {Ix2, 0, 0},
         {0, Iy2, 0},
         {0, 0, Iz2}
      };
      
      double Ix3 = 1.1;
      double Iy3 = 1.21;
      double Iz3 = 1.4;
      Matrix3d Il3 {
         {Ix3, 0, 0},
         {0, Iy3, 0},
         {0, 0, Iz3}
      };
      
      vector <Matrix3d> Ils {Il1, Il2, Il3};
      vector <Matrix3d> Ims {Il1, Il2, Il3};
      
      // Link and motor masses
      vector <double> mls {1.3,2.6,4.8};
      vector <double> mms {1.3,2.6,4.8};
      vector <double> krs {1,1,1};
          
      Matrix<double,3,1> thetas {0,0,0};
      Matrix<double,3,1> theta_dots {0,0,0};
      Matrix<double,3,1> theta_ddots {0,0,0};
      
      double r0 = 0.3;
      double l0 = 0.6;
      double r1 = 0.2;
      double l1 = 0.4;
      double r2 = 0.1;
      double l2 = 0.2;
      
      
      MatrixXd axis_joints {
         {0,0,1},
         {-1,0,0},
         {-1,0,0}
      };
      MatrixXd q_joints{
         {0,0,0},
         {0,0,l0},
         {0,l1,l0}
      };
      
      Matrix<double, 3, 6> twist_coords;
      
      MatrixXd p_links{
         {0,0,r0},
         {0,r1,l0},
         {0,l1+r2,l0}
      };
      
      Matrix4d gst0 {
         {1, 0, 0, 0},
         {0, 1, 0, l1 + l2},
         {0, 0, 1, l0},
         {0, 0, 0, 1}
      };
      
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
      
      MatrixXd reference_twist_coords();
      Matrix3d reference_mass_matrix();
      Matrix3d reference_coriolis_matrix();
      Vector3d reference_gravity_term();
};

Three_link::Three_link() {
   cout << "Three_link Contructor" << endl;
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
}

MatrixXd Three_link::reference_twist_coords(){
   Matrix<double,3,6> twist_coords {
      {0,0,0,0,0,1},
      {0,-l0,0,-1,0,0},
      {0, -l0, l1, -1, 0, 0}
   };
   return twist_coords;
}

Matrix3d Three_link::reference_mass_matrix() {
   double t1 = thetas(0);
   double t2 = thetas(1);
   double t3 = thetas(2);
   double m1 = mls.at(0);
   double m2 = mls.at(1);
   double m3 = mls.at(2);
   
   double m11 = Iy2 * pow(sin(t2),2) + Iy3 * pow(sin(t2+t3),2) + Iz1 + Iz2 * pow(cos(t2),2) + Iz3 * pow(cos(t2+t3),2) + m2 * pow(r1,2) * pow(cos(t2),2) + m3 * pow((l1*cos(t2) + r2 * cos(t2 + t3)),2);
   double m12 = 0;
   double m13 = 0;
   double m21 = 0;
   double m22 = Ix2 + Ix3 + m3 * pow(l1,2) + m2 * pow(r1,2) + m3 * pow(r2,2) + 2 * m3 * l1 * r2 * cos(t3);
   double m23 = Ix3 + m3 * pow(r2,2) + m3 * l1 * r2 * cos(t3);
   double m31 = 0;
   double m32 = Ix3 + m3 * pow(r2,2) + m3 * l1 * r2 * cos(t3);
   double m33 = Ix3 + m3 * pow(r2,2);
   
   Matrix3d mass {
      {m11, m12, m13},
      {m21, m22, m23},
      {m31, m32, m33}
   };
   return mass;
}

Matrix3d Three_link::reference_coriolis_matrix() {
   double t1 = thetas(0);
   double t2 = thetas(1);
   double t3 = thetas(2);
   double td1 = theta_dots(0);
   double td2 = theta_dots(1);
   double td3 = theta_dots(2);
   double m1 = mls.at(0);
   double m2 = mls.at(1);
   double m3 = mls.at(2);

   //Christoffel symbols
   // c11
   double T112 = (Iy2 - Iz2 - m2 * pow(r1,2)) * cos(t2)*sin(t2) + (Iy3 - Iz3) * cos(t2 + t3) * sin(t2 + t3) - m3 * (l1*cos(t2) + r2*cos(t2+t3)) * (l1*sin(t2) + r2*sin(t2+t3));
   //cout << "T112: " << T112 << endl;
   double T113 = (Iy3 - Iz3) * cos(t2+t3)*sin(t2+t3) - m3 * r2 * sin(t2+t3) * (l1 * cos(t2) + r2*cos(t2+t3));
   // c12
   double T121 = T112;
   // c13
   double T131 = T113;
   // c21
   double T211 = (Iz2 - Iy2 + m2 * pow(r1,2)) * cos(t2)*sin(t2) + (Iz3 - Iy3) * cos(t2 + t3) * sin(t2 + t3) + m3 * (l1*cos(t2) + r2*cos(t2+t3)) * (l1*sin(t2) + r2*sin(t2+t3));
   // c22
   double T223 = -l1 * m3 * r2 * sin(t3);
   // c23
   double T232 = T223;
   double T233 = T223;
   // c31
   double T311 = (Iz3 - Iy3) * cos(t2+t3)*sin(t2+t3) + m3 * r2 * sin(t2+t3) * (l1 * cos(t2) + r2*cos(t2+t3));
   // c32
   double T322 = l1 * m3 * r2 * sin(t3);
   
   double c11 = T112 * td2 + T113 * td3;
   double c12 = T121 * td1;
   double c13 = T131 * td1;
   double c21 = T211 * td1;
   double c22 = T223 * td3;
   double c23 = T232 * td2 + T233 * td3;
   double c31 = T311 * td1;
   double c32 = T322 * td2;
   double c33 = 0;
   Matrix3d coriolis_matrix {
      {c11, c12, c13},
      {c21, c22, c23},
      {c31, c32, c33}
   };
   return coriolis_matrix;
}

Vector3d Three_link::reference_gravity_term() {
   double t1 = thetas(0);
   double t2 = thetas(1);
   double t3 = thetas(2);
   double m1 = mls.at(0);
   double m2 = mls.at(1);
   double m3 = mls.at(2);
   
   double g = 9.81;
   
   double g1 = 0;
   double g2 = -g * ((m2 * r1 + m3 * (l1+r2)) * cos(t2) + m3 * r2 * cos(t2 + t3));
   //double g2 = -(m2*g*r1+m3*g*l1)*cos(t2) - m3*r2*cos(t2+t3);
   double g3 = -m3*g*r2*cos(t2+t3);
   
   Vector3d gravity_term {g1, g2, g3};
   return gravity_term;

}


#endif
