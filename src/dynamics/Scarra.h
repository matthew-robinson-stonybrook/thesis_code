#ifndef _SCARRA_H_
#define _SCARRA_H_

#include<iostream>
#include<vector>

#include "manip.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Scarra: public Manip{
   public:
      Scarra();
      int joints = 3;
      
      // Link and motor inertia tensors
      
      double Ix1 = 1.0;
      double Iy1 = 1.0;
      double Iz1 = 1.0;
      Matrix3d Il1 {
         {Ix1, 0, 0},
         {0, Iy1, 0},
         {0, 0, Iz1}
      };
      
      double Ix2 = 1.0;
      double Iy2 = 1.0;
      double Iz2 = 1.0;
      Matrix3d Il2 {
         {Ix2, 0, 0},
         {0, Iy2, 0},
         {0, 0, Iz2}
      };
      
      double Ix3 = 1.0;
      double Iy3 = 1.0;
      double Iz3 = 1.0;
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
      MatrixXd p_links{
         {0,0,r0},
         {0,r1,l0},
         {0,l1+r2,l0}
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
      virtual MatrixXd get_p_links() {return p_links;}
      
      MatrixXd reference_twist_coords();
      Matrix3d reference_mass_matrix();
};

Scarra::Scarra() {
   cout << "Scarra Contructor" << endl;
}

MatrixXd Scarra::reference_twist_coords(){
   Matrix<double,3,6> twist_coords {
      {0,0,0,0,0,1},
      {0,-l0,0,-1,0,0},
      {0, -l0, l1, -1, 0, 0}
   };
   return twist_coords;
}

Matrix3d Scarra::reference_mass_matrix() {
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


#endif
