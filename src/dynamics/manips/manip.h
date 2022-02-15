#ifndef _MANIP_H_
#define _MANIP_H_

#include<iostream>
#include<vector>

#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Manip {
   public:
      int joints;
      
      // Link and motor inertia tensors
      vector <Matrix3d> Ils;
      vector <Matrix3d> Ims;
      
      // Link and motor masses
      vector <double> mls;
      vector <double> mms;
      vector <double> krs;
          
      MatrixXd thetas;
      MatrixXd theta_dots;
      MatrixXd theta_ddots;
      
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd p_links;
      
      Matrix4d gst0;
      
      
      virtual int get_joints()=0;;
      
      // Link and motor inertia tensors
      virtual vector <Matrix3d> get_Ils()=0;
      virtual vector <Matrix3d> get_Ims()=0;
      
      // Link and motor masses
      virtual vector <double> get_mls()=0;
      virtual vector <double> get_mms()=0;
      virtual vector <double> get_krs()=0;
          
      virtual MatrixXd get_thetas()=0;
      virtual MatrixXd get_theta_dots()=0;
      virtual MatrixXd get_theta_ddots()=0;
      
      virtual MatrixXd get_axis_joints()=0;
      virtual MatrixXd get_q_joints()=0;
      virtual MatrixXd get_p_links()=0;

      virtual Matrix4d get_gst0()=0;
      
      Manip();
      virtual ~Manip() {cout << "Manip:: destructor" << endl;}
      
      
};

Manip::Manip() {
   cout << "Manip constructor" << endl;
}


#endif
