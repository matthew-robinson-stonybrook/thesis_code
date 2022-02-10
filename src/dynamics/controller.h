#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
#include "Baxter.h"
#include "manip.h"
#include "dynamics.h"
#include "quat_math.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Controller {
   public:
      Controller(Robot_Dynamics*);
      Robot_Dynamics *dynamics_ptr;
      int joints;
      
      Matrix<double, 6, 6> Md {linalg::eye6};
      Matrix<double, 6, 6> Kd {linalg::eye6};
      Matrix<double, 6, 6> Kp {linalg::eye6};
      
      Matrix<double, 6, 1> x {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_ddot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_ddot {0, 0, 0, 0, 0, 0};
   
      MatrixXd y; 
      MatrixXd jac_pseudo_inv;
      
      void calc_jac_pseudo_inv();
      void calc_control_input();
};

Controller::Controller(Robot_Dynamics* ptr) : dynamics_ptr{ptr} {
   cout << "Controller Contructor" << endl;
   
   joints = dynamics_ptr->manip_ptr->get_joints();
   y.resize(dynamics_ptr->manip_ptr->get_joints(), 1);
   jac_pseudo_inv.resize(dynamics_ptr->manip_ptr->get_joints(), 6);
}

void Controller::calc_control_input() {
   Matrix4d ge = dynamics_ptr->forward_kin();
   Matrix3d R = ge(seq(0,2), seq(0,2));
   Vector3d p = ge(seq(0,2), 3);
   // Orientation quaternion
   Vector4d qr = quat_math::rot_to_quat(R);
   xd = {p(0), p(1), p(2), qr(1), qr(2), qr(3)};
}

void Controller::calc_jac_pseudo_inv() {
   MatrixXd jac = dynamics_ptr->spatial_manip_jac;
   jac_pseudo_inv = jac.transpose() * ((jac * jac.transpose()).inverse());
}

#endif
