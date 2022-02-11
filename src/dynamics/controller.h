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
      Robot_Dynamics *dynamics_ptr {nullptr};
      Manip *manip_ptr {nullptr};
      int joints;
      
      // Mass, Stiffness, and Damping matrices
      Matrix<double, 6, 6> Md {linalg::eye6};
      Matrix<double, 6, 6> Kd {linalg::eye6};
      Matrix<double, 6, 6> Kp {linalg::eye6};
      
      // Position/orientation vector and desired pos/or vector
      // The rest follow same logic (x_dot and x_ddot)
      Matrix<double, 6, 1> x {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_ddot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_ddot {0, 0, 0, 0, 0, 0};
      
      // Control input
      MatrixXd y;
      void calc_control_input();
};

Controller::Controller(Robot_Dynamics* ptr) : dynamics_ptr{ptr} {
   cout << "Controller Contructor" << endl;
   manip_ptr = dynamics_ptr->manip_ptr;
   
   joints = manip_ptr->get_joints();
   y.resize(manip_ptr->get_joints(), 1);
}

void Controller::calc_control_input() {
   dynamics_ptr->forward_kin();
   Matrix4d ge = dynamics_ptr->ge;
   Matrix3d R = ge(seq(0,2), seq(0,2));
   Vector3d p = ge(seq(0,2), 3);
   // Orientation quaternion
   Vector4d qr = quat_math::rot_to_quat(R);
   // Current configuration (position and orientation)
   x = {p(0), p(1), p(2), qr(1), qr(2), qr(3)};
   
   MatrixXd t_dots = manip_ptr->get_theta_dots();
   dynamics_ptr->calc_analytic_jac();
   x_dot = dynamics_ptr->analytic_jac * t_dots;
   
   //Error Terms
   Matrix<double, 6, 1> xe = xd - x;
   Matrix<double, 6, 1> xe_dot = xd_dot - x_dot;
   
   dynamics_ptr->calc_analytic_jac_pseudo_inv();
   //y = jac_pseudo_inv() * Md.inverse() * (Md * xd_ddot + Kd * xe_dot + Kp * xe - Md * analytic_jac_dot * 
   
}

#endif
