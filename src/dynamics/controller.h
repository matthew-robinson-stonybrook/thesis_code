#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
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
      
      // End-effector position/orientation vector (xe) 
      // Desired pos/or vector (xd)
      // The rest follow same logic (x_dot and x_ddot)
      Matrix<double, 6, 1> xe {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xe_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xe_ddot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_ddot {0, 0, 0, 0, 0, 0};
      
      // End effector wrenches
      Matrix<double, 6, 1> he {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> ha {0, 0, 0, 0, 0, 0};
      
      // Joints
      // qe is current joint configuration 
      // qd is desired joint configuration
      MatrixXd qe;
      MatrixXd qd;
      MatrixXd qe_dot;
      MatrixXd qd_dot;
      MatrixXd qe_ddot;
      MatrixXd qd_ddot;
      
      // Control input
      MatrixXd y;
      MatrixXd u;
      void calc_control_input();
      void calc_control_torque();
};

Controller::Controller(Robot_Dynamics* ptr) : dynamics_ptr{ptr} {
   cout << "Controller Contructor" << endl;
   manip_ptr = dynamics_ptr->manip_ptr;
   
   joints = manip_ptr->get_joints();
   y.resize(manip_ptr->get_joints(), 1);
   u.resize(manip_ptr->get_joints(), 1);
}

void Controller::calc_control_input() {
   //Retrieve joint pos, vel, acc
   qe = manip_ptr->get_thetas();
   qe_dot = manip_ptr->get_theta_dots();
   qe_ddot = manip_ptr->get_theta_ddots();
   
   // Forward kinematics to retrieve position and orientation vector
   dynamics_ptr->forward_kin();
   Matrix4d ge = dynamics_ptr->ge;
   Matrix3d R = ge(seq(0,2), seq(0,2));
   Vector3d p = ge(seq(0,2), 3);
   // Orientation quaternion
   Vector4d qr = quat_math::rot_to_quat(R);
   // Current configuration (position and orientation)
   xe = {p(0), p(1), p(2), qr(1), qr(2), qr(3)};
   
   // Velocity Kinematics
   dynamics_ptr->calc_analytic_jac();
   // End-effector velocity term
   xe_dot = dynamics_ptr->analytic_jac * qe_dot;
   
   // Acceleration term
   dynamics_ptr->calc_analytic_jac_dot();
   xe_ddot = dynamics_ptr->analytic_jac * qe_ddot + dynamics_ptr->analytic_jac_dot * qe_dot;
   
   //Error Terms
   Matrix<double, 6, 1> x = xd - xe;
   Matrix<double, 6, 1> x_dot = xd_dot - xe_dot;
   Matrix<double, 6, 1> x_ddot = xd_ddot - xe_ddot;
   
   dynamics_ptr->calc_analytic_jac_pseudo_inv();
   y = dynamics_ptr->analytic_jac_pseudo_inv * Md.inverse() * (Md * xd_ddot + Kd * x_dot + Kp * x - Md * dynamics_ptr->analytic_jac_dot * qe_dot - ha);
   
}


// Make sure to calculate dynamics matrices and terms first
void Controller::calc_control_torque() {
   MatrixXd B = dynamics_ptr->mass_matrix;
   MatrixXd C = dynamics_ptr->coriolis_matrix;
   MatrixXd G = dynamics_ptr->gravity_term;
   MatrixXd J = dynamics_ptr->spatial_jac;
   
   u = B * y + C * manip_ptr->get_theta_dots() + G + J.transpose() * he;
}

#endif
