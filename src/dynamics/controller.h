#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
#include "Baxter.h"
#include "manip.h"
#include "dynamics.h"

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
   //jac_pseudo_inverse = 
}

void Controller::calc_jac_pseudo_inv() {
   const int js = dynamics_ptr->manip_ptr->get_joints();
   Matrix<double, 6, joints> jac = dynamics_ptr->spatial_manip_jac;
   jac_pseudo_inv = jac.transpose() * ((jac * jac.transpose()).inverse());
}

#endif
