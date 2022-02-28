#ifndef _RUNGE_KUTTA_H_
#define _RUNGE_KUTTA_H_

#include<iostream>
#include<vector>

#include "../dynamics.h"
#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

// rk is a namespace that has functions for performing runge-kutta
// numerical ode solving for manipulator systems
namespace rk {
   vector<MatrixXd> f1(MatrixXd q1, MatrixXd q2, Robot_Dynamics* dynamics_ptr) {
   MatrixXd q1_prime {q2};
   MatrixXd q2_prime;
   
   // Calculate and set mass and coriolis matrices
   dynamics_ptr->calc_pre_dynamics();
   dynamics_ptr->calc_mass_matrix();
   dynamics_ptr->calc_coriolis_matrix();
   MatrixXd mass = dynamics_ptr->mass_matrix;
   MatrixXd coriolis = dynamics_ptr->coriolis_matrix;
   
   q2_prime = -mass.inverse() * coriolis * q2;
   
   vector<MatrixXd> q_prime {q1_prime, q2_prime};
   return q_prime;
   }
   
   vector<MatrixXd> f2(MatrixXd q1, MatrixXd q2, Robot_Dynamics* dynamics_ptr) {
   MatrixXd q1_prime {q2};
   MatrixXd q2_prime;
   
   // Calculate and set mass and coriolis matrices
   dynamics_ptr->calc_pre_dynamics();
   dynamics_ptr->calc_mass_matrix();
   dynamics_ptr->calc_coriolis_matrix();
   dynamics_ptr->calc_gravity_term();
   
   MatrixXd mass = dynamics_ptr->mass_matrix;
   MatrixXd coriolis = dynamics_ptr->coriolis_matrix;
   MatrixXd grav = dynamics_ptr->gravity_term;
   
   q2_prime = - mass.inverse() * (coriolis * q2 + grav);
   
   vector<MatrixXd> q_prime {q1_prime, q2_prime};
   return q_prime;
   }
}

#endif
