#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>
#include<time.h>

#include "runge_kutta.h"
#include "../manips/Baxter.h"
#include "../dynamics.h"

#include "../../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;

int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   clock_t tStart = clock();
   
   // Define the baxter object, the 7 joint angle values for testing
   Baxter *baxter_ptr = new Baxter();
   baxter_ptr->thetas = {pi/2, -pi/8, pi/4, -pi/6, 0.1, 0.1, 0.1};
   //baxter_ptr->thetas = {0, 0, 0, 0, 0, 0, 0};
   baxter_ptr->theta_dots = {pi/40, pi/40, pi/40, pi/40, pi/40, pi/40, pi/40};
   //baxter_ptr->theta_dots = {0, 0, 0, 0, 0, 0, 0};
   
   // Robot dynamics instance using baxter instance pointer
   Robot_Dynamics baxter_dynamics(baxter_ptr);
   
   // Iteration step size
   double h = 0.005;
   double t = 0;
   double desired_t = 2;
   
   // csv file to write results to
   string csv_file {"results/02_27_2022/RK_V1.csv"};
   ofstream myfile;
   myfile.open(csv_file);
   myfile << "02/27/2022 Test 1,h=0.005,t=2 seconds\n";
   myfile << "Time,Total Energy,q1,q2,q3,q4,q5,q6,q7,q1d,q2d,q3d,q4d,q5d,q6d,q7d\n";

   double E;
   // Runge-Kutta method for solving for "y_k+1"
   while(t < desired_t) {
      // q1 and q2 are current state (y_k) of system
      Matrix<double, 7, 1> q1 = baxter_ptr->thetas;
      Matrix<double, 7, 1> q2 = baxter_ptr->theta_dots;
      
      // q_kplus is using euler method to solve for 'next step'
      // f(tk + hk, yk + hk * f(tk, yk))
      vector<MatrixXd> q_prime = rk::f(q1, q2, &baxter_dynamics);
      Matrix<double, 7, 1> q1_kplus = q1 + h * q_prime.at(0);
      Matrix<double, 7, 1> q2_kplus = q2 + h * q_prime.at(1);
      
      // Set baxter_ptr joints to q_kplus for calculation
      baxter_ptr->thetas = q1_kplus;
      baxter_ptr->theta_dots = q2_kplus;
      vector<MatrixXd> q_prime_plus = rk::f(q1_kplus, q2_kplus, &baxter_dynamics);
      
      // y_k+1
      Matrix<double, 7, 1> q1_kp1 = q1 + h * 0.5 * (q_prime.at(0) + q_prime_plus.at(0));
      Matrix<double, 7, 1>q2_kp1 = q2 + h * 0.5 * (q_prime.at(1) + q_prime_plus.at(1));
      
      // Rewrite baxter pointer joint pos and vels for next iteration
      baxter_ptr->thetas = q1_kp1;
      baxter_ptr->theta_dots = q2_kp1;
      
      baxter_dynamics.calc_mass_matrix();
      baxter_dynamics.calc_kinetic_energy();
      baxter_dynamics.calc_potential_energy();
      double E = baxter_dynamics.T + baxter_dynamics.V;
      t += h;
      cout << "t: " << t << endl;
      
      // Writing to csv file
      // First column: Total Energy (E)
      myfile << to_string(t) << ",";
      myfile << to_string(E) << ",";
      
      // Second-Eigth Columns: Joint Positions (q1)
      //myfile << to_string(q1(0)) << "," << to_string(q1(1)) << to_string(q1(2)) << "," << to_string(q1(3)) << "," << to_string(q1(4)) << "," << to_string(q1(5)) << "," << to_string(q1(6)) << ",";
      // Ninth-Whatever Columns: Joint Velocities (q2)
      //myfile << to_string(q2(0)) << "," << to_string(q2(1)) << to_string(q2(2)) << "," << to_string(q2(3)) << "," << to_string(q2(4)) << "," << to_string(q2(5)) << "," << to_string(q2(6)) << ",";
      myfile << q1(0) << "," << q1(1) << "," << q1(2) << "," << q1(3) << "," << q1(4) << "," << q1(5) << "," << q1(6) << ",";
      myfile << q2(0) << "," << q2(1) <<  "," << q2(2) << "," << q2(3) << "," << q2(4) << "," << q2(5) << "," << q2(6) << ",";
      // New Line
      myfile << "\n";      
   }

   myfile.close();
   
   double ellapsed_time_us = ((clock() - tStart) * 1000000) / CLOCKS_PER_SEC;
   if(ellapsed_time_us <= 999) {
      cout << "TIME: " << ellapsed_time_us << "us" << endl;
   }
   if(ellapsed_time_us > 999 && ellapsed_time_us <= 999999) {
      cout << "TIME: " << ellapsed_time_us / 1000 << "ms" << endl;
   }
   if(ellapsed_time_us > 999999) {
      cout << "TIME: " << ellapsed_time_us / 1000000 << "s" << endl;
   }
   return 0;
}

   
