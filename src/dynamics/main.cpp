#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>
#include<time.h>

#include "Baxter.h"
#include "mass_matrix2.h"
#include "dynamics.h"
#include "quat_math.h"
#include "linalg.h"
#include "transformation.h"
#include "controller.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;
string baxter_to_handle_csv {"../../csv_files/baxter_to_handle.csv"};
   
int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   clock_t tStart = clock();
   
   // Define the baxter object, the 7 joint angle values for testing
   Baxter *baxter_ptr = new Baxter();
   baxter_ptr->thetas = {pi/2, -pi/8, pi/4, -pi/6, 0.1, 0.1, 0.1};
   baxter_ptr->thetas = {pi/2, 0, 0, 0, 0, 0, 0};
   baxter_ptr->theta_dots = {pi/40, pi/40, pi/40, pi/40, pi/40, pi/40, pi/40};
   
   Robot_Dynamics baxter_dynamics(baxter_ptr);
   
   baxter_dynamics.calc_mass_matrix();
   baxter_dynamics.calc_coriolis_matrix();
   baxter_dynamics.calc_gravity_term();
   baxter_dynamics.calc_potential_energy();
   baxter_dynamics.calc_kinetic_energy();
   baxter_ptr->set_joint_path(baxter_to_handle_csv);
   
   // Baxter Impedence Controller
   Controller baxter_ic = Controller(baxter_ptr);
   
   Vector2d n1 {1,3};
   Vector2d n2 {2,5};
   Matrix<double, 3, 2> v1 {
      {0.1, 0.2},
      {0.3, 0.4},
      {0.5, 0.6}
   };
   Matrix<double, 3, 2> v2 {
      {0.7, 0.8},
      {0.9, 1.0},
      {1.1, 1.2}
   };
   
   double a = sqrt(pow(n1(0),2) + pow(v1(0,0),2) + pow(v1(1,0),2) + pow(v1(2,0),2));
   double b = sqrt(pow(n1(1),2) + pow(v1(0,1),2) + pow(v1(1,1),2) + pow(v1(2,1),2));
   double c = sqrt(pow(n2(0),2) + pow(v2(0,0),2) + pow(v1(1,0),2) + pow(v1(2,0),2));
   double d = sqrt(pow(n2(1),2) + pow(v2(0,1),2) + pow(v1(1,1),2) + pow(v1(2,1),2));
   
   Matrix<double, 4, 2> dq1{
      {n1(0)/a, n1(1)/b},
      {v1(0,0)/a, v1(0,1)/b},
      {v1(1,0)/a, v1(1,1)/b},
      {v1(2,0)/a, v1(2,1)/b}
   };

   Matrix<double, 4, 2> dq2{
      {n2(0)/c, n2(1)/d},
      {v2(0,0)/c, v2(0,1)/d},
      {v2(1,0)/c, v2(1,1)/d},
      {v2(2,0)/c, v2(2,1)/d}
   };
   
   Matrix4d g {
      {0.707, -0.707, 0, 5},
      {0.707, 0.707, 0, 7},
      {0, 0, 1, 9.3},
      {0, 0, 0, 1}
   };
   
   cout << "G to dual quat" << endl;
   cout << quat_math::g_to_dual_quat(g) << endl;
   
   double ellapsed_us = ((clock() - tStart) * 1000000) / CLOCKS_PER_SEC;
   if(ellapsed_us <= 999999) {
      cout << "TIME: " << ellapsed_us << "s e-6" << endl;
   }
   else {
      cout << "TIME: " << ellapsed_us/1000 << "s e-3" << endl;
   }
   return 0;
}

   
