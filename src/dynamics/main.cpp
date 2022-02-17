#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<sstream>
#include<time.h>

#include "manips/Baxter.h"
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
   baxter_ptr->thetas = {0, 0, 0, 0, 0, 0, 0};
   baxter_ptr->theta_dots = {pi/40, pi/40, pi/40, pi/40, pi/40, pi/40, pi/40};
   baxter_ptr->theta_dots = {0, 0, 0, 0, 0, 0, 0};
   
   // Robot dynamics instance using baxter instance pointer
   Robot_Dynamics baxter_dynamics(baxter_ptr);
   
   // All tests
   baxter_dynamics.calc_spatial_jac();
   baxter_dynamics.calc_spatial_jac_dot();
   baxter_dynamics.calc_analytic_jac();
   baxter_dynamics.calc_mass_matrix();
   baxter_dynamics.calc_coriolis_matrix();
   baxter_dynamics.calc_gravity_term();
   baxter_dynamics.calc_potential_energy();
   baxter_dynamics.calc_kinetic_energy();
   baxter_ptr->set_joint_path(baxter_to_handle_csv);
   
   // Baxter Impedence Controller
   Controller baxter_ic = Controller(&baxter_dynamics);
   baxter_ic.calc_control_input();
   
   cout << "Control Input: " << endl;
   cout << baxter_ic.y << endl;
   
   cout << "Control Torque: " << endl;
   cout << baxter_ic.u << endl;
   /*
   cout << "Spatial: " << endl;
   cout << baxter_dynamics.spatial_jac << endl;
   cout << "Analytic: " << endl;
   cout << baxter_dynamics.analytic_jac << endl;
   cout << "Spatial Dot: " << endl;
   cout << baxter_dynamics.spatial_jac_dot << endl;
   Matrix<double, 6, 1> vs = baxter_dynamics.spatial_jac * baxter_ptr->theta_dots;
   Matrix<double, 6, 1> va = baxter_dynamics.analytic_jac * baxter_ptr->theta_dots;
   cout << "Spatial Vel" << endl;
   cout << vs << endl;
   cout << "Analytic vel" << endl;
   cout << va << endl;
   */ 
   
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

   
