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
 
   
   double ellapsed_us = ((clock() - tStart) * 1000000) / CLOCKS_PER_SEC;
   if(ellapsed_us <= 999999) {
      cout << "TIME: " << ellapsed_us << "s e-6" << endl;
   }
   else {
      cout << "TIME: " << ellapsed_us/1000 << "s e-3" << endl;
   }
   return 0;
}

   
