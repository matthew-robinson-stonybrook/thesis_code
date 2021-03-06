#include<iostream>
#include<time.h>

#include "manips/Three_link.h"
#include "dynamics.h"
#include "linalg.h"
#include "transformation.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;
   
int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   clock_t tStart = clock();
   
   Three_link *three_ptr = new Three_link();
   three_ptr->thetas = {pi/4, -pi/8, pi/4};
   //three_ptr->thetas = {0, 0, 0};
   three_ptr->theta_dots = {pi/16, pi/24, 0};
   
   Robot_Dynamics three_link_dynamics(three_ptr);
   
   three_link_dynamics.calc_spatial_jac();
   three_link_dynamics.calc_mass_matrix();
   three_link_dynamics.calc_coriolis_matrix();
   three_link_dynamics.calc_gravity_term();
   
   //cout << "Christoffel 112: " << scarra_dynamics.calc_christoffel(1, 1, 2) << endl;
   
   cout << "Reference Scarra Grav Term: " << endl;
   cout << three_ptr->reference_gravity_term() << endl;
   cout << "Calculated Scarra Grav Term (Robot_Dynamics): " << endl;
   cout << three_link_dynamics.gravity_term << endl;
   
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
