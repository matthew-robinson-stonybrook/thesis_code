#include<iostream>
#include<time.h>
#include "mass_matrix2.h"
#include "dynamics.h"
#include "Baxter.h"
#include "linalg.h"
#include "transformation.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;

int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   clock_t tStart = clock();
   
   // Define the baxter object, the 7 joint angle values for testing
   // and a spatial jacobian object
   Baxter *baxter_ptr {nullptr};
   Baxter baxter;
   baxter_ptr = &baxter;
   baxter_ptr->thetas = {pi/2, -pi/8, pi/4, -pi/6, 0.1, 0.1, 0.1};
   baxter_ptr->theta_dots = {pi/40, pi/40, pi/40, pi/40, pi/40, pi/40, pi/40};
   //baxter_ptr->theta_dots = {0,0,0,0,0,0,0};
   
   Robot_Dynamics baxter_dynamics(baxter_ptr);
   /*
   baxter_dynamics.calc_link_adjusted_gmasss();
   baxter_dynamics.calc_mass_matrix();
   baxter_dynamics.calc_coriolis_matrix();
   baxter_dynamics.calc_potential_energy();
   baxter_dynamics.calc_kinetic_energy();
   */ 
   
   for(int t{0}; t<=1000; t++) {
      // Calculate EoM
      baxter_dynamics.calc_link_adjusted_gmasss();
      baxter_dynamics.calc_mass_matrix();
      baxter_dynamics.calc_coriolis_matrix();
      baxter_dynamics.calc_theta_ddot();
      
      // Calculate and display total energy
      baxter_dynamics.calc_potential_energy();
      baxter_dynamics.calc_kinetic_energy();
      
      cout << "KE " << endl;
      cout << baxter_dynamics.T << endl;
      
      cout << "Jointt Accel: " << endl;
      cout << baxter_ptr->theta_ddots << endl;
      
      baxter_ptr->theta_dots += (baxter_ptr->theta_ddots); 
      baxter_ptr->thetas += (baxter_ptr->theta_dots);
   }
   
   /*
   cout << "Mass Matrix: " << endl;
   cout << baxter_dynamics.mass_matrix << endl;
   cout << " " << endl;
   cout << "Joint Accelerations: " << endl;
   cout << theta_ddot << endl;
   cout << " " << endl;
   cout << "Resulting Joint Torques From Robot Dynamics: " << endl;
   cout << tau1 << endl;
   cout << " " << endl;
   cout << "Resulting Joint Torques From Mass Matrix2: " << endl;
   cout << tau2 << endl;
   cout << " " << endl;
   */
   double ellapsed_us = ((clock() - tStart) * 1000000) / CLOCKS_PER_SEC;
   cout << "TIME: " << ellapsed_us << "s e-6" << endl;
   return 0;
}
