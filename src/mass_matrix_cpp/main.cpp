#include<iostream>
#include "mass_matrix2.h"
#include "mass_matrix3.h"
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
   
   // Define the baxter object, the 7 joint angle values for testing
   // and a spatial jacobian object
   Baxter *baxter_ptr {nullptr};
   Baxter baxter;
   baxter_ptr = &baxter;
   baxter_ptr->thetas = {pi/4, 0, pi/8, 0, pi/4, 0, 0};
   baxter_ptr->theta_dots = {pi/40, 0, pi/80, 0, pi/40, 0, 0};
   
   Mass_Matrix3 mass3(baxter_ptr);
   mass3.calculate_mass_matrix();
   
   Robot_Dynamics baxter_dynamics(baxter_ptr);
   baxter_dynamics.calc_link_adjusted_gmasss();
   baxter_dynamics.calc_mass_matrix();
   baxter_dynamics.calc_coriolis_matrix();
   
   cout << "Mass Matrix 3: " << endl;
   cout << mass3.mass_matrix << endl;
   cout << " " << endl;
   cout << "Robot Dynamics Mass Matrix: " << endl;
   cout << baxter_dynamics.mass_matrix << endl;
   cout << " " << endl;
   cout << "Robot Dynamics Coriolis Matrtix " << endl;
   cout << baxter_dynamics.coriolis_matrix << endl;
   cout << " " << endl;
      
   return 0;
}
