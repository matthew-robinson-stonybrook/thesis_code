#include<iostream>
#include<time.h>

#include "Scarra.h"
#include "mass_matrix2.h"
#include "dynamics.h"
#include "linalg.h"
#include "transformation.h"
#include "mass_matrix2.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;
   
int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   Scarra *scarra_ptr = new Scarra();
   
   Robot_Dynamics scarra_dynamics(scarra_ptr);
   scarra_ptr->thetas = {pi/2, pi/4, pi/8};
   
   scarra_dynamics.calc_link_adjusted_gmasss();
   scarra_dynamics.calc_mass_matrix();
   
   cout << "Reference Scarra Mass Matrix: " << endl;
   cout << scarra_ptr->reference_mass_matrix() << endl;
   cout << "Calculated Scarra Twist Coordinates: " << endl;
   cout << scarra_dynamics.mass_matrix << endl;
   
   return 0;      
}
