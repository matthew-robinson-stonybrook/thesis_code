#include<iostream>
#include "mass_matrix.h"
#include "Baxter.h"
#include "linalg.h"
#include "transformation.h"
#include "pos_def_test.h"

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
   Mass_Matrix mass(baxter_ptr);
   
   mass.calculate_mass_matrix();

  
   return 0;
}
