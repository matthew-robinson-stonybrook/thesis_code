#include<iostream>
#include "mass_matrix.h"
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
   Baxter baxter;
   baxter.thetas = {pi/4, 0, 0, pi/2, 0, 0, 2};
   Mass_Matrix mass(baxter);
   
   mass.calculate_mass_matrix();
   
   cout << "MASS MATRIX: " << endl;
   cout << mass.mass_matrix << endl;
   
   return 0;
}
