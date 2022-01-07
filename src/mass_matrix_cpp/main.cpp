#include<iostream>
#include "mass_matrix.h"
#include "mass_matrix2.h"
#include "mass_matrix3.h"
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
   
   Baxter *baxter_ptr2 {nullptr};
   Baxter baxter2;
   baxter_ptr2 = &baxter2;
   baxter_ptr2->thetas = {pi/4, 0, pi/8, 0, pi/4, 0, 0};
   
   Mass_Matrix2 mass2(baxter_ptr);
   mass2.calculate_mass_matrix();
   Mass_Matrix3 mass3(baxter_ptr);
   mass3.calculate_mass_matrix();
   
   MatrixXd mass_test = mass2.calculate_mass_matrix2();
   
   
   cout << " " << endl;
   cout << "Mass Matrix 2: " << endl;
   cout << mass2.mass_matrix << endl;
   cout << " " << endl;
   cout << "Mass Matrix 3: " << endl;
   cout << mass3.mass_matrix << endl;
   cout << " " << endl;
   
  
   return 0;
}
