#include<iostream>
#include "mass_matrix.h"
#include "mass_matrix2.h"
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
   
   //baxter_ptr->thetas = {pi/4, 0, pi/8, 0, pi/4, 0, 0};
   baxter_ptr->thetas = {pi/4, 0, 0, 0, 0,  0, 0};
    
   Mass_Matrix mass(baxter_ptr);
   mass.calculate_mass_matrix();
   
   Mass_Matrix2 mass2(baxter_ptr);
   //mass2.calculate_mass_matrix();
   mass2.calculate_screws();

   
   cout << "Mass1: " << endl;
   cout << mass.link_oJacs.at(2) << endl;
   cout << "Mass2 : " << endl;
   cout << mass2.calculate_link_Jac(3) << endl;  
    
   Matrix<double,6,1> twist1_test {1, 3, 5, 7, 9, 11};
   Matrix<double,6,1> twist2_test {2, 4, 6, 8, 10, 12};
   Matrix<double,6,1> twist_product {g::lie_bracket(twist1_test, twist2_test)};
   
   cout << "Original Twist1: " << endl;
   cout << twist1_test << endl;
   cout << "Original Twist2: " << endl;
   cout << twist2_test << endl;
   cout << "Product " << endl;
   cout << twist_product << endl; 
   
   MatrixXd m2_test = mass2.calculate_mass_matrix2();
   cout << "Mass Matrix calc 2: " << endl;
   cout << m2_test << endl;
   
   
   return 0;
}
