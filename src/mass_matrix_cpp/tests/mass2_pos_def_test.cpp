#include<iostream>
#include "../mass_matrix2.h"
#include "../Baxter.h"
#include "../linalg.h"
#include "../transformation.h"
#include "pos_def_test.h"

#include "../../../eigen-3.4.0/Eigen/Dense"

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
   Mass_Matrix2 mass(baxter_ptr);
   
   mass.calculate_mass_matrix();
   
   bool is_sym {true};
   for (double t1 {-pi/2}; t1 <= pi/2; t1 += pi/4) {
      cout << "t1: " << t1 << endl;
      for (double t2 {-pi/2}; t2 <= pi/2; t2 += pi/4) {
         cout << "t2: " << t2 << endl;
         for (double t3 {-pi/2}; t3 <= pi/2; t3 += pi/4) {
            for (double t4 {-pi/2}; t4 <= pi/2; t4 += pi/4) {
               for (double t5 {-pi/2}; t5 <= pi/2; t5 += pi/4) {
                  for (double t6 {-pi/2}; t6 <= pi/2; t6 += pi/4) {
                     for (double t7 {-pi/2}; t7 <= pi/2; t7 += pi/4) { 
                        baxter.thetas = {t1, t2, t3, t4, t5, t6, t7};
                        mass.calculate_mass_matrix();
                        if (!mass_tester::symmetric(mass.mass_matrix) || !mass_tester::pos_def(mass.mass_matrix)) {
                           is_sym = false;
                           break;
                        }
                     }
                  }
               }
            }
         }
      }
   }
   
   cout << "Is the mass matrix always symmetric? : " << is_sym << endl;
   
  
   return 0;
}
