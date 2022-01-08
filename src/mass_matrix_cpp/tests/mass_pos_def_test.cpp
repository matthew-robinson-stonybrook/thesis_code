#include<iostream>
#include "../dynamics.h"
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
   Robot_Dynamics bd(baxter_ptr);
   
   bd.calc_link_adjusted_gmasss();
   bd.calc_mass_matrix();
   
   bool is_pos_def {true};
   for (double t1 {-pi/2}; t1 <= pi/2; t1 += pi/4) {
      cout << "t1: " << t1 << endl;
      for (double t2 {-pi/2}; t2 <= pi/2; t2 += pi/4) {
         cout << "t2: " << t2 << endl;
         for (double t3 {-pi/2}; t3 <= pi/2; t3 += pi/4) {
            cout << "t3: " << t3 << endl;
            for (double t4 {-pi/2}; t4 <= pi/2; t4 += pi/4) {
               cout << "t4: " << t4 << endl;
               for (double t5 {-pi/2}; t5 <= pi/2; t5 += pi/4) {
                  for (double t6 {-pi/2}; t6 <= pi/2; t6 += pi/4) {
                     for (double t7 {-pi/2}; t7 <= pi/2; t7 += pi/4) { 
                        baxter.thetas = {t1, t2, t3, t4, t5, t6, t7};
                        bd.calc_link_adjusted_gmasss();
                        bd.calc_mass_matrix();
                        if (!mass_tester::symmetric(bd.mass_matrix) || !mass_tester::pos_def(bd.mass_matrix)) {
                           is_pos_def = false;
                           break;
                        }
                     }
                  }
               }
            }
         }
      }
   }
   
   cout << "Is the mass matrix always symmetric and positive definite? : " << endl;
   cout << is_pos_def << endl;
   
  
   return 0;
}
