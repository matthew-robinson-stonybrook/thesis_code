#include<iostream>
#include "spatial_jacobian.h"
#include "Baxter.h"
#include "linalg.h"
#include "transformation.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265;


int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   
   
   Baxter baxter;
   Matrix<double, 7, 1> thetas {pi/4, 1, 2, 1, 2, 1, 2};
   Spatial_Jacobian jac(baxter.axis_joints, baxter.q_joints, thetas);
   
   jac.calculate_twists();
   
   cout << "Twists = \n" << jac.twists << endl;
   
   MatrixXd spatial_jac = jac.calculate_jacobian();
   
   cout << "Jacobian: " << endl;
   cout << spatial_jac << endl;
   
   return 0;
}
