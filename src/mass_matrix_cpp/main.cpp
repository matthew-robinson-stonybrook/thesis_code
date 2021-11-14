#include<iostream>
#include "spatial_jacobian.h"
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
   Matrix<double, 7, 1> thetas {pi/4, pi/4, 2, 1, 2, 1, 2};
   baxter.thetas = thetas;
   Spatial_Jacobian jac(baxter.axis_joints, baxter.q_joints, baxter.thetas, baxter.joints);
   
   jac.calculate_twists();
   
   cout << "Twists = \n" << jac.twists << endl;
   jac.calculate_jacobian();
   MatrixXd spatial_jac = jac.spatial_jacobian;
   
   cout << "Jacobian: " << endl;
   cout << spatial_jac << endl;
   
   Matrix4d g_test = g::axis(baxter.axis_joints.row(2), baxter.q_joints.row(2), thetas(1));
   cout << g_test << endl;
   
   return 0;
}
