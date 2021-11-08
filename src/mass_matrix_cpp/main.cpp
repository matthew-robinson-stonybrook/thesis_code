#include<iostream>
#include "spatial_jacobian.h"
#include "Baxter.h"
#include "linalg.h"
#include "transformation.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14169265;


int main() {	
   cout << "Hello World From Thesis_Code" << endl;
   
   Baxter baxter;
   Spatial_Jacobian jac(baxter.axis_joints, baxter.q_joints, baxter.thetas);
   
   cout << "axis_joints =  \n" << jac.axis_joints << endl;
   cout << "q_joints =  \n" << jac.q_joints << endl;
   cout << "thetas =  \n" << jac.thetas << endl;
   
   jac.calculate_twists();
   
   cout << "Twists = \n" << jac.twists << endl;

   Matrix4d t1 = g::axis(jac.axis_joints.row(1), jac.q_joints.row(1), pi/2);
   cout << "T1: " << t1 << endl;
   
   Matrix4d t2 = g::twist(jac.twists.row(1), pi/2);
   cout << "T2: " << t2 << endl;
   
   return 0;
}
