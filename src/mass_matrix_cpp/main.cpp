#include<iostream>
#include "spatial_jacobian.h"
#include "Baxter.h"

//#include </home/matt_robinson/cpp_libraries/eigen-3.4.0/Eigen/Dense>
#include "../../eigen-3.4.0/Eigen/Dense"
using namespace std;
using namespace Eigen;

double pi = 3.14169265;

int main() {
   std::cout << "Hello World From Thesis_Code" << std::endl;
   
   Baxter baxter;
   Spatial_Jacobian test_jacobian(baxter.axis_joints, baxter.q_joints, baxter.thetas);
   
   cout << "axis_joints =  \n" << test_jacobian.axis_joints << endl;
   cout << "q_joints =  \n" << test_jacobian.q_joints << endl;
   cout << "thetas =  \n" << test_jacobian.thetas << endl;

   return 0;
}
