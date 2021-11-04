#include<iostream>
#include "spatial_jacobian.h"

#include </home/matt_robinson/cpp_libraries/eigen-3.4.0/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
   std::cout << "Hello World From Thesis_Code" << std::endl;
   
   Spatial_Jacobian test_jacobian;
   MatrixXf new_axis_joints = test_jacobian.return_axis_joints();
   cout << "New Axis Joints: " << new_axis_joints << endl;
   new_axis_joints(0) = 3;
   new_axis_joints(1) = 2;
   new_axis_joints(2) = 7;
   cout << "New Axis Joints: " << new_axis_joints << endl;
   
   return 0;
}
