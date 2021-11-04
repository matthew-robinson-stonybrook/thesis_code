#ifndef _SPATIAL_JAC_H_
#define _SPATIAL_JAC_H_

#include<iostream>
#include<vector>

#include </home/matt_robinson/cpp_libraries/eigen-3.4.0/Eigen/Dense>

using namespace std;
using namespace Eigen;

//spatial jacobian
class Spatial_Jacobian {
   private:
      MatrixXf axis_joints(6,7);
      
   
   public:
      MatrixXf return_axis_joints();
};

MatrixXf Spatial_Jacobian::return_axis_joints(){
   return axis_joints;
}

#endif
