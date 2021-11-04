#include<iostream>
#include<vector>

#include </home/matt_robinson/cpp_libraries/eigen-3.4.0/Eigen/Dense>

using namespace std
using namespace Eigen

using namespace std

//spatial jacobian
class Spatial_Jacobian {
   private:
      Eigen::MatrixXd axis_joints(2,2);
      axis_joints << 2,4,5,6;
   
   public:
      Eigen::MatrixXd return_axis_joints();
};

Eigen::MatrixXd return_axis_joints();
