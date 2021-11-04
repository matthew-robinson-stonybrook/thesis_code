#ifndef _SPATIAL_JAC_H_
#define _SPATIAL_JAC_H_

#include<iostream>
#include<vector>

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

//spatial jacobian
class Spatial_Jacobian {
   public:
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd thetas;

      Spatial_Jacobian(MatrixXd, MatrixXd, MatrixXd);
};

Spatial_Jacobian::Spatial_Jacobian(MatrixXd ajs, MatrixXd qjs, MatrixXd ts){
   axis_joints = ajs;
   q_joints = qjs;
   thetas = ts;
}
   

#endif
