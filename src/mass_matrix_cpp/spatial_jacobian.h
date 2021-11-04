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
      MatrixXd twists;

      Spatial_Jacobian(MatrixXd, MatrixXd, MatrixXd);
      void calculate_twists();
};

Spatial_Jacobian::Spatial_Jacobian(MatrixXd ajs, MatrixXd qjs, MatrixXd ts){
   axis_joints = ajs;
   q_joints = qjs;
   thetas = ts;
}

void Spatial_Jacobian::calculate_twists(){
   int joints = axis_joints.rows();
   twists.resize(joints, 6);
   //for (int joint; joint < joints; joint++) {
      
   //}
}
   

#endif
