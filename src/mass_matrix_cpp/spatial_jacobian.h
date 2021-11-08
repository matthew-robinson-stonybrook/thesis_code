#ifndef _SPATIAL_JAC_H_
#define _SPATIAL_JAC_H_

#include<iostream>
#include<vector>

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

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
   
   Vector3d cross;
   Vector3d uj;
   Vector3d qj;
   int t;
   
   for (int joint = 0; joint < joints; joint++) {
      uj = axis_joints.row(joint);
      qj = q_joints.row(joint);
      cross = (-1 * uj).cross(qj);
      
      twists(joint, 0) = cross(0);
      twists(joint, 1) = cross(1);
      twists(joint, 2) = cross(2);
      twists(joint, 3) = uj(0);
      twists(joint, 4) = uj(1);
      twists(joint, 5) = uj(2);

   }
}
   

#endif
