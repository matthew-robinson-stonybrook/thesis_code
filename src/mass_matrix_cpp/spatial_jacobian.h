#ifndef _SPATIAL_JAC_H_
#define _SPATIAL_JAC_H_

#include<iostream>
#include<vector>
#include "transformation.h"

#include "linalg.h"
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
      
      MatrixXd spatial_jacobian;
      

      Spatial_Jacobian(MatrixXd, MatrixXd, MatrixXd);
      void calculate_twists();
      MatrixXd calculate_jacobian();
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

MatrixXd Spatial_Jacobian::calculate_jacobian() {
   int joints = axis_joints.rows();
   spatial_jacobian.resize(6, joints);
   Matrix4d g1_is [joints + 1];
   *(g1_is) = linalg::eye4;
   for (int joint = {0}; joint < joints; joint++) {
      Matrix<double, 6, 1> joint_twist = twists.row(joint);
      double joint_theta = thetas(joint);
      *(g1_is + joint + 1) = *(g1_is + joint) * g::twist(joint_twist, joint_theta);
      Matrix<double, 6, 1> twist_prime = g::adjoint(*(g1_is + joint)) * joint_twist;
      
      spatial_jacobian(0, joint) = twist_prime(0);
      spatial_jacobian(1, joint) = twist_prime(1);
      spatial_jacobian(2, joint) = twist_prime(2);
      spatial_jacobian(3, joint) = twist_prime(3);
      spatial_jacobian(4, joint) = twist_prime(4);
      spatial_jacobian(5, joint) = twist_prime(5);
   }
   return spatial_jacobian;
}
   

#endif
