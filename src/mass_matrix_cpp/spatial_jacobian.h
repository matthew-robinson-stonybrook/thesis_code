#ifndef _SPATIAL_JAC_H_
#define _SPATIAL_JAC_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "Baxter.h"

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

//spatial jacobian
class Spatial_Jacobian {
   public:
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd thetas;
      MatrixXd twists;
      
      MatrixXd spatial_jacobian;
      
      vector <Matrix4d> g_joints {linalg::eye4};
      

      Spatial_Jacobian(MatrixXd, MatrixXd, MatrixXd, int);
      Spatial_Jacobian(Baxter);
      
      void calculate_twists();
      void calculate_jacobian();
};

Spatial_Jacobian::Spatial_Jacobian(MatrixXd ajs, MatrixXd qjs, MatrixXd ts, int js){
   axis_joints = ajs;
   q_joints = qjs;
   thetas = ts;
   joints = js;
}

Spatial_Jacobian::Spatial_Jacobian(Baxter baxter){
   axis_joints = baxter.axis_joints;
   q_joints = baxter.q_joints;
   thetas = baxter.thetas;
   joints = baxter.joints;
}

void Spatial_Jacobian::calculate_twists(){
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

void Spatial_Jacobian::calculate_jacobian() {
   // Calculate twists...
   calculate_twists();

   // Calculate total number of joints
   
   //Resize spatial jacobian to have correct columns for # of joints
   spatial_jacobian.resize(6, joints);

   // Create transofmration matrices for adjoint to transform twists
   for (int joint = {0}; joint < joints; joint++) {
      Matrix<double, 6, 1> joint_twist = twists.row(joint);
      double joint_theta = thetas(joint);
      g_joints.push_back(g_joints.at(joint) * g::twist(joint_twist, joint_theta));
      Matrix<double, 6, 1> twist_prime = g::adjoint(g_joints.at(joint)) * joint_twist;

      spatial_jacobian(0, joint) = twist_prime(0);
      spatial_jacobian(1, joint) = twist_prime(1);
      spatial_jacobian(2, joint) = twist_prime(2);
      spatial_jacobian(3, joint) = twist_prime(3);
      spatial_jacobian(4, joint) = twist_prime(4);
      spatial_jacobian(5, joint) = twist_prime(5);

   }

}
   

#endif
