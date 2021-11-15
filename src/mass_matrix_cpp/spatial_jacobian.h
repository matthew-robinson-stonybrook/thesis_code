#ifndef _MASS_MATRIX_H_
#define _MASS_MATRIX_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "Baxter.h"

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

//spatial jacobian
class Mass_Matrix {
   public:
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd thetas;
      MatrixXd twists;

      MatrixXd spatial_jacobian;
      MatrixXd mass_matrix;
      
      vector <Matrix4d> g_joints {linalg::eye4};
      
      vector <MatrixXd> link_pJacs;
      vector <MatrixXd> link_oJacs;
      vector <MatrixXd> a
      

      Mass_Matrix(MatrixXd, MatrixXd, MatrixXd, int);
      Mass_Matrix(Baxter);
      
      void calculate_twists();
      void calculate_jacobian();
      void calculate_link_pJacs(Baxter);
};

Mass_Matrix::Mass_Matrix(MatrixXd ajs, MatrixXd qjs, MatrixXd ts, int js)
   :axis_joints{ajs}, q_joints{qjs}, thetas{ts}, joints{js} {
   spatial_jacobian.resize(6,joints);
   mass_matrix.resize(joints,joints);
}

Mass_Matrix::Mass_Matrix(Baxter baxter)
   : Mass_Matrix {baxter.axis_joints, baxter.q_joints, baxter.thetas, baxter.joints} {
 
}

void Mass_Matrix::calculate_twists(){
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

void Mass_Matrix::calculate_jacobian() {
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

// Link n will be defined to be the link after joint n-1
// Therefore links will range from 1-7, joints from 0-6
void Mass_Matrix::calculate_link_pJacs(Baxter baxter) {
   for (int link{1}; link <= joints; link++) {
      // Copy link position vectors and joint position/orientation vectors
      Matrix<double, 7, 3> pls = baxter.p_links;
      Matrix<double, 7, 3> qjs = baxter.q_joints;
      Matrix<double, 7, 3> ajs = baxter.axis_joints;
   
      // Initialize jacobian for link i and resize it
      MatrixXd linki_pJac;
      linki_pJac.resize(3,joints);
   
      // matrix isnt zeros after resize method for some reason idrk
      // zero out new pos jacobian for link i 
      for (int i = 0; i < joints; i++) {
         linki_pJac(0, i) = 0;
         linki_pJac(1,i) = 0;
         linki_pJac(2,i) = 0;
      }
   
      // Create homog vector of initial link position and transform it using transformation matrices calculated in 'calculate_jacobian' method
      //link should only be transformed based on joints/transformation matrices up to most previous joint (n-1)
     Vector4d p_link_init {pls(link-1, 0), pls(link-1, 1), pls(link-1, 2), 1};
      Vector4d pl_hom = g_joints.at(link) * p_link_init;
   
      // Calculate transformed joint vector, and find the difference between link and joint vectors (pli-pj-1)
      // Find cross prod from joint axis (zj-1 x (pli - pj-1)) and set as column of jacobian
      for (int col{0}; col < link; col++) {
         Vector4d pj_init {qjs(col, 0), qjs(col, 1), qjs(col, 2), 1};
         Vector4d pj_hom {g_joints.at(col + 1) * pj_init};
         Vector3d p_link_joint {pl_hom(0) - pj_hom(0), pl_hom(1) - pj_hom(1), pl_hom(2) - pj_hom(2)};  
      
         Vector3d cross {ajs.row(col).cross(p_link_joint)};
      
         linki_pJac(0, col) = cross(0);
         linki_pJac(1, col) = cross(1);
         linki_pJac(2, col) = cross(2);           
      }
      
      link_pJacs.push_back(linki_pJac);
   }
}
   

#endif
