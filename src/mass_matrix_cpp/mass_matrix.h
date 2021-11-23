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
      Baxter *baxter;
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd twists;

      MatrixXd spatial_jacobian;
      MatrixXd mass_matrix;
      
      vector <Matrix4d> g_joints {linalg::eye4};
      
      vector <MatrixXd> link_pJacs;
      vector <MatrixXd> link_oJacs;
      vector <Matrix3d> link_rots;      
      
      vector <MatrixXd> actuator_pJacs;
      vector <MatrixXd> actuator_oJacs;
      vector <Matrix3d> actuator_rots;   

      Mass_Matrix(Baxter*);
      
      void calculate_twists();
      void calculate_jacobian();
      
      void calculate_link_Jacs();
      void calculate_link_rots();
      
      void calculate_actuator_Jacs();
      void calculate_actuator_rots();
      
      void calculate_mass_matrix();
};

Mass_Matrix::Mass_Matrix(Baxter* manip) : baxter{manip} {
   joints = baxter->joints;
   axis_joints = baxter->axis_joints;
   q_joints = baxter->q_joints;
   spatial_jacobian.resize(6, baxter->joints);
   mass_matrix.resize(baxter->joints, baxter->joints);
   
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
      double joint_theta = baxter->thetas(joint);
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

// Calculates the position and orientation jacobians for each link
// Stores position and orientation jacobians in link_pJacs and link_oJacs
void Mass_Matrix::calculate_link_Jacs() {

   // Link n will be defined to be the link after joint n-1
   // Therefore links will range from 1-7, joints from 0-6
   for (int link{1}; link <= joints; link++) {
      // Copy link position vectors and joint position/orientation vectors
      Matrix<double, 7, 3> pls = baxter->p_links;
      Matrix<double, 7, 3> qjs = baxter->q_joints;
      Matrix<double, 7, 3> ajs = baxter->axis_joints;
   
      // Initialize jacobian for link i and resize it
      MatrixXd linki_pJac;
      MatrixXd linki_oJac;
      linki_pJac.resize(3,joints);
      linki_oJac.resize(3,joints);
   
      // matrix isnt zeros after resize method for some reason idrk
      // zero out new jacobians for link i 
      for (int i = 0; i < joints; i++) {
         linki_pJac(0, i) = 0;
         linki_pJac(1,i) = 0;
         linki_pJac(2,i) = 0;
         linki_oJac(0, i) = 0;
         linki_oJac(1,i) = 0;
         linki_oJac(2,i) = 0;
      }
   
      // Create homog vector of initial link position 
     Vector4d pl_init {pls(link-1, 0), pls(link-1, 1), pls(link-1, 2), 1};
     
      // Transforms it using g matrices calculated in 'calculate_jacobian'
      // Link should only be transformed from joints/gs up to joint (n)
      Vector4d pl_hom = g_joints.at(link) * pl_init;
   
      // Calculate transformed joint vector similar to link vector above
      // (pj-1), defined as pj_hom
      // Find difference between link and joint vectors (pli-pj-1),
      // defined as p_link_joint
      // Find transformed joint axis
      // Find cross between joint axis and pos vector (zj-1 x (pli - pj-1))
      // Set as column of jacobian
      for (int col{0}; col < link; col++) {
         Vector4d pj_init {qjs(col, 0), qjs(col, 1), qjs(col, 2), 1};
         Vector4d pj_hom {g_joints.at(col + 1) * pj_init};
         Vector3d p_link_joint {pl_hom(0) - pj_hom(0), pl_hom(1) - pj_hom(1), pl_hom(2) - pj_hom(2)};  
         
         Vector3d wj {spatial_jacobian(3, col), spatial_jacobian(4,col), spatial_jacobian(5,col)};
         
         Vector3d cross {wj.cross(p_link_joint)};
      
         linki_pJac(0, col) = cross(0);
         linki_pJac(1, col) = cross(1);
         linki_pJac(2, col) = cross(2);    
         
         linki_oJac(0,col) =  wj(0);
         linki_oJac(1,col) =  wj(1);
         linki_oJac(2,col) =  wj(2);               
      }
      
      link_pJacs.push_back(linki_pJac);
      link_oJacs.push_back(linki_oJac);
   }
}

// Calculates the position and orientation jacobians for each motor
// Stores position and orientation jacobians in actuator_pJacs and actuator_oJacs
void Mass_Matrix::calculate_actuator_Jacs() {

   for (int act{0}; act < joints; act++) {
      // Copy actuator position/orientation vectors
      Matrix<double, 7, 3> pas = baxter->p_motors;
      Matrix<double, 7, 3> qjs = baxter->q_joints;
      Matrix<double, 7, 3> ajs = baxter->axis_joints;
   
      // Initialize jacobian for link i and resize it
      MatrixXd actuatori_pJac;
      MatrixXd actuatori_oJac;
      actuatori_pJac.resize(3,joints);
      actuatori_oJac.resize(3,joints);
   
      // matrix isnt zeros after resize method for some reason idrk
      // zero out new jacobians for link i 
      for (int i = 0; i < joints; i++) {
         actuatori_pJac(0, i) = 0;
         actuatori_pJac(1,i) = 0;
         actuatori_pJac(2,i) = 0;
         actuatori_oJac(0, i) = 0;
         actuatori_oJac(1,i) = 0;
         actuatori_oJac(2,i) = 0;
      }
   
      // Create homog vector of initial actuator position 
     Vector4d pa_init {pas(act, 0), pas(act, 1), pas(act, 2), 1};
     
      // Transforms it using g matrices calculated in 'calculate_jacobian'
      // motor should only be transformed from joints/gs up to joint (n-1)
      Vector4d pa_hom = g_joints.at(act) * pa_init;
   
      // Calculate transformed joint vector similar to link vector above
      // Find difference between joint i and joint (j-1) vectors (pi-pj-1)
      // Find transformed joint axis
      // Find cross between joint axis and pos vector (zj-1 x (pli - pj-1))
      // Set as column of jacobian
      for (int col{0}; col <= act; col++) {
         Vector4d qi_init {qjs(col, 0), qjs(col, 1), qjs(col, 2), 1};
         Vector4d qi_hom {g_joints.at(col + 1) * qi_init};
         Vector3d pji {pa_hom(0) - qi_hom(0), pa_hom(1) - qi_hom(1), pa_hom(2) - qi_hom(2)};  
         
         Vector3d wj {spatial_jacobian(3, col), spatial_jacobian(4,col), spatial_jacobian(5,col)};
         
         Vector3d cross {wj.cross(pji)};
      
         actuatori_pJac(0, col) = cross(0);
         actuatori_pJac(1, col) = cross(1);
         actuatori_pJac(2, col) = cross(2);    
         
         actuatori_oJac(0,col) =  wj(0);
         actuatori_oJac(1,col) =  wj(1);
         actuatori_oJac(2,col) =  wj(2);               
      }
      
      // The last non-zero column (column i)of the orientation jac 
      // for the motor must be kri * zmi
      actuatori_oJac(0,act) =  baxter->krs.at(act) * spatial_jacobian(3, act);
      actuatori_oJac(1,act) =  baxter->krs.at(act) * spatial_jacobian(4, act);
      actuatori_oJac(2,act) =  baxter->krs.at(act) * spatial_jacobian(5, act);
      
      actuator_pJacs.push_back(actuatori_pJac);
      actuator_oJacs.push_back(actuatori_oJac);
   }
}

// Calculate the rotation matrices for each link (Ri)
// These will be inside the already calculated g matrices for each joint
void Mass_Matrix::calculate_link_rots() {
   for (int link{1}; link <= joints; link++) {
      Vector4d r1 {g_joints.at(link).row(0)};
      Vector4d r2 {g_joints.at(link).row(1)};
      Vector4d r3 {g_joints.at(link).row(2)};
      Matrix3d linki_rot {
      {r1(0), r1(1), r1(2)},
      {r2(0), r2(1), r2(2)},
      {r3(0), r3(1), r3(2)}
      };
      
      link_rots.push_back(linki_rot);
      actuator_rots.push_back(linki_rot);
   }
}

void Mass_Matrix::calculate_mass_matrix() {
   for (int i{0}; i < joints; i++) {
      for (int j{0}; j < joints; j++) {
         mass_matrix(i, j) = 0;
      }
   }
   
   // Using the joint positions/orientation vectors (q_joints/axis_joints),
   // calculate the twists for the manipulator
   calculate_twists();
   
   // Calculate the spatial jacobian, which also calculates g_joints,
   // which is all the g matrices for all the joints
   calculate_jacobian();
   
   // Calculate the position and orientation jacobians for the links/motors
   calculate_link_Jacs();
   calculate_actuator_Jacs();
   
   // Calculate rotation matrices for the links/motors
   calculate_link_rots();
  
   // Calculate mass matrix
   for (int i{0}; i < joints; i++) {
      // Add the link mass component
      mass_matrix += baxter->mls.at(i) * link_pJacs.at(i).transpose() * link_pJacs.at(i);
      // Add the link inertia component
      mass_matrix += link_oJacs.at(i).transpose() * link_rots.at(i) * baxter->Ils.at(i) * link_rots.at(i).transpose() * link_oJacs.at(i);
      
      // Add the motor mass component
      mass_matrix += baxter->mms.at(i) * actuator_pJacs.at(i).transpose() * actuator_pJacs.at(i);
      // Add the motor inertia component
      mass_matrix += actuator_oJacs.at(i).transpose() * actuator_rots.at(i) * baxter->Ims.at(i) * actuator_rots.at(i).transpose() * actuator_oJacs.at(i);
   }
   
}

   

#endif
