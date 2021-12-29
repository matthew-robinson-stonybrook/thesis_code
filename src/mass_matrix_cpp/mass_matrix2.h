#ifndef _MASS_MATRIX2_H_
#define _MASS_MATRIX2_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "Baxter.h"

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Mass_Matrix2 {
   public:
      Baxter *baxter;
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd screws;

      MatrixXd spatial_jacobian;
      MatrixXd mass_matrix;
      
      vector <Matrix4d> g_joints {linalg::eye4};
      
      vector <MatrixXd> link_pJacs;
      vector <MatrixXd> link_oJacs;
      vector <Matrix3d> link_rots;      
      
      vector <MatrixXd> actuator_pJacs;
      vector <MatrixXd> actuator_oJacs;
      vector <Matrix3d> actuator_rots;   

      Mass_Matrix2(Baxter*);
      
      void calculate_screws();
      Matrix<double, 6, 6> calculate_generalized_mass(int mass, Matrix3d inertia);
      
      MatrixXd calculate_link_Jac(int);
      //MatrixXd calculate_actuator_Jac();
      //void calculate_mass_matrix();
      
      /*
      void calculate_jacobian();
      void calculate_link_rots();
      void calculate_actuator_rots();
      */
};

Mass_Matrix2::Mass_Matrix2(Baxter* manip) : baxter{manip} {
   joints = baxter->joints;
   axis_joints = baxter->axis_joints;
   q_joints = baxter->q_joints;
   spatial_jacobian.resize(6, baxter->joints);
   mass_matrix.resize(baxter->joints, baxter->joints);
   
}

void Mass_Matrix2::calculate_screws(){
   screws.resize(joints, 6);
   Vector3d cross;
   Vector3d uj;
   Vector3d qj;
   int t;
   
   for (int joint = 0; joint < joints; joint++) {
      uj = axis_joints.row(joint);
      qj = q_joints.row(joint);
      cross = (-1 * uj).cross(qj);
      
      screws(joint, 0) = cross(0);
      screws(joint, 1) = cross(1);
      screws(joint, 2) = cross(2);
      screws(joint, 3) = uj(0);
      screws(joint, 4) = uj(1);
      screws(joint, 5) = uj(2);

   }
}

Matrix<double, 6, 6> Mass_Matrix2::calculate_generalized_mass(int mass, Matrix3d inertia) {
   Matrix<double, 6, 6> gen_mass;
   
   for(int row{0}; row < 6; row++) {
      for(int col{0}; col < 6; col++) {
         if(row >= 3 && col >= 3) {
            gen_mass(row,col) = inertia(row - 3, col - 3);
         }
         else {
            gen_mass(row,col) = 0;
         }
      }
   }
   
   gen_mass(0,0) = mass;
   gen_mass(1,1) = mass;
   gen_mass(2,2) = mass;
   
   
   return gen_mass;
   
}

MatrixXd Mass_Matrix2::calculate_link_Jac(int link) {
   MatrixXd jac;
   jac.resize(6,joints);
   
   // Zero out matrix
   for (int i{0}; i < joints; i++) {
      for (int j{0}; j < 6; j++) {
         jac(i, j) = 0;
      }
   }
   MatrixXd p_link;
   Matrix4d gsli;
   Matrix4d g_hatj {linalg::eye4};
   
   for(int col{0}; col < link; col++) {     
      p_link = baxter->p_links.row(link);
      gsli = linalg::eye4;
      gsli(0, 3) = p_link(0);
      gsli(1, 3) = p_link(1);
      gsli(2, 3) = p_link(2);
      
      for(int t{col}; t < link; t++) {
         Matrix<double, 6, 1> screw = screws.row(t);
         Matrix4d gi = g::twist(screw, baxter->thetas.at(t));
         
      }
   }
   
   return gsli;
}

#endif
