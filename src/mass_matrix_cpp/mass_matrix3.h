#ifndef _MASS_MATRIX3_H_
#define _MASS_MATRIX3_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "Baxter.h"

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Mass_Matrix3 {
   public:
      Baxter *baxter;
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd twist_coords;

      MatrixXd mass_matrix;
      
      vector <Matrix4d> g_joints {linalg::eye4};
      
      vector <MatrixXd> link_pJacs;
      vector <MatrixXd> link_oJacs;
      vector <Matrix3d> link_rots;      
      
      vector <MatrixXd> actuator_pJacs;
      vector <MatrixXd> actuator_oJacs;
      vector <Matrix3d> actuator_rots;   

      Mass_Matrix3(Baxter*);
      
      void calculate_twist_coords();
      Matrix<double, 6, 6> calculate_generalized_mass(int mass, Matrix3d inertia);
      Matrix<double, 6, 6> calculate_adjusted_gmass(Matrix<double, 6, 6> gmass, int link);
      Matrix<double, 6, 6> calculate_adjoint_ij(int i, int j);
      
      //MatrixXd calculate_actuator_Jac();
      void calculate_mass_matrix();

};

Mass_Matrix3::Mass_Matrix3(Baxter* manip) : baxter{manip} {
   joints = baxter->joints;
   axis_joints = baxter->axis_joints;
   q_joints = baxter->q_joints;
   mass_matrix.resize(baxter->joints, baxter->joints);
   
}

void Mass_Matrix3::calculate_twist_coords(){
   twist_coords.resize(joints, 6);
   Vector3d cross;
   Vector3d uj;
   Vector3d qj;
   int t;
   
   for (int joint = 0; joint < joints; joint++) {
      uj = axis_joints.row(joint);
      qj = q_joints.row(joint);
      cross = (-1 * uj).cross(qj);
      
      twist_coords(joint, 0) = cross(0);
      twist_coords(joint, 1) = cross(1);
      twist_coords(joint, 2) = cross(2);
      twist_coords(joint, 3) = uj(0);
      twist_coords(joint, 4) = uj(1);
      twist_coords(joint, 5) = uj(2);

   }
}

Matrix<double, 6, 6> Mass_Matrix3::calculate_generalized_mass(int mass, Matrix3d inertia) {
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

Matrix<double, 6, 6> Mass_Matrix3::calculate_adjusted_gmass(Matrix<double, 6, 6> gmass, int link) {
   Matrix4d gsli0 = linalg::eye4;
   for(int joint{0}; joint < link; joint++) {
      double theta = baxter->thetas(joint);
      Matrix4d gj = g::twist(twist_coords.row(joint), theta);
      gsli0 = gsli0 * gj;
   }
   MatrixXd p_link = baxter->p_links.row(link-1);
   Matrix4d gsli = linalg::eye4;
   gsli(0, 3) = p_link(0);
   gsli(1, 3) = p_link(1);
   gsli(2, 3) = p_link(2);
   gsli0 = gsli0 * gsli;
   
   Matrix<double,6,6> adj_gsli0_inv = g::adjoint(gsli0.inverse());
   
   Matrix<double,6,6> adjusted_gmass = adj_gsli0_inv.transpose() * gmass * adj_gsli0_inv;
   
   return adjusted_gmass;
}

Matrix<double, 6, 6> Mass_Matrix3::calculate_adjoint_ij(int i, int j) {
   Matrix<double, 6, 6> adjoint;
   Matrix4d g {linalg::eye4};
   
   if (i > j) {
      for(int k{j+1}; k <= i; k++) {
         g = g * g::twist(twist_coords.row(k), baxter->thetas(k));
      }
      
      adjoint = g::adjoint(g.inverse());
   }
   else if (i == j) {
      adjoint = linalg::eye6;
   }
   
   else if (i < j) {
      adjoint = linalg::eye6;
      for(int k{0}; k < 6; k++) {
         adjoint(k,k) = 0;
      } 
   }
   
   return adjoint;
}

void Mass_Matrix3::calculate_mass_matrix() {
   cout << "M3 " << endl;
   
   calculate_twist_coords();
   
   const int js {baxter->joints};
   
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         mass_matrix(row,col) = 0;
      }
   }
   
   int m;
   Matrix3d I;
   MatrixXd p_link;
   
   Matrix<double,6,6> gmass;
   Matrix<double,6,6> adj_gsli0;
   Matrix4d gsli0;
   Matrix<double,6,6> adjusted_gmass;
   
   for(int row {0}; row < js; row++) {
      for(int col {0}; col < js; col++) {
         double  val = 0;
         
         for(int l{max(row,col)}; l < js; l++) {
            m = baxter->mls.at(l);
            I = baxter->Ils.at(l);
            gmass = calculate_generalized_mass(m, I);
            
            adjusted_gmass = calculate_adjusted_gmass(gmass, l + 1);
            
            Matrix<double, 6, 1> twist_i = twist_coords.row(row);
            Matrix<double, 6, 6> adj_li = calculate_adjoint_ij(l, row);
            Matrix<double, 6, 1> twist_j = twist_coords.row(col);
            Matrix<double, 6, 6> adj_lj = calculate_adjoint_ij(l, col);
            
            val += twist_i.transpose() * adj_li.transpose() * adjusted_gmass * adj_lj * twist_j;
         }
         mass_matrix(row,col) = val;
      }
   }
}

#endif
