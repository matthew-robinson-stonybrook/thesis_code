#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
#include "Baxter.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Robot_Dynamics {
   public:
      Baxter *baxter_ptr;
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd twist_coords;
      
      vector <Matrix<double,6,6>> link_gmasss;
      vector <Matrix<double,6,6>> link_adjusted_gmasss;
      
      MatrixXd mass_matrix;
      MatrixXd coriolis_matrix;
      double V;

      Robot_Dynamics(Baxter*);
      
      void calc_twist_coords();
      void calc_link_gmasss();
      
      void calc_link_adjusted_gmasss();
      
      Matrix<double, 6, 6> calc_gmass(int mass, Matrix3d inertia);
      Matrix<double, 6, 6> calc_adjusted_gmass(Matrix<double, 6, 6> gmass, int link);
      Matrix<double, 6, 6> calc_adjoint_ij(int i, int j);
      double calc_partialM_partialT(const int i, const int j, const int k);
      double calc_christoffel(const int i, const int j, const int k);
      
      void calc_mass_matrix();
      void calc_coriolis_matrix();
      void calc_potential_energy();
      
};

// Contructor
Robot_Dynamics::Robot_Dynamics(Baxter* manip) : baxter_ptr{manip} {
   joints = baxter_ptr->joints;
   axis_joints = baxter_ptr->axis_joints;
   q_joints = baxter_ptr->q_joints;
   mass_matrix.resize(baxter_ptr->joints, baxter_ptr->joints);
   coriolis_matrix.resize(baxter_ptr->joints, baxter_ptr->joints);
   
   calc_twist_coords();
   calc_link_gmasss();
}

// HAPPENS ONLY IN CONSTRUCTOR
//Uses manipulator joint positions and orientation to calculate twist coordinates
void Robot_Dynamics::calc_twist_coords(){
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

// HAPPENS ONLY IN CONTRUCTOR
// Creates vector of general mass matrices for the links (Mi)
void Robot_Dynamics::calc_link_gmasss() {
   link_gmasss.clear();
   
   double m;
   Matrix3d I;

   for(int j{0}; j < joints; j++) {
      m = baxter_ptr->mls.at(j);
      I = baxter_ptr->Ils.at(j);
      link_gmasss.push_back(calc_gmass(m, I));
   }
}

// Should be run before and dynamic matrices
// Calculates adjusted mass matrices due to configuration 
void Robot_Dynamics::calc_link_adjusted_gmasss() {
   link_adjusted_gmasss.clear();
   for(int j{0}; j < joints; j++) {
      Matrix<double,6,6> gmass = link_gmasss.at(j);
      link_adjusted_gmasss.push_back(calc_adjusted_gmass(gmass, j));
   }
}

// Should only be run in calc_gmasss() in contructor
Matrix<double, 6, 6> Robot_Dynamics::calc_gmass(int mass, Matrix3d inertia) {
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

// Is run in calc_adjusted_gmasss() which gets all link adjusted general masses (Mi_hat)
Matrix<double, 6, 6> Robot_Dynamics::calc_adjusted_gmass(Matrix<double, 6, 6> gmass, int link) {
   Matrix4d gsli0 = linalg::eye4;
   // Calculate gljli for adjoint (product of twist exponentials)
   for(int joint{0}; joint < link; joint++) {
      double theta = baxter_ptr->thetas(joint);
      Matrix4d gj = g::twist(twist_coords.row(joint), theta);
      gsli0 = gsli0 * gj;
   }
   MatrixXd p_link = baxter_ptr->p_links.row(link);
   Matrix4d gsli = linalg::eye4;
   gsli(0, 3) = p_link(0);
   gsli(1, 3) = p_link(1);
   gsli(2, 3) = p_link(2);
   gsli0 = gsli0 * gsli;
   
   Matrix<double,6,6> adj_gsli0_inv = g::adjoint(gsli0.inverse());
   
   Matrix<double,6,6> adjusted_gmass = adj_gsli0_inv.transpose() * gmass * adj_gsli0_inv;
   
   return adjusted_gmass;
}

// Adjoint_ij for mass and coriolis matrix computations
Matrix<double, 6, 6> Robot_Dynamics::calc_adjoint_ij(int i, int j) {
   Matrix<double, 6, 6> adjoint;
   Matrix4d g {linalg::eye4};
   
   if (i > j) {
      for(int k{j+1}; k <= i; k++) {
         g = g * g::twist(twist_coords.row(k), baxter_ptr->thetas(k));
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

// Make sure to run calc_link_adjusted_gmasss() before this
void Robot_Dynamics::calc_mass_matrix() {
   
   const int js {baxter_ptr->joints};
   
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         mass_matrix(row,col) = 0;
      }
   }
   
   double m;
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
            gmass = link_gmasss.at(l);
            adjusted_gmass = link_adjusted_gmasss.at(l);
            
            Matrix<double, 6, 1> twist_i = twist_coords.row(row);
            Matrix<double, 6, 6> adj_li = calc_adjoint_ij(l, row);
            Matrix<double, 6, 1> twist_j = twist_coords.row(col);
            Matrix<double, 6, 6> adj_lj = calc_adjoint_ij(l, col);
            
            val += twist_i.transpose() * adj_li.transpose() * adjusted_gmass * adj_lj * twist_j;
         }
         mass_matrix(row,col) = val;
      }
   }
}

// Should (probably) only be run in calc_coriolis_matrix()
double Robot_Dynamics::calc_partialM_partialT(const int i, const int j, const int k) {
   double partialM {0};
   const int js {baxter_ptr->joints};
   
   // Define three base twist coordinates
   Matrix<double,6,1> twisti = twist_coords.row(i);
   Matrix<double,6,1> twistj = twist_coords.row(j);
   Matrix<double,6,1> twistk = twist_coords.row(k);
   
   for(int l{max(i,j)}; l < js; l++) {
      // Adjusted mass matrix l
      Matrix<double, 6, 6> adjusted_gmass = link_adjusted_gmasss.at(l);
      
      // Adjoints li, lj, lk
      Matrix<double, 6, 6> adj_li = calc_adjoint_ij(l, i);
      Matrix<double, 6, 6> adj_lj = calc_adjoint_ij(l, j);
      Matrix<double, 6, 6> adj_lk = calc_adjoint_ij(l, k);

      // Adjoint(k-1)i
      Matrix<double, 6, 6> adj_k1_i = calc_adjoint_ij(k-1, i);
      // Adjoint(k-1)j
      Matrix<double, 6, 6> adj_k1_j = calc_adjoint_ij(k-1, j);
      
      // Lie bracket twist coord [A(k-1)i * ei, ek]  
      Matrix<double,6,1> twist_lie1 = g::lie_bracket(adj_k1_i * twisti, twistk);
      // Lie bracket twist coord [A(k-1)j * ej, ek]  
      Matrix<double,6,1> twist_lie2 = g::lie_bracket(adj_k1_j * twistj, twistk);
      
      partialM += twist_lie1.transpose() * adj_lk.transpose() * adjusted_gmass * adj_lj * twistj;
      partialM += twisti.transpose() * adj_li.transpose() * adjusted_gmass * adj_lk * twist_lie2;
   }
   
   return partialM;
}

// Should (probably) only be run in calc_coriolis_matrix()
double Robot_Dynamics::calc_christoffel(const int i, const int j, const int k) {
   // Mij_k: Partial derivative or mass matrix element ij wrt to theta k
   double Mij_k = calc_partialM_partialT(i, j, k);
   double Mik_j = calc_partialM_partialT(i, k, j);
   double Mkj_i = calc_partialM_partialT(k, j, i);
   
   double christoffel = 0.5 * (Mij_k + Mik_j - Mkj_i);
   return christoffel;
}

// Make sure to run calc_link_adjusted_gmasss() before this
void Robot_Dynamics::calc_coriolis_matrix() {
   const int js {baxter_ptr->joints};
   
   // Zero out
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         coriolis_matrix(row,col) = 0;
      }
   }
   
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         for(int k{0}; k < js; k++) {
            double theta_dot_k = baxter_ptr->theta_dots(k);
            double christoffel_ijk = calc_christoffel(row, col, k);
            
            coriolis_matrix(row, col) += christoffel_ijk * theta_dot_k;
         } 
      }
   }
}

void Robot_Dynamics::calc_potential_energy() {
   V = 0;
   double g = 9.81;
   for(int link {0}; link < joints; link++) {
      double m = baxter_ptr->mls.at(link);
      MatrixXd p = baxter_ptr->p_links.row(link);
      Matrix<double, 4, 1> p_link_hom {p(0), p(1), p(2), 1};
      // Calculate total twist fromm preceding joints
      Matrix4d gsli = linalg::eye4;
      for(int joint{0}; joint < link; joint++) {
         double t = baxter_ptr->thetas(joint);
         gsli = gsli * g::twist(twist_coords.row(joint), t);
      }
      // Adjusted position of link due to twists
      Matrix<double, 4, 1> p_link_theta = gsli * p_link_hom;
      // Height is z-component
      double h = p_link_theta(2);
      cout << h << endl;
      V += m * g * h;
   } 
}


#endif
