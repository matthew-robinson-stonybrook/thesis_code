#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
#include "manips/Baxter.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Robot_Dynamics {
   public:
      Robot_Dynamics(Manip*);
      Manip *manip_ptr;
      
      int joints;
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd twist_coords;
      
      Matrix4d ge;
      // End effector position and orientation vector
      Matrix<double, 6, 1> x {0, 0, 0, 0, 0, 0};
      
      
      MatrixXd spatial_jac;
      MatrixXd spatial_jac_dot;
      MatrixXd analytic_jac;
      MatrixXd analytic_jac_dot;
      MatrixXd analytic_jac_pseudo_inv;
      
      // Spatial to analytic jacobian map
      Matrix<double, 6, 6> A;
      Matrix<double, 6, 6> A_dot;
      
      vector <Matrix<double,6,6>> link_gmasss;
      vector <Matrix<double,6,6>> link_adjusted_gmasss;
      
      MatrixXd mass_matrix;
      MatrixXd coriolis_matrix;
      MatrixXd gravity_term;
      double V;
      double T;
      
      void calc_twist_coords();
      void calc_link_gmasss();
      
      void calc_link_adjusted_gmasss();
      
      Matrix<double, 6, 6> calc_gmass(double mass, Matrix3d inertia);
      Matrix<double, 6, 6> calc_adjusted_gmass(Matrix<double, 6, 6> gmass, int link);
      Matrix<double, 6, 6> calc_adjoint_ij(int i, int j);
      double calc_partialM_partialT(const int i, const int j, const int k);
      double calc_christoffel(const int i, const int j, const int k);
      
      void calc_mass_matrix();
      void calc_coriolis_matrix();
      void calc_gravity_term();
      
      void calc_potential_energy();
      void calc_kinetic_energy();
      
      void calc_spatial_jac();
      void calc_spatial_jac_dot();
      // NOTE the analytic jacobian is derived from the spatial jacobian
      // So the spatial jac will first be calculated in function
      void calc_analytic_jac();
      void calc_analytic_jac_dot();
      void calc_analytic_jac_pseudo_inv();
      // Map between spatial and analytic jacs
      void calc_A();
      void calc_A_dot();
      void forward_kin();
      
};

// Contructor
Robot_Dynamics::Robot_Dynamics(Manip* manip) : manip_ptr{manip} {
   cout << "Robot Dynamics Constructor" << endl;
   
   joints = manip_ptr->get_joints();
   axis_joints = manip_ptr->get_axis_joints();
   q_joints = manip_ptr->get_q_joints();
   mass_matrix.resize(manip_ptr->get_joints(), manip_ptr->get_joints());
   coriolis_matrix.resize(manip_ptr->get_joints(), manip_ptr->get_joints());
   gravity_term.resize(manip_ptr->get_joints(), 1);
   
   // resize matrices to be respective of joints
   spatial_jac.resize(6, manip_ptr->get_joints());
   spatial_jac_dot.resize(6, manip_ptr->get_joints());
   analytic_jac.resize(6, manip_ptr->get_joints());
   analytic_jac_dot.resize(6, manip_ptr->get_joints());
   analytic_jac_pseudo_inv.resize(6, manip_ptr->get_joints());
   
   calc_twist_coords();
   calc_link_gmasss();
   calc_link_adjusted_gmasss();
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
      twist_coords(joint, seq(0,2)) = cross;
      twist_coords(joint, seq(3,5)) = uj;

   }
}

// HAPPENS ONLY IN CONTRUCTOR
// Creates vector of general mass matrices for the links (Mi)
void Robot_Dynamics::calc_link_gmasss() {
   link_gmasss.clear();
   
   double m;
   Matrix3d I;

   for(int j{0}; j < joints; j++) {
      m = manip_ptr->get_mls().at(j);
      I = manip_ptr->get_Ils().at(j);
      link_gmasss.push_back(calc_gmass(m, I));
   }
}

// HAPPENS ONLY IN CONTRUCTOR
// Calculates adjusted mass matrices 
void Robot_Dynamics::calc_link_adjusted_gmasss() {
   link_adjusted_gmasss.clear();
   for(int j{0}; j < joints; j++) {
      Matrix<double,6,6> gmass = link_gmasss.at(j);
      link_adjusted_gmasss.push_back(calc_adjusted_gmass(gmass, j));
   }
}

// Should only be run in calc_gmasss() in contructor
Matrix<double, 6, 6> Robot_Dynamics::calc_gmass(double mass, Matrix3d inertia) {
   Matrix<double, 6, 6> gen_mass {linalg::zero6};
   gen_mass(seq(3,5), seq(3,5)) = inertia;
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
      double theta = manip_ptr->get_thetas()(joint);
      Matrix4d gj = g::twist(twist_coords.row(joint), theta);
      gsli0 = gsli0 * gj;
   }
   MatrixXd p_link = manip_ptr->get_p_links().row(link);
   Matrix4d gsli = linalg::eye4;
   gsli(seq(0,2), 3) = p_link.transpose();
   gsli0 = gsli;
   
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
         g *= g::twist(twist_coords.row(k), manip_ptr->get_thetas()(k));
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
   
   const int js {manip_ptr->get_joints()};
   
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
   const int js {manip_ptr->get_joints()};
   
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
      
      // Adjointki
      Matrix<double, 6, 6> adj_ki = calc_adjoint_ij(k, i);
      // Adjointkj
      Matrix<double, 6, 6> adj_kj = calc_adjoint_ij(k, j);
      
      // Lie bracket twist coord [A(k-1)i * ei, ek]  
      Matrix<double,6,1> twist_lie1 = g::lie_bracket(adj_ki * twisti, twistk);
      // Lie bracket twist coord [A(k-1)j * ej, ek]  
      Matrix<double,6,1> twist_lie2 = g::lie_bracket(adj_kj* twistj, twistk);
      
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
   const int js {manip_ptr->get_joints()};
   
   // Zero out
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         coriolis_matrix(row,col) = 0;
      }
   }
   
   for(int row{0}; row < js; row++) {
      for(int col{0}; col < js; col++) {
         double sum = 0;
         for(int k{0}; k < js; k++) {
            double theta_dot_k = manip_ptr->get_theta_dots()(k);
            double christoffel_ijk = calc_christoffel(row, col, k);
            sum += christoffel_ijk * theta_dot_k;
         }
         coriolis_matrix(row, col) = sum;
      }
   }
}

void Robot_Dynamics::calc_gravity_term() {
   Vector4d g {0, 0, 9.81, 0};
   vector <Matrix4d> g1_is {linalg::eye4};
   
   // Vector of adjusted link positions to be calculated next
   vector <Vector4d> ps {};
   // Vector of adjusted twists to be calculated next
   vector <Matrix4d> twists{};
   for(int link {0}; link < joints; link++) {
      Vector3d p = manip_ptr->get_p_links().row(link);
      Vector4d ph = {p(0), p(1), p(2), 1};
      double t = manip_ptr->get_thetas()(link);
      
      Matrix4d transformation = g::twist(twist_coords.row(link), t);
      g1_is.push_back(g1_is.at(link) * transformation);
      
      Vector4d ph_prime = g1_is.at(link+1) * ph;
      Matrix<double, 6, 1> twist_coord_prime = g::adjoint(g1_is.at(link)) * twist_coords.row(link).transpose();
      Matrix4d twist_prime = linalg::twist_skew(twist_coord_prime);
      
      ps.push_back(ph_prime);
      twists.push_back(twist_prime);
   }
   
   
   Vector4d sum {0, 0, 0, 0};
   // Calculating rows of gravity term
   for(int row {0}; row < joints; row++) {
      sum = {0, 0, 0, 0};
      cout << "ROW: " << endl;
      cout << row << endl;
      for(int link {row}; link < joints; link++) {
         Vector4d p = ps.at(link);
         double m = manip_ptr->get_mls().at(link);
         sum += m * p;
         /*
         cout << "LINK: " << endl;
         cout << link << endl;
         cout << "SUM: " << endl;
         cout << sum << endl;
         */
      }
      
      Matrix<double, 6, 1> twist_p = spatial_jac.col(row);
      Matrix4d twistp_hat = linalg::twist_skew(twist_p);
      
      cout << "Twist Hat: " << endl;
      cout << twistp_hat << endl;
      cout << "SUM: " << endl;
      cout << sum << endl;
      
      gravity_term(row) = g.transpose() * twistp_hat * sum;
   }
}

// sum of link mass * g * link COM height (h)
void Robot_Dynamics::calc_potential_energy() {
   V = 0;
   double g = 9.81;
   for(int link {0}; link < joints; link++) {
      double m = manip_ptr->get_mls().at(link);
      MatrixXd p = manip_ptr->get_p_links().row(link);
      Matrix<double, 4, 1> p_link_hom {p(0), p(1), p(2), 1};
      // Calculate total twist fromm preceding joints
      Matrix4d gsli = linalg::eye4;
      for(int joint{0}; joint < link; joint++) {
         double t = manip_ptr->get_thetas()(joint);
         gsli = gsli * g::twist(twist_coords.row(joint), t);
      }
      // Adjusted position of link due to twists
      Matrix<double, 4, 1> p_link_theta = gsli * p_link_hom;
      // Height is z-component
      double h = p_link_theta(2);
      V += m * g * h;
   } 
}

// theta_dot^T * M * theta_dot (1/2mv^2)
void Robot_Dynamics::calc_kinetic_energy() {
   MatrixXd a = (manip_ptr->get_theta_dots().transpose() * mass_matrix * manip_ptr->get_theta_dots());
   T = a(0,0)/2;
}

void Robot_Dynamics::forward_kin() {
   ge = linalg::eye4;
   double t;
   for (int joint {0}; joint < joints; joint++) {
      t = manip_ptr->get_thetas()(joint);
      ge *= g::twist(twist_coords.row(joint), t);
   }
   ge *= manip_ptr->get_gst0();
}

void Robot_Dynamics::calc_spatial_jac() {
   vector <Matrix4d> g1_is {linalg::eye4};
   for(int joint {0}; joint < manip_ptr->get_joints(); joint++) {
      double t = manip_ptr->get_thetas()(joint);
      Matrix4d transformation = g::twist(twist_coords.row(joint), t);
      g1_is.push_back(g1_is.at(joint) * transformation);
      Matrix<double, 6, 6> adj = g::adjoint(g1_is.at(joint));
      Matrix<double, 6, 1> twist_prime = adj * twist_coords.row(joint).transpose();
      
      spatial_jac(seq(0,5), joint) = twist_prime; 
   }
}


void Robot_Dynamics::calc_spatial_jac_dot() {
   calc_spatial_jac();
   MatrixXd t_dot = manip_ptr->get_theta_dots();
   cout << "Theta Dot: " << endl;
   cout << t_dot << endl;
   Matrix<double, 6, 1> sum;
   for(int col{0}; col < joints; col++) {
      sum = {0, 0, 0, 0, 0, 0};
      /*
      This next for loop is essentially creating the twist jacobian and
      multiplying by the joint velocity vector for this twist 'col'
      */
      for(int joint{0}; joint < joints; joint++) {
         if(joint < col) {
            // Column 'joint' of the current spat jac
            Matrix<double, 6, 1> twistj_prime {spatial_jac.col(joint)};
            // Column 'col' of the current spat jac
            Matrix<double, 6, 1> twisti_prime {spatial_jac.col(col)};
            // Twist i prime derivative wrt theta j is Lie Bracket j,i
            Matrix<double, 6, 1> twisti_dj = g::lie_bracket(twistj_prime, twisti_prime);
            
            // Twist Jacobian Matrix times joint vel vector
            sum += twisti_dj * t_dot(joint);
         }
         spatial_jac_dot(seq(0,5), col) = sum;
      }
   }
}

void Robot_Dynamics::calc_analytic_jac() {
   calc_spatial_jac();
   calc_A();
   analytic_jac = A * spatial_jac;
}

void Robot_Dynamics::calc_analytic_jac_dot() {
   calc_spatial_jac();
   calc_spatial_jac_dot();
   calc_A();
   calc_A_dot();
   analytic_jac_dot = A_dot * spatial_jac + A * spatial_jac_dot;
}

void Robot_Dynamics::calc_analytic_jac_pseudo_inv() {
   analytic_jac_pseudo_inv = spatial_jac.transpose() * ((spatial_jac * spatial_jac.transpose()).inverse());
}

void Robot_Dynamics::calc_A() {
   forward_kin();
   Vector3d p = ge(seq(0,2), 3);
   A = linalg::eye6;
   Matrix3d p_hat = linalg::skew3(p);
   A(seq(0,2), seq(3,5)) = -p_hat; 
}

void Robot_Dynamics::calc_A_dot() {
   MatrixXd t_dot = manip_ptr->get_theta_dots();
   Matrix<double, 6, 1> va = analytic_jac * t_dot;
   Vector3d p_dot = va(seq(0,2));
   Matrix3d p_dot_hat = linalg::skew3(p_dot);
   
   A_dot = linalg::zero6;
   A_dot(seq(0,2), seq(3,5)) = p_dot_hat;
}

#endif
