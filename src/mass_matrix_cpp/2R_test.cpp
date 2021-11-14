#include<iostream>
#include "spatial_jacobian.h"
#include "Baxter.h"
#include "linalg.h"
#include "transformation.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

const double pi = 3.14159265359;

const double a1 = 10;
const double a2 = 10;
const double l1 = 5;
const double l2 = 5; 

Matrix<double, 2, 1> ts {pi/5, pi/4};

Matrix<double, 2, 3> axis_joints {
   {0, 0, 1},
   {0, 0, 1}
};

Matrix<double, 2, 3> q_joints {
   {0, 0, 0},
   {a1, 0, 0}
};

Matrix<double, 2, 3> q_link {
   {l1, 0, 0},
   {a1 + l1, 0, 0}
};

Matrix4d g0 {
   {1, 0, 0, 20},
   {0, 1, 0, 0},
   {0, 0, 1, 0},
   {0, 0, 0, 1}
};


int main() {

   double t1 {ts(0)};
   double t2 {ts(1)};

   Matrix<double, 3, 2> jp_l2 {
      {-a1*sin(t1) - l2*sin(t1 + t2), -l2*sin(t1 + t2)},
      {a1*cos(t1) + l2*cos(t1 + t2), l2*cos(t1 + t2)},
      {0,0},
   };
   //Desired jacoban
   cout << "Desired Jacobian: " << endl;
   cout << jp_l2 << endl;
   
   //Jac
   Spatial_Jacobian jac(axis_joints, q_joints, ts);

   // The spatial jacobian isn't necessary for this, but was computed
   // Because the twist computation is in that func, and we need the twists
   jac.calculate_jacobian();
   MatrixXd spatial_jac = jac.spatial_jacobian;
   
   // Transformation matrix due to joint 1 and joint 2
   Matrix4d g1 = g::twist(jac.twists.row(0), t1);
   Matrix4d g2 = g::twist(jac.twists.row(1), t2);  
   
   // Initial position/orientation of link 2
   Matrix4d g_link20 {
      {1, 0, 0, q_link(1, 0)},
      {0, 1, 0, q_link(1, 1)},
      {0, 0, 1, q_link(1, 2)},
      {0, 0, 0, 1},
   };
   
   // Transformed pos/orientation of link2
   Matrix4d g_link2_prime = g1 * g2 * g_link20;
   cout << g_link2_prime << endl;
   
   //
   MatrixXd v {g_link2_prime.col(3)};
   Vector3d p {v(0), v(1), v(2)};
   Vector3d jac_column = axis_joints.row(0).cross(p);
   cout << jac_column << endl;
   
   return 0;
}
