#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include<iostream>
#include<vector>

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

// g namespace has functions for computing transformation matrices
// given different quantities, as well as the adjoint matrix
namespace g {

   // Transformation matrix given axis vector, position vector to axis, and theta
   Matrix4d axis(const Vector3d &axis_joint, const Vector3d &q_joint, const double &theta) {
      Matrix3d skew = linalg::skew3(axis_joint);
      
      Matrix3d rot = linalg::eye3 + skew*sin(theta) + skew*skew * (1-cos(theta));
      
      Matrix3d a = linalg::eye3 - rot;
      Vector3d trans = a*q_joint;
      
      Matrix<double, 4, 4> transformation;
      transformation << rot(0,0), rot(0,1), rot(0,2), trans(0),
      			 rot(1,0), rot(1,1), rot(1,2), trans(1),
      			 rot(2,0), rot(2,1), rot(2,2), trans(2),
      			 0, 0, 0, 1;
      			 
      return transformation;				 
   }
   
   // Transformation matrix given twist vector and theta
   Matrix4d twist(const Matrix<double, 6, 1> &twist, const double &theta) {
      Vector3d v = {twist(0), twist(1), twist(2)};
      Vector3d w = {twist(3), twist(4), twist(5)};
      
      Matrix3d skew = linalg::skew3(w);
      
      Matrix3d rot = linalg::eye3 + skew*sin(theta) + skew*skew * (1-cos(theta));
      
      Vector3d a = (linalg::eye3 - rot) * w.cross(v);
      Vector3d b = (w * w.transpose()) * v * theta;
      Vector3d trans = a + b;
      
      Matrix<double, 4, 4> transformation;
      transformation << rot(0,0), rot(0,1), rot(0,2), trans(0),
      			 rot(1,0), rot(1,1), rot(1,2), trans(1),
      			 rot(2,0), rot(2,1), rot(2,2), trans(2),
      			 0, 0, 0, 1;
      			 
      return transformation;	
      			 
   }
   
   //Adjoint matrix given transformation matrix
   Matrix<double, 6, 6> adjoint(MatrixXd g) {
      Matrix3d rot = g(seq(0,2), seq(0,2));
      Vector3d p = g(seq(0, 2), 3);
      Matrix3d p_hat = linalg::skew3(p);
      Matrix3d pR = p_hat * rot;
      
      Matrix<double, 6, 6> adjoint {
      {rot(0,0), rot(0, 1), rot(0,2), pR(0,0), pR(0,1), pR(0,2)},
      {rot(1,0), rot(1, 1), rot(1,2), pR(1,0), pR(1,1), pR(1,2)},
      {rot(2,0), rot(2, 1), rot(2,2), pR(2,0), pR(2,1), pR(2,2)},
      {0, 0, 0, rot(0,0), rot(0,1), rot(0,2)},
      {0, 0, 0, rot(1,0), rot(1,1), rot(1,2)},
      {0, 0, 0, rot(2,0), rot(2,1), rot(2,2)},
      };
      
      return adjoint;
   }
}

#endif
