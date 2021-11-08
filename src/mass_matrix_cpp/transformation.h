#ifndef _TRANSFORMATION_H_
#define _TRANSFOMRATION_H_

#include<iostream>
#include<vector>

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

namespace g {

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

}

#endif
