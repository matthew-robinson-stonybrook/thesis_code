#ifndef _LINALG_H_
#define _LINALG_H_

#include<iostream>
#include<vector>

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;
   
namespace linalg {
   // Identity matrices
   Matrix3d eye3 {MatrixXd::Identity(3,3)};
   Matrix4d eye4 {MatrixXd::Identity(4,4)};
   Matrix<double,5,5> eye5 {MatrixXd::Identity(5,5)};
   Matrix<double,6,6> eye6 {MatrixXd::Identity(6,6)};
   // Zero Matrices
   Matrix3d zero3 {MatrixXd::Zero(3,3)};
   Matrix4d zero4 {MatrixXd::Zero(4,4)};
   Matrix<double,5,5> zero5 {MatrixXd::Zero(5,5)};
   Matrix<double,6,6> zero6 {MatrixXd::Zero(6,6)};
   
   Matrix3d skew3(Vector3d vec3) {
      Matrix3d s3 {
      {0, -vec3(2), vec3(1)},
      {vec3(2), 0, -vec3(0)},
      {-vec3(1), vec3(0), 0}
      };
      return s3;
   }
   
   Vector3d vec3_from_skew(Matrix3d skew3) {
      Vector3d vec3 {-skew3(1,2), skew3(0,2), -skew3(0,1)};
      return vec3;
   }
   
   Matrix4d twist_skew(Matrix<double, 6, 1> twist) {
      Vector3d v {twist(0), twist(1), twist(2)};
      Vector3d w {twist(3), twist(4), twist(5)};
      Matrix3d w_hat {skew3(w)};
      
      Matrix4d twist_hat {
      {w_hat(0,0), w_hat(0,1), w_hat(0,2), v(0)},
      {w_hat(1,0), w_hat(1,1), w_hat(1,2), v(1)},
      {w_hat(2,0), w_hat(2,1), w_hat(2,2), v(2)},
      {0, 0, 0, 0}
      };
      
      return twist_hat;
   }
   
   Matrix<double, 6, 1> twist_from_skew(Matrix4d twist_hat) {
      Matrix3d w_hat {
      {twist_hat(0,0), twist_hat(0,1), twist_hat(0,2)},
      {twist_hat(1,0), twist_hat(1,1), twist_hat(1,2)},
      {twist_hat(2,0), twist_hat(2,1), twist_hat(2,2)}
      };
      
      Vector3d w {vec3_from_skew(w_hat)};
      Matrix<double, 6, 1> twist {
      twist_hat(0,3), 
      twist_hat(1,3),
      twist_hat(2,3),
      w(0), w(1), w(2)
      };
      
      return twist;
   }

}

#endif
