#ifndef _LINALG_H_
#define _LINALG_H_

#include<iostream>
#include<vector>

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;
   
namespace linalg {
   Matrix3d eye3 {
   {1, 0, 0},
   {0, 1, 0},
   {0, 0, 1}
   };
   
   Matrix3d skew3(Vector3d vec3) {
      Matrix3d s3 {
      {0, -vec3(2), vec3(1)},
      {vec3(2), 0, -vec3(0)},
      {-vec3(1), vec3(0), 0}
      };
      return s3;
   }

}

#endif
