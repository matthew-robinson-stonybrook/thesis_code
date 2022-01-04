#ifndef _CORIOLIS_MATRIX_H_
#define _CORIOLIS_MATRIX_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "Baxter.h"

#include "linalg.h"
#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Coriolis_Matrix {
   public:
      Baxter *baxter;
      MatrixXd mass_matrix;
      
};

#endif
