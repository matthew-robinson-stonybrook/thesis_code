#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include<iostream>
#include<vector>
#include "transformation.h"
#include "linalg.h"
#include "Baxter.h"
#include "manip.h"

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Controller {
   public:
      Controller(Manip*);
      Manip *manip_ptr;
      
      Matrix<double, 6, 6> Md {linalg::eye6};
      Matrix<double, 6, 6> Kd {linalg::eye6};
      Matrix<double, 6, 6> Kp {linalg::eye6};
      
      Matrix<double, 6, 1> x {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_dot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> x_ddot {0, 0, 0, 0, 0, 0};
      Matrix<double, 6, 1> xd_ddot {0, 0, 0, 0, 0, 0};
   
};

Controller::Controller(Manip* manip) : manip_ptr{manip} {
   cout << "Controller Contructor" << endl;
}

#endif
