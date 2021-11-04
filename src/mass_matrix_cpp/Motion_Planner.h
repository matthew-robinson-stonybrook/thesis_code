#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include<iostream>
#include<vector>
#include "spatial_jacobian.h"

#include </home/matt_robinson/cpp_libraries/eigen-3.4.0/Eigen/Dense>

using namespace std;
using namespace Eigen;

//spatial jacobian
class Motion_Planner {
   public:
      Matrix<double, 4, 4> gst0;
      Matrix<double, 4, 4> g_init;
      MatrixXd config_init;
      Matrix<double, 4, 4> gstf;
      
      MatrixXd axis_joints;
      MatrixXd q_joints;
      MatrixXd thetas;

      Motion_Planner();
      Motion_Planner(MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd);
};

Motion_Planner::Motion_Planner(){
   return 0;
}

Motion_Planner::Motion_Planner(MatrixXd g0, MatrixXd gi, MatrixXd configi, MatrixXd gf,MatrixXd ajs, MatrixXd qjs, MatrixXd ts, MatrixXd){
   gst0 = g0;
   g_init = gi;
   config_init = configi;
   gstf = gf;
   axis_joints = ajs;
   q_joints = qjs;
   thetas = ts;
}
   

#endif
