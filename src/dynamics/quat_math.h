#ifndef _QUAT_MATH_H_
#define _QUAT_MATH_H_

#include<iostream>
#include<vector>

#include "../../eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace Eigen;
   
namespace quat_math {
   Matrix3d skew3(Vector3d x) {
      Matrix3d skew3_matrix {
         {0, -x(2), x(1)},
         {x(2), 0, -x(0)},
         {-x(1), x(0), 0}
      };
        return skew3_matrix;
   }

   Matrix<double, 3, 4> quat_skew(Vector4d Q) {
      Matrix<double, 3, 4> quat_skew_matrix {
         {-Q(1), Q(0), Q(3), -Q(2)},
         {-Q(2), -Q(3), Q(0), Q(1)},
         {-Q(3), Q(2), -Q(1), Q(0)}
      };
      return quat_skew_matrix;
   }

   Vector4d conjugate(Vector4d A) {
      Vector4d conj {A(0), -A(1), -A(2), -A(3)};
      return conj;
   }

   Matrix<double, 4, 2> dual_conjugate(Matrix<double, 4, 2> A) {
      Matrix<double, 4, 2> dual_conj {
         {A(0,0), A(0,1)},
         {-A(1,0), -A(1,1)},
         {-A(2,0), -A(2,1)},
         {-A(3,0), -A(3,1)}
      };
      return dual_conj;
   }

   Vector4d quat_prod(Vector4d q, Vector4d p) {
      Matrix3d b3x3 {q(0) * linalg::eye3 + skew3(q(seq(1,3)))};
                            
      Matrix4d Q_plus {
         {q(0), -q(1), -q(2), -q(3)},
         {q(1), b3x3(0,0), b3x3(0,1), b3x3(0,2)},
         {q(2), b3x3(1,0), b3x3(1,1), b3x3(1,2)},
         {q(3), b3x3(2,0), b3x3(2,1), b3x3(2,2)}
      };

      return Q_plus * p;
   }
   
   Vector2d dual_num_prod(Vector2d n1, Vector2d n2) {
      Vector2d prod {n1(0) * n2(0), (n1(0) * n2(1) + n2(0) * n1(1))};
      return prod;
   }
   
   
   Matrix<double, 3, 2> dual_num_dual_vec(Vector2d D, Matrix<double, 3, 2> E){
      Vector2d DE1 = dual_num_prod(D, E(0, seq(0,1)));
      Vector2d DE2 = dual_num_prod(D, E(1, seq(0,1)));
      Vector2d DE3 = dual_num_prod(D, E(2, seq(0,1)));
      Matrix<double, 3, 2> DE {
         {DE1(0), DE1(1)},
         {DE2(0), DE2(1)},
         {DE3(0), DE3(1)}
      };
      return DE;
   }
   
   Matrix<double, 3, 2> dual_vec_cross(Matrix<double, 3, 2> E, Matrix<double, 3, 2> F) {
      Vector2d E1 = E(0, seq(0,1));
      Vector2d E2 = E(1, seq(0,1));
      Vector2d E3 = E(2, seq(0,1));
      Vector2d F1 = F(0, seq(0,1));
      Vector2d F2 = F(1, seq(0,1));
      Vector2d F3 = F(2, seq(0,1));
      Vector2d row1 = dual_num_prod(E2, F3) - dual_num_prod(E3, F2);
      Vector2d row2 = dual_num_prod(E3, F1) - dual_num_prod(E1, F3);
      Vector2d row3 = dual_num_prod(E1, F2) - dual_num_prod(E2, F1);
      Matrix<double, 3, 2> EF {
         {row1(0), row1(1)},
         {row2(0), row2(1)},
         {row3(0), row3(1)}
      };
      return EF;
   }
   
   // The product of 2 dual vectors
   Vector2d dual_vec_prod(Matrix<double, 3, 2> E, Matrix<double, 3, 2> F) {
      Vector2d EF1 = dual_num_prod(E(0, seq(0,1)), F(0, seq(0,1)));
      Vector2d EF2 = dual_num_prod(E(1, seq(0,1)), F(1, seq(0,1)));
      Vector2d EF3 = dual_num_prod(E(2, seq(0,1)), F(2, seq(0,1)));
      return EF1 + EF2 + EF3;
   }

   // The product of 2 dual quaternions
   Matrix<double, 4, 2> dual_quat_prod(Matrix<double, 4, 2>  A, Matrix<double, 4, 2>  B) {
      // Scalar Component
      Vector2d As = A(0, seq(0,1));
      Vector2d Bs = B(0, seq(0,1));
      Matrix<double, 3, 2> Av = A(seq(1,3), seq(0,1));
      Matrix<double, 3, 2> Bv = B(seq(1,3), seq(0,1));
      Vector2d AsBs = dual_num_prod(As, Bs);
      Vector2d ABs = AsBs - dual_vec_prod(Av, Bv);

      // Vector Component
      Matrix<double, 3, 2> AsBv = dual_num_dual_vec(As, Bv);
      Matrix<double, 3, 2> BsAv = dual_num_dual_vec(Bs, Av);
      Matrix<double, 3, 2> AvBv = dual_vec_cross(Av, Bv);
      Matrix<double, 3, 2> ABv = (AsBv + BsAv) + AvBv;
      Matrix<double, 4, 2> AB {
         {ABs(0), ABs(1)},
         {ABv(0,0), ABv(0,1)},
         {ABv(1,0), ABv(1,1)},
         {ABv(2,0), ABv(2,1)}
      };
      return AB;
   }
   
  Vector3d dual_quat_to_p(Matrix<double, 4, 2> A) {
      Vector4d Ar = A(seq(0,3), 0);
      Vector4d Ar_conj = conjugate(Ar);
      Vector4d Ad = A(seq(0,3), 1);
      Vector4d p_4x1 = quat_prod((2 * Ad), Ar_conj);
      Vector3d p = p_4x1(seq(1,3));
      return p;
   }
   
   Matrix<double, 4, 2> power_of_dual_quat(Matrix<double, 4, 2> A, double tau) {
      Vector4d Ar = A(seq(0,3), 0);
      double theta = 2 * acos(Ar(0));
      Vector3d u = Ar(seq(1,3)) / sin(theta / 2);
      Vector3d p = dual_quat_to_p(A);
      double d = p.dot(u);
      Vector3d m = (0.5) * (p.cross(u) + ((p - (d * u)) * (1 / tan(theta / 2))));
      Matrix<double, 3, 2> u_hat {
         {u(0), m(0)},
         {u(1), m(1)},
         {u(2), m(2)}
      };
      
      Vector2d s {sin((tau * theta) / 2), ((tau * d) / 2) * cos((tau * theta) / 2)};
      Vector2d c {cos((tau * theta) / 2), -(((tau * d) / 2)) * sin((tau * theta) / 2)};
      Matrix<double, 3, 2> D_vector = dual_num_dual_vec(s, u_hat);
      Matrix<double, 4, 2> D_tau {
         {c(0), c(1)},
         {D_vector(0,0), D_vector(0,1)},
         {D_vector(1,0), D_vector(1,1)},
         {D_vector(2,0), D_vector(2,1)}
      };

      return D_tau;
   }
   Vector4d rot_to_quat(Matrix3d R) {
      double r11 = R(0,0);
      double r12 = R(0,1);
      double r13 = R(0,2);

      double r21 = R(1,0);
      double r22 = R(1,1);
      double r23 = R(1,2);

      double r31 = R(2,0);
      double r32 = R(2,1);
      double r33 = R(2,2);

      Matrix4d M {
         {1, 1, 1, 1},
         {1, -1, -1, 1},
         {-1, 1, -1, 1},
         {-1, -1, 1, 1}
      };
      
      Vector4d D {r11, r22, r33, 1};
      Vector4d Q = 0.25 * (M*D);  
      double q_max {0};
      int q_max_index {0};
      for(int i{0}; i<4; i++) {
         if(Q(i) > q_max) { 
            q_max = Q(i);
            q_max_index = i;
         }
      }
      
      double q0;
      double q1; 
      double q2;
      double q3;

      if(q_max_index == 0) {
         q0 = sqrt(q_max);
         q1 = (r32 - r23) / (4 * q0);
         q2 = (r13 - r31) / (4 * q0);
         q3 = (r21 - r12) / (4 * q0);
      }

      if(q_max_index == 1) {
         q1 = sqrt(q_max);
         q0 = (r32 - r23) / (4 * q1);
         q2 = (r12 + r21) / (4 * q1);
         q3 = (r13 + r31) / (4 * q1);
      }

      if(q_max_index == 2) {
         q2 = sqrt(q_max);
         q0 = (r13 - r31) / (4 * q2);
         q1 = (r12 + r21) / (4 * q2);
         q3 = (r23 + r32) / (4 * q2);
      }

      if(q_max_index == 3) {
        q3 = sqrt(q_max);
        q0 = (r21 - r12) / (4 * q3);
        q1 = (r13 + r31) / (4 * q3);
        q2 = (r23 + r32) / (4 * q3);
      }
      Vector4d q {q0, q1, q2, q3};
      return q;
   }
        
   Matrix<double, 4, 2> g_to_dual_quat(Matrix4d g) {
      Matrix3d R = g(seq(0,2), seq(0,2));
      Vector3d p = g(seq(0,2), 3);
      Vector4d p_quat {0, p(0), p(1), p(2)};
      Vector4d Ar = rot_to_quat(R);
      Vector4d Ad = 0.5 * quat_prod(p_quat, Ar);
      Matrix<double, 4, 2> A {
         {Ar(0), Ad(0)},
         {Ar(1), Ad(1)},
         {Ar(2), Ad(2)},
         {Ar(3), Ad(3)}
      };
      return A;
   }
   
   
   Matrix<double, 7, 1> g_to_gamma(Matrix4d g) {
      Matrix3d R = g(seq(0,2), seq(0,2));
      Vector3d p = g(seq(0,2), 3);
      Vector4d Ar = rot_to_quat(R);
      Matrix<double, 7, 1> gamma {p(0), p(1), p(2), Ar(0), Ar(1), Ar(2), Ar(3)};
      return gamma;
   }

   Matrix<double, 7, 1> dual_quat_to_gamma(Matrix<double, 4, 2> A) {
      Vector4d Ar = A(seq(0,3), 0);
      Vector4d Ad = A(seq(0,3), 1);
      Vector4d p = 2 * quat_prod(Ad, conjugate(Ar));
      Matrix<double, 7, 1> gamma {p(1), p(2), p(3), Ar(0), Ar(1), Ar(2), Ar(3)};
      return gamma;
   }
   
}

#endif
