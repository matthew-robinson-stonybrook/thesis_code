#ifndef _POS_DEF_TEST_H_
#define _POS_DEF_TEST_H_

#include<iostream>

namespace mass_tester {

   // A Function to check if the mass matrix is symmetric
   bool symmetric(MatrixXd mass) {
      bool symmetric {true};
      
      int columns = mass.cols();
      int rows = mass.rows();
      
      for (int col{0}; col < columns; col++) {
         for (int row{0}; row < rows; row++) {
            double error = (mass(col, row) - mass(row,col)) / mass(col, row);
            if (error >= 0.01) {
               symmetric = false;
               return symmetric;
            }
         }
      }
      
      return symmetric;
   }
   
   bool pos_def(MatrixXd mass) {
      bool pos_def {true};
      
      for (int size{2}; size <= mass.rows(); size++) {
         if (mass(seqN(0,size), seqN(0,size)).determinant() <= 0) {
            pos_def = false;
            return pos_def;
         }
      }
      
      return pos_def;
   }

}

#endif 
