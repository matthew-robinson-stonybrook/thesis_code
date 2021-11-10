#ifndef _PARSER_H_
#define _PARSER_H_

#include <iostream>

using namespace std;

namespace parser {

   // Parses through a line and checks if it is a joint declaration
   // Once a '<' is found, checks next word for 'joint',
   // If it is, sets the 'in_joint_bool' boolean to true
   void check_for_joint_dec(ifstream &input, bool &in_joint_bool) {
      
      string empty;
      string next_word;
      string line; 
      
      // Parse through each line until '<' is reached
      // Store empty text in string 'empty'
      getline(input, empty, '<');
      
      // Get the next word after the '<' initializer
      getline(input, next_word, ' ');
      
      if (next_word == "joint") {
         cout << "ENTERING JOINT DECLARATION" << endl;
         in_joint_bool = true;
      }
      
      else {
         getline(input, line, '>');
         input.get();
      }
   }
   
}

#endif
