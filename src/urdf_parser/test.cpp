#include <iostream>
#include <fstream>
#include <vector>

#include "parser.h"

using namespace std;

string txt_file = "test_stats.txt";
string urdf_file = "urdf_test.urdf";

vector <string> joint_names;

string str_joint = "joint";
string str_end_joint = "/joint";

string empty;
string line;
string next_word;
int file_pos;

bool in_joint {false};

int main() {
   // Create ifstream object
   string filename = urdf_file;
   ifstream input;
   
   // Error checker
   input.open(filename);
   if (!input.is_open()) {
      return 1;
   }
   
   // MAIN LOOP
   while(!input.eof()) {
      // If not already parsing a joint declaration, 
      // check next line for joint declaration
      if (!in_joint) {
         parser::check_for_joint_dec(input, in_joint);
      }
      
      cout << in_joint << endl;
   }
   
   input.close();
   
   return 0;
}
