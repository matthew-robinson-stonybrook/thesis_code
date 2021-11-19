#include<iostream>
#include<vector>

using namespace std;

void factorize(vector<int>& f, int n) {
   
   for (int i{1}; i <= n; i++) {
      if (n%i == 0) {
         answer.push_back(i);
      }
   }
   return answer;
}

void print_vectors(vector<int>& v) {
   cout << v.at(3) << endl;
}

int main() {
   cout << abs(4)<< endl;
   return 0;
}
