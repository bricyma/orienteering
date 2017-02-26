#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace std;

void loadMatrix(string matrix_file, int size);
void init();
void tsp_opt();
vector<int> tsp(string matrix_file, int size);


extern vector<int> assignment;
extern vector<vector<double> > mat;