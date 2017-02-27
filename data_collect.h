#ifndef DATA_COLLECT
#define DATA_COLLECT
#include <iostream>
#include <fstream>
#include <sstream>
#include "utils/topology.h"
#include <cmath>
#include <vector>

using namespace std;
// TODO
extern int test_collect;

vector<real_vertex> data_collect();
vector<vector<vector<int> > > collect_score();

#endif