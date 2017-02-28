#ifndef DAG_OP
#define DAG_OP

#define NINF -2000000
#define SMALL 0.000001
#include "assignment/Assignment.h"
#include "utils/topology.h"
#include "utils/argparser.h"
#include <cmath>
#include <fstream>
#include <queue>

using namespace std;

class Dag
{

public:
  Dag(){}
  ~Dag(){}

  vector<real_vertex>& getNodes(void){ return nodes; }
  void createNodes(uint _size, uint _seed, double radius);
  void setStartEnd(int _start, int _end){ start = _start; end = _end; }
  void setT(int _T){T = _T;}
  void setResolution(double _resolution){ resolution = _resolution;}
  int getDistance(int i, int j);
  double getRealDistance(int i, int j);
  void init();
  void init_Graph();
  vector<int> bfs();
  vector<int> topOrder();
  vector<int> MaximumPath();

  //optimized solution
  void numerate(vector<int>& flag, vector<int> nums, int level );
  void permute(vector<int>& nums);
  vector<int> OptimizedSol();
  double calculateScore(vector<int> path);
  double calculateCost(vector<int> path);
  double scoreFunc(int i, int t);
  vector<int> findPath();
private:
  vector<real_vertex> nodes; 
  vector<vertex> v; //time model nodes

  vector<vector<int> > dpath; //for each state v[i], the distinct node path 
  vector<vector<int> > test;

  vector<vector<int>> per_res; // the result of permutation
  vector<int> cur;
    
  int start,end;
  int T;
  int eT; //estimate T, eT=T*0.95
  double resolution; //default 1.0
};

#endif


