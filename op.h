#ifndef ORIENTEERING
#define ORIENTEERING

#include "assignment/Assignment.h"
#include "utils/topology.h"
#include "utils/argparser.h"
#include "tsp.h"
#include <cmath>
#include <fstream>

//extern int      StartID;
//extern int      EndID;

using namespace std;

class Orienteering
{

public:
  Orienteering(){first = true;}
  ~Orienteering(){}

  vector<node_t>& getNodes(void){ return nodes; }

  void createNodes(uint _size, uint _seed, double radius);
  void init(void);
  void setStartEnd(int _start, int _end){ start = _start; end = _end; }
  void setT(int _T){ T = _T;}
  int getDistance(int i, int j);
  double totalCost(void);
  double totalPathCost(vector<int> path);
  double totalScore(void);

  int edge_len(int i, int j, int t);
  node2 findNodeByIdT(int node_id, int node_t);
  void initGraph();
  void dijkstra();

  vector<int> getPath();
  vector<int> path(void);
  point2d_t calculateGravity(void);
  point2d_t calculateGravityAll(void);
  int calculateRank(int id,  double a, double b, double c);
  int getFirstRankNode(double a, double b, double c);
  void updateDistance(void);
  bool constructRouteInsert(double tmax, double a, double b, double c);
  void constructRoute(double tmax);
  void opt();
  
  //genetic algorithm solving op
  void genetic_init();
  void select(int tmax);

private:
  vector<node_t> nodes; 
  vector<node2> node;
  int start,end;   //id of start node and end node
  int T;
  bool first;
};

#endif

