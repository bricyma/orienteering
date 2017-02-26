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
  Orienteering(){cur_t=0; v=1.0;}
  ~Orienteering(){}

  vector<node_t>& getNodes(void){ return nodes; }

  void createNodes(uint _size, uint _seed, double radius);
  void init_recent_time(void);
  void init(void);
  void setStartEnd(int _start, int _end){ start = _start; end = _end; }
  void setStart(int _start){ start = _start;}
  void setSpeed(double _speed){v = _speed; }
  double getDistance(int i, int j);
  double totalCost(void);
  double totalScore(void);
  point2d_t calculateGravity(void);
  void updateDistance(void);
  int calculateRank(int id, double a, double b, double c);
  bool insert( double tmax, double a, double b, double c);
  void constructRoute(double tmax, double a, double b, double c);
  void opt();
  void improveRoute(double tmax, double a, double b, double c);
//  bool myComparison(const pair<int,double> &a, const pair<int,double> &b);
//  vector<pair<int, double> >sortRatio();
  double calculateRatio(int id, point2d_t cog);  //given the center of gravity and id, output ratio 
  bool gravityInsert(double tmax, point2d_t cog);
  void gravityStep(double tmax, double a, double b, double c);
  void gravityStepIteration(double tmax, double a, double b, double c);
  vector<int> path(void);
  void updateRecentTime2();

//my old algorithm to solve OP, dijkstra based algorithm
  vector<vector<double> >createCostMatrix(vector<node_t> nodes);
  double score_log(double t, double T);
  double score_linear(double t, double T);
  double score_linear_time(double t, double T, double t0);
  double getRemainScore(double t, double T, int k);
  vector<double> getRecentTime(double t, int k); 
  vector<double> getCurScore(vector<node_t> nodes, double t, double T, int k);
  vector <double> getRoute(vector<int> order);
  vector<int> dijkstra(vector<node_t> nodes, double tmax, double radius);
  vector<int> dijkstra2(vector<node_t> nodes, double tmax, double radius);
  vector<int> op(vector<node_t> nodes, double tmax, double radius);
  vector<int> opFindBest(double tmax, double radius);
  vector<int> opIteration(double tmax, double radius);

  void calculateAverage(double tmax, int k); //calculate the remain average, remain/time 
  //int start, end;   //id of start node and end node
 // vector<vector<double> >createCostMatrix(vector<node_t> nodes);

//time dependent op problem
  double getDistanceById(int i, int j);
  static bool compare_score(node_t a, node_t b){return a.score < b.score;};
  static bool compare_eff(node_i a, node_i b){return a.eff > b.eff;};
  double pathCost(vector<int> tsp);
  vector<int> transform(vector<int> tsp_set, vector<int> order);
  void createTspMatrix(vector<int> node);
  vector<int> opInit(double T);
  vector<int> getTimeClock(vector<int> path);
  vector<int> opOperation(double T);
  int findIndex(int ID);
  node_t findNode(int ID);
  int findPos(vector<int> path, int num);
  
private:
  vector<node_t> nodes; 
  double v; //the speed of the vechile 
  int start,end;   //id of start node and end node
  double average;
  double cur_t;   //the current total time cost
  vector<vector<double> > update_time; //the most recent time that node is visited
  vector<vector<double> > cur_recent_time; 
  vector<vector<double> > recent_time;  //my op
  vector<vector<double> > recent_time2; //traditional op
};

#endif

