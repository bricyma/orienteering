
/******************************************************************************
* File: Topology.h
* Description: Construct the topologies and manipulation methods.
* Author: Lantao Liu
* Date: 6/2010; Last update 2013
******************************************************************************/

#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "argparser.h"
#include "agent.h"
#include "util.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include "geometry_utils/Vector2.h"

typedef geometry_utils::Vector2d point2d_t;

typedef struct Node {
  uint id;
  point2d_t pos;
//  std::set<uint> neighbors;  
//  std::set<uint> op_neighbors; //store the possible nodes in the radius area of start node. different from neighbors
  vector<int> neighbors;
  //used to find the most efficient node in this area
  double score;
  double dc; //distance to center-of-gravity rank
  double df; //a sum of distances to the two foci of an ellipse
  bool routed;  //for traditional op algorithm
  bool visited; //for my op algorithm
  bool tsp_flag; //if tsp_flag=true, tsp tour includes this node.
  bool radius_flag; //if radius_flag=false, don't need to consider this node in current period.
  bool ok;
  uint parent;
  uint son;
} node_t;

typedef struct Node2 {
  uint id;
  double score;
  double eff;
  double dis;
  int num;
} node_i;

typedef struct Node3{
    int id;
    double ratio;
} node_o;


struct node2{
  int index;
  int id;
  int t;
  bool visited;
  int dist;
  int prev;
  double score;
  vector<int> neighbors;
};

struct real_vertex{
  int id;
  point2d_t pos;
  vector<int> neighbors;
  double score;
};

struct vertex{
  //T is tmax

  int id; //0,1,2,...,nT
  point2d_t pos;
  int index; //0,1,2,...,n
  double t;  //n*resolution
  int dist;
  vector<int> next; //next neighbor
  int parent; //parent id
  double score;  //score only itself
  double sum;   //the objective
  int degree; //inner degree
};


using namespace std;
//using namespace Eigen;

//extern uint RandSeed;
extern size_t Num_agents;
extern double Lambda;

  // import topo from outside
  vector<Agent> TopoImport(char* filename);


  vector<node_t> RandomTopo(uint _size,
                                 uint _bx = BOUND_X,
                                 uint _by = BOUND_Y,
                                 uint _seed = 1,
                                 double radius = 5.0);
  
  vector<real_vertex> RandomTopology(uint _size,
                                 uint _bx = BOUND_X,
                                 uint _by = BOUND_Y,
                                 uint _seed = 1,
                                 double radius = 5.0);
  // 3 values stores the number of states: body, starts, goals, in order
  vector<uint> NumStates(const vector<Agent>&);

  // perturb each agent with a small distance
  void PerturbTopo(vector<Agent>&, double, uint _seed = RandSeed);

  //add starts goal nodes manualy
  void
  GenerateStartGoalAgts(vector<Agent>& _agts);

  //agts in vec may be disordered between the vec indices and their IDs, the pointers here point the indices directly to the associated IDs
  void MapOrderedPtrs(vector<Agent>& _agtsall, vector<Agent*>& _ptrs);

  //split all into body, starts and goals
  void SplitByStates(const vector<Agent*>& _agtsall, vector<Agent*>& _body, vector<Agent*>& _starts, vector<Agent*>& _goals);
  
  // the neighbors among body agents, symmetric
  void GenerateNeighbors(vector<Agent*>&, double radius);
  // the neighbors between new insertion and the body, non-symmetric, replace by GenerateNeighborsAll
  void GenerateNeighbors2(vector<Agent*>& _news, vector<Agent*>& _body, double radius);
  // generate all neighbors inclduing the body and insertions
  void GenerateNeighborsAll(vector<Agent*>& _agtsall, double radius);
  void CleanNeighbors(vector<Agent*>&);

  vector<vector<double> >
  MorphingPairsInsert(vector<Agent*>& _agtsall, double lambda=Lambda, double radius=Radius);

  // _starts is the set of starting IDs (multi-path)
  vector<vector<AgtID> > 
  RetrievePath(const vector<Agent*>& _agtsall);  
  // add way points in each agt's to-go list, if _update_pose, the current pose is also updated to the waypoint pose (for next computation iteration)
  void AddWayPoints(vector<Agent*>& _agtsallptrs, const vector<vector<AgtID> >& _paths, bool _update_pose =false);

  // after each morphing iteration, upate poses (virtually) for next computation, the layer here denotes the index of the waypoints history
  //void UpdatePosesAll(const vector<vector<AgtID> >& _paths, vector<Agent*>& _agtsall, uint _layer=0);
  // after paths computation and before movement, resume to the start point
  void Resume2StartPoints(vector<Agent*>& _agtsall);

  // fill the states for agtsall via the given sets of starts and goals
  void UpdateMorphStates(vector<Agent*>& _agtsall, set<AgtID>& _s_set, set<AgtID>& _g_set);
  // update starts only, cause goals will update via UpdateGoal
  void UpdateStartStates(vector<Agent*>& _agtsall, set<AgtID>& _s_set);
  void RemoveFinishedGoals(vector<Agent*>& _agtsall, const vector<vector<AgtID> >& _paths);


  void WriteTopo(vector<Agent>&);
  void WriteMatrix(const vector<vector<double> >&);
  void WritePaths(vector<vector<AgtID> >&);

  void DisplayAgentInfo(Agent&);

#endif


