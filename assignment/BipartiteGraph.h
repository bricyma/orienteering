
///////////////////////////////////////////////////////////////////////////////
// File name: BipartiteGraph.h
// This file defines a complete bipartite graph, which includes all the basic
// properties (data members) and basic operations.
// Lantao Liu, Nov 1, 2009
///////////////////////////////////////////////////////////////////////////////


#ifndef BIPARTITE_GRAPH_H
#define BIPARTITE_GRAPH_H

#include "Matrix.h"
#include "Assignment.h"


///////////////////////////////////////////////////////////////////////////////
//
//  BipartiteGraph class: represent a complete bipartite graph
//			  it uses matrix to contain edges, and also two vectors
//			  of Vertices to denote Agents and Tasks
//
///////////////////////////////////////////////////////////////////////////////


class BipartiteGraph{
public:
  //BipartiteGraph(){}
  BipartiteGraph(size_t _num_agents = AGENTS_SIZE, size_t _num_tasks = TASKS_SIZE):
	num_matched(0), num_agents(_num_agents),num_tasks(_num_tasks){}

  BipartiteGraph(Matrix& m){ this->ConstructBG(m); }

  ~BipartiteGraph(){}

  //basic data operations
  inline void SetNumMatched(size_t _num_matched){ num_matched = _num_matched; }
  inline size_t GetNumMatched(void){ return num_matched; }
  inline size_t GetNumAgents(void){ return num_agents; }
  inline size_t GetNumTasks(void){ return num_tasks; }
  inline double GetMinDelta(void){ return min_delta; }
  inline void SetMinDelta(double _min_delta){ min_delta = _min_delta; }

  inline Vertex* GetAgent(VID _vid){ return &agents[_vid]; }
  inline Vertex* GetTask(VID _vid){ return &tasks[_vid]; }
 
  inline vector<Vertex>* GetAgents(void){ return &agents; }
  inline vector<Vertex>* GetTasks(void){ return &tasks; }

  inline Matrix* GetMatrix(void){ return &bg_matrix; }
  inline Edge* GetMatrix(EID& _eid){return &bg_matrix[_eid.first][_eid.second];}
  inline Edge* GetMatrix(size_t row_index, size_t col_index){ 
		return &bg_matrix[row_index][col_index]; }

  void DisplayLabels(void);
  void DisplayLabels(vector<Vertex>& v);

  //Get the maximal weighted edge among all outgoing edges from vertex "agent" 
  Edge GetMaxWeightEdge(Vertex& agent);

  //Construct a bipartite graph based on a specific matrix
  //All argments are initialized in it
  void ConstructBG(Matrix&);
 
  //Check if a bipartite graph is feasible or not
  bool CheckFeasibility(Matrix&, vector<Vertex>&, vector<Vertex>&);
  bool CheckFeasibility(void);
 
private:
  //basic data members
  size_t num_matched;
  size_t num_agents;
  size_t num_tasks;
  double min_delta;
 
  //data members storing vertices, and matrix that contains edges/utils
  vector<Vertex> agents;
  vector<Vertex> tasks; 
  Matrix bg_matrix;

};

#endif
