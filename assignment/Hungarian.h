
/******************************************************************************
*                                                                             *
*  C++ Implementation of Hungarian Algorithm (Bipartite graph version).       *
*  Lantao Liu <lantao@cs.tamu.edu>                                            *
*  Nov 1, 2009                                                                *
*                                                                             *
******************************************************************************/

///////////////////////////////////////////////////////////////////////////////
// File name: Hungarian.h
// This file defines the main algorithm of Kuhn-Munkres Hungarian Method.
// The algorithm is decomposed into functional modules, and each module is
// encapsulated as a function here.
// Lantao Liu, Nov 1, 2009
///////////////////////////////////////////////////////////////////////////////

#ifndef Hungarian_H
#define Hungarian_H

#include "Assignment.h"
#include "BipartiteGraph.h"
#include "PlotGraph.h"

#define MAX_LOOP 30000 //debug


///////////////////////////////////////////////////////////////////////////////
//
// Hungarian class: defines basic modules (methods) for Hungarian procedure.
//
///////////////////////////////////////////////////////////////////////////////


class Hungarian{

public:
  Hungarian(){ S.reserve(AGENTS_SIZE); T.reserve(TASKS_SIZE); }
  Hungarian(BipartiteGraph& _bg):bg(_bg){
    S.reserve(_bg.GetNumAgents()); 
    T.reserve(_bg.GetNumTasks());
    N.reserve(_bg.GetNumTasks());
  }
  ~Hungarian(){}

  //To access private bipartite graph member
  inline BipartiteGraph* GetBG(void){ return &bg; }

  //Judge if the bipartite graph is perfect or not, to control program to halt
  inline bool IsPerfect(BipartiteGraph& _bg){ 
    bool result = _bg.GetNumMatched() == _bg.GetNumAgents() ? true: false;
    return result;
  }
  inline bool IsPerfect(void){ return this->IsPerfect(this->bg); } 
 
  //Start a new alternating tree, pass the root to set S
  //and initialize all other sets which need to initialized.
  void InitNewRoot(BipartiteGraph& _bg, VID root, vector<VID>& _S, vector<VID>& _T);
  void InitNewRoot(VID root);

  //Randomly pick an unmatched (free) agent, as new root
  VID PickFreeAgent(BipartiteGraph& _bg);
  VID PickFreeAgent(void);

  //Randomly pick a new task in set {N-T} during alternating tree
  VID PickAvailableTask(vector<VID>& _T, vector<VID>& _N);
  VID PickAvailableTask(void);

  //Update labels, forcing N != T, return minimal delta
  double UpdateLabels(BipartiteGraph& _bg);  
  double UpdateLabels(void);  
  
  //Refresh the whole bipartite graph, update all available sets
  //based on new state of bipartite graph _bg
  void RefreshBG(BipartiteGraph& _bg, 
		const vector<VID>& _S, const vector<VID>& _T, 
		vector<VID>& _N, vector<EID>& _EG, vector<EID>& _M);
  void RefreshBG(void);
  
  //Judge if need relabel or not, via comparing sets T and N
  bool NeedReLabel(vector<VID>& _T,  vector<VID>& _N);
  bool NeedReLabel(void);

  //Find neighbors and return a neighbor-set, computed from EG and S
  vector<VID> FindNeighbors(const vector<EID>& _EG, const vector<VID>& _S);
  vector<VID> FindNeighbors(void);

  //Breadth-first Searching augmenting path, return the path as a vector of EID
  //x: root
  //y: unmatched task
  vector<EID> BFSAugmentingPath(BipartiteGraph& _bg, VID x, VID y);
  vector<EID> BFSAugmentingPath(VID x, VID y);

  //Augment the path, via the information of augmenting path
  //flip the matched edges and unmatched edges, M size increases 1
  //_path: an augmenting path
  void AugmentPath(BipartiteGraph& _bg, vector<EID>& _path);
  void AugmentPath(vector<EID>& _path);

  //When found a matched task, put it and its mate (matching) agent
  //into T and S correspondingly, and color them to flag as having been 
  //visited and covered
  void ExtendTree(BipartiteGraph& _bg, VID x, vector<VID>& _S, vector<VID>& _T);
  void ExtendTree(VID x);

  void HungarianAlgo(BipartiteGraph& _bg, vector<VID>& _S, vector<VID>& _T,
			vector<VID>& _N, vector<EID>& _EG, vector<EID>& _M);
  void HungarianAlgo(void);
  void HungarianAlgo(vector<EID>& out); //call and fill output

  //Calculate optimal value: the sum of weights of matched edges
  double OptimalValue(BipartiteGraph& _bg, vector<EID>& _M);
  double OptimalValue(void);

  //Display data
  void DisplayData(vector<EID>&);
  void DisplayData(vector<VID>&);
  //The five args are in order of S, T, N, EG, M. Set true if want to display
  void DisplayData(bool s=false, bool t=false, bool n=false, 
	bool eg=false, bool m=false);

  void DisplayLabels(vector<Vertex>&);

  void DisplayMatrix(Matrix&);
  void DisplayConfig(BipartiteGraph&);
  void WriteConfig(BipartiteGraph&);
  void WriteConfig2(const vector<EID>&);
  void RetrievePath(void);

//data member
//public:
private:
  //A basic bipartite graph is needed
  BipartiteGraph bg;
  
  //All sets required in Hungarian algo
  vector<VID> S;
  vector<VID> T;
  vector<VID> N;
  vector<EID> EG;
  vector<EID> M; 
 
};


#endif
