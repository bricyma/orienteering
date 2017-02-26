
#include "paths.h"


vector<Agent> AgtsAll;
vector<Agent*> AgtsAllPtrs; //ptr to strictly ordered agts, 0, 1, 2, ..n-1
vector<Agent*> AgtsSpecial; // selected set for scaling weights

void
agentImportInit(void){

  AgtsAll.clear();
  AgtsAll = TopoImport((char*)"configs/topo.txt");

  MapOrderedPtrs(AgtsAll, AgtsAllPtrs);

  //GenerateNeighborsAll(AgtsAllPtrs, Radius);
  GenerateNeighbors(AgtsAllPtrs, Radius);

  vector<vector<double> > m =MorphingPairsInsert(AgtsAllPtrs);

  //manually change matrix values here
  m[5][5] -= 15;
  //m[7][7] -= 6.2;
  //m[15][15] -= 17;
  //m[13][13] -= 15;
  m[11][11] -= 15;

  WriteMatrix(m);
  int state = system("assignment/hungarian -i data/adj_matrix_mod.txt -v 0");
  assert(state!=-1);

  vector<vector<AgtID> > Paths=RetrievePath(AgtsAllPtrs);
  AddWayPoints(AgtsAllPtrs, Paths);
 
}


void
agentImportInit2(void){

  AgtsAll.clear();
  AgtsAll = TopoImport((char*)"configs/topo.txt");

  MapOrderedPtrs(AgtsAll, AgtsAllPtrs);

  //GenerateNeighborsAll(AgtsAllPtrs, Radius);
  //vector<vector<double> > m =MorphingPairsInsert(AgtsAllPtrs);
  //MorphingPairsInsert(AgtsAllPtrs);

  // import from outside
  ifstream input((char*)"data/adj_matrix.txt");
  vector<vector<double> > m = ImportMatrix(input);
  // turn to utility maximization, and embed lambda
  vector<uint> num=NumStates(AgtsAll);
  for(uint i=0; i<m.size(); i++)
    for(uint j=0; j<m[i].size(); j++){
      //generate neighbors
      if(i!=j && m[i][j] < 1e+6 )
        AgtsAllPtrs[i]->neighbors.insert(AgtsAllPtrs[j]->id);
      //negate values
      m[i][j] = - m[i][j];
      //for body diagonal, also scale with lambda
      if(i==j && i<num[0]) 
        m[i][j] *= Lambda; 
    }

  WriteMatrix(m);
  int state = system("assignment/hungarian -i data/adj_matrix_mod.txt -v 0");
  assert(state!=-1);

  vector<vector<AgtID> > Paths=RetrievePath(AgtsAllPtrs);
  WritePaths(Paths);

  AddWayPoints(AgtsAllPtrs, Paths);
  
}


void 
agentRandomInit(void){

  //modified from function DemoSquareMorph();
  double _bx = 3*BOUND_X;
  //double _by = 3*BOUND_Y; 
  //double _bz = 3*BOUND_Z;

  //determine the slice distance, assume 12 slices
  double sd = 2*_bx/12; //should be consistent with "sd" in render.c
  uint agt_id = 0;
  Agent a;

  //the square shape for body 
  double zoffset= -(5.5*sd);
  int num_row=12;
  int num_col=12;
  for(int i=1; i<=num_row; i++){
    double xoffset= -(5.5*sd);
    for(int j=1; j<=num_col; j++){
      Pose p;
      p.y=0;
      p.z=zoffset;
      p.x=xoffset;
      a.id = agt_id++;
      a.state = BODY;
      a.pose = p;
      a.waypoints.clear();
      a.waypoints.push_back(a.pose);
      AgtsAll.push_back(a);
      xoffset += sd;
    }
    zoffset+=sd;
  }

  MapOrderedPtrs(AgtsAll, AgtsAllPtrs);
  PerturbTopo(AgtsAll, 0.02);

  const int num = 5;
  uint s_ary[num] = {12, 36, 60, 96, 120};
  //uint s_ary[num] = {60};
  set<AgtID> s_set(s_ary, s_ary+num);
  uint g_ary[num] = {23, 47, 71, 107, 131};
  //uint g_ary[num] = {71};
  set<AgtID> g_set(g_ary, g_ary+num);
  UpdateMorphStates(AgtsAllPtrs, s_set, g_set);

  // get the neighbors, will be used for plotting the connections
  GenerateNeighborsAll(AgtsAllPtrs, Radius);

  vector<vector<double> > m =MorphingPairsInsert(AgtsAllPtrs);

  // change matrix values here
  //double new_weight= -Weight; // this is a max problem
  double new_weight= 100; // this is a max problem
  //for(uint i=0; i<60; i++){
  for(uint i=72; i<96; i++){
   //uint idx= mapMatrixIndex2BodyAgts(AgtsAllPtrs, AgtsSpecial[i]);
   uint idx= mapMatrixIndex2BodyAgts(AgtsAllPtrs, AgtsAllPtrs[i]);
   assert(idx<m.size());
   m[idx][idx] = new_weight;
  }

  WriteMatrix(m);
  int state = system("assignment/hungarian -i data/adj_matrix_mod.txt -v 0");
  assert(state!=-1);

  vector<vector<uint> > Paths=RetrievePath(AgtsAllPtrs);
  //AddWayPoints(AgtsAllPtrs, Paths, true);
  AddWayPoints(AgtsAllPtrs, Paths);


  //compute the total length for all paths
  double lengths =0; 
  //compute path length
  for(uint i=0; i<Paths.size(); i++){
    for(uint j=0; j<Paths[i].size()-1; j++){
      Agent* a=AgtsAllPtrs[Paths[i][j]];
      Agent* b=AgtsAllPtrs[Paths[i][j+1]];
      lengths += Distance(a->pose.x, a->pose.y, a->pose.z, 
			b->pose.x, b->pose.y, b->pose.z);
    }
  }

  cout<<"@@ The total path length is:"<<lengths<<endl;
  cout<<"@@ The avg path length is:"<<lengths/Paths.size()<<endl<<endl;

  //paths lengths of only the last relaying robots (=the time to reach goal)
  double avg_relay_length = lengthLastRelay(Paths, AgtsSpecial)/Paths.size();
  cout<<"## The avg last travel segment length is:"<<avg_relay_length<<endl<<endl;
  cout<<"$$ The avg path length of those relayed paths is:"<<avgLengthWithRelay(Paths, AgtsSpecial)<<endl<<endl;

}


double lengthLastRelay(vector<vector<AgtID> >& _paths, vector<Agent*>& _agts_special){

  double length = 0;
  set<Agent*> loc_copy(_agts_special.begin(), _agts_special.end() );

  for(uint i=0; i<_paths.size(); i++){
    for(int j=_paths[i].size()-2; j>=0; j--){ // in reverse direction
      Agent* a=AgtsAllPtrs[_paths[i][j]];
      Agent* b=AgtsAllPtrs[_paths[i][j+1]];
      length += Distance(a->pose.x, a->pose.y, a->pose.z,
                        b->pose.x, b->pose.y, b->pose.z);

      if( loc_copy.find(AgtsAllPtrs[_paths[i][j]]) != loc_copy.end() )
        break;
    }
  }

  return length;
}

double avgLengthWithRelay(vector<vector<AgtID> >& _paths, vector<Agent*>& _agts_special){

  double length = 0;
  int count = 0;
  set<Agent*> loc_copy(_agts_special.begin(), _agts_special.end() );

  for(uint i=0; i<_paths.size(); i++){
    bool find=false;
    for(int j=_paths[i].size()-2; j>=0; j--){ // in reverse direction
      if( loc_copy.find(AgtsAllPtrs[_paths[i][j]]) != loc_copy.end() ){
        find = true;
        count ++;
        break;
      }
    }
   
   if(find){
    for(int j=_paths[i].size()-2; j>=0; j--){ // in reverse direction
      Agent* a=AgtsAllPtrs[_paths[i][j]];
      Agent* b=AgtsAllPtrs[_paths[i][j+1]];
      length += Distance(a->pose.x, a->pose.y, a->pose.z,
                        b->pose.x, b->pose.y, b->pose.z);

    }
   }//if
  }//for

  return length/count;
}


//
uint 
mapMatrixIndex2BodyAgts(vector<Agent*>& _agts, Agent* _a){

  int index =0;
  for(vector<Agent*>::const_iterator itr=_agts.begin(); itr!=_agts.end(); itr++){
    if((*itr)->id == _a->id) break;
    if((*itr)->state==BODY)
        index++;
  }
 
  return index; 
}



