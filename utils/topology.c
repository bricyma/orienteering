
#include "topology.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iterator>
//#include <Eigen/Dense>

vector<Agent> TopoImport(char* filename){

  vector<Agent> as;
 
   char str[256];
   FILE *pFile;
   char* c;
   float x, y, z;

   int ID = 0;

   pFile = fopen (filename,"rt");

   Agent a;
   while(1){
     do{
       // read the line
       c=fgets(str, 256, pFile);
     }while( ((str[0] == '/') || (str[0] == '\n'))&& (c!=NULL));
   
     int match_cnt = 0;
     if(c!=NULL){
       // for BODY node
       match_cnt = sscanf(str, "BODY %f %f %f", &x, &y, &z);
       if(match_cnt > 0){
       //printf("Body %f, %f, %f\n", x, y, z);
         a.id = ID++;
         a.pose.x = x;
         a.pose.y = y;
         a.pose.z = z;
         a.state = BODY;
         as.push_back(a);
       }
       match_cnt =0;

       // for START node
       match_cnt = sscanf(str, "START %f %f %f", &x, &y, &z);
       if(match_cnt > 0){
       //printf("Body %f, %f, %f\n", x, y, z);
         a.id = ID++;
         a.pose.x = x;
         a.pose.y = y;
         a.pose.z = z;
         a.state = START;
         as.push_back(a);
       }
       match_cnt =0;

       // for BODY node
       match_cnt = sscanf(str, "GOAL %f %f %f", &x, &y, &z);
       if(match_cnt > 0){
       //printf("Body %f, %f, %f\n", x, y, z);
         a.id = ID++;
         a.pose.x = x;
         a.pose.y = y;
         a.pose.z = z;
         a.state = GOAL;
         as.push_back(a);
       }

     }
     else
     { printf("Finished reading files!\n");
       break;
     } 
   }

   fclose (pFile);

   return as;
}


vector<node_t>
RandomTopo(uint _size, uint _bx, uint _by, uint _seed, double radius){

  vector<node_t> vec;
  srand(_seed);

  node_t n;
  for(uint i=0; i<_size; i++){
    n.id = i;
    int rdm1 = rand()%100, rdm2 = rand()%100;
    int flag1,flag2;
    flag1 = rdm1 >= 50 ? 1:-1;
    flag2 = rdm2 >= 50 ? 1:-1;
    n.pos.x() = flag1 * int (rand()%(_bx*100)) /100;
    n.pos.y() = flag2 * int (rand()%(_by*100)) /100;
    
    n.score = rand()%100;
    

    n.routed = false; //the node is not in the route yet.
    n.visited = false;
    vec.push_back(n);
  }

  
/*  ifstream file2("./utils/problem3.txt");
  if (!file2) {
    cout << "Cannot open file.\n";
  }
  for (int i=0; i<33; i++){
    double x,y;
    int s;
    file2>>x>>y>>s;
    n.id=i;
    n.pos.x()=x;
    n.pos.y()=y;
    n.score=s;
    n.routed = false;
    n.visited = false;
    //cout<<x<<" "<<y<<" "<<s<<endl;
    vec.push_back(n);
  }
  file2.close();*/

  //create neighboring information

/*  for(uint i=0; i<_size; i++)
    for(uint j=0; j<_size; j++){
      double d = Distance(vec[i].pos.x(), vec[i].pos.y(), 0,
                vec[j].pos.x(), vec[j].pos.y(), 0);
      if(d<=radius && d>EPSILON){
        vec[i].neighbors.insert(vec[j].id);
      }
    }
*/

    for (uint i=0; i<_size; i++){
      for (uint j=0; j<_size; j++){
        double d = Distance(vec[i].pos.x(), vec[i].pos.y(), 0,
                            vec[j].pos.x(), vec[j].pos.y(), 0);
        if (d<=radius && d>EPSILON){
          vec[i].neighbors.push_back(vec[j].id);
        }
      }
    }

/*
  for(uint i=0; i<_size; i++)
    cout<<as[i].pos.x<<"\t"<<as[i].pos.z<<endl;
*/
  return vec;

}


vector<real_vertex>
RandomTopology(uint _size, uint _bx, uint _by, uint _seed, double radius){

  vector<real_vertex> vec;
  srand(_seed);

  real_vertex n;
  //version 1.0
  for(uint i=0; i<_size; i++){
    n.id = i;
    int rdm1 = rand()%100, rdm2 = rand()%100;
    int flag1,flag2;
    flag1 = rdm1 >= 50 ? 1:-1;
    flag2 = rdm2 >= 50 ? 1:-1;
    n.pos.x() = flag1 * double (rand()%(_bx*100)) /100;
    n.pos.y() = flag2 * double (rand()%(_by*100)) /100;
    n.score = rand()%100; 
    //if (n.pos.x()>0 && n.pos.y()<0)
     // n.score = 100;
    vec.push_back(n);
  }


  //version 2.0 for time-dependent step-funtion case
  /*for(uint i=0; i<_size; i++){
    n.id = i;
    int rdm1 = rand()%100, rdm2 = rand()%100;
    int flag1,flag2;
    if (i<_size/2){
      flag1 = rdm1 >= 50 ? 1:-1;
      flag2 = rdm2 >= 50 ? 1:-1;
      n.pos.x() = flag1 * int (rand()%(_bx*100)) /100;
      n.pos.y() = flag2 * int (rand()%(_by*100)) /100;
      n.score = rand()%100;
    }else{
      n.pos.x() = vec[i-_size/2].pos.x();
      n.pos.y() = -vec[i-_size/2].pos.y();
      n.score = vec[i-_size/2].score;
    }

    vec.push_back(n);
  }*/

  for (uint i=0; i<_size; i++){
    for (uint j=0; j<_size; j++){
        double d = Distance(vec[i].pos.x(), vec[i].pos.y(), 0,
                            vec[j].pos.x(), vec[j].pos.y(), 0);
        if (d<=radius && d>EPSILON){
          vec[i].neighbors.push_back(vec[j].id);
        }
    }
  }


  return vec;

}


vector<uint> 
NumStates(const vector<Agent>& _agts){

 int x=0; int y=0; int z=0;
 for(uint i=0; i<_agts.size(); i++){
   if(_agts[i].state == BODY)
     x ++;
   else if(_agts[i].state == START)
     y ++;
   else if(_agts[i].state == GOAL)
     z ++;
   else
     assert(0);
 }
 vector<uint> v;
 v.resize(3);
 v[0]=x; v[1]=y; v[2]= z;
 
 return v;

}

void
UpdateMorphStates(vector<Agent*>& _agtsall, set<AgtID>& _s_set, set<AgtID>& _g_set){

  //split the copy into agts and starts
  for(uint i=0; i<_agtsall.size(); i++){
    if(_s_set.find(_agtsall[i]->id) != _s_set.end()){
      _agtsall[i]->state=START;
      //_starts.push_back(_agtsall[i]); 
    }
    else if(_g_set.find(_agtsall[i]->id) != _g_set.end()){
      _agtsall[i]->state=GOAL;
      //_goals.push_back(_agtsall[i]);
    }
    else{
      _agtsall[i]->state=BODY;
      //_body.push_back(_agtsall[i]);
    }
  }
  //assert(_body.size()+_starts.size()+_goals.size()==_agtsall.size());

}


void 
MapOrderedPtrs(vector<Agent>& _agtsall, vector<Agent*>& _ptrs){

  _ptrs.clear();
  _ptrs.resize(_agtsall.size());
  vector<Agent>::iterator itr;
  Agent* pa=NULL;
  for(uint idx=0; idx<_ptrs.size(); idx++){
    for(itr=_agtsall.begin(); itr!=_agtsall.end(); itr++){
      if(itr->id == idx){
        pa = &(*itr); 
        _ptrs[idx]=pa;
	break;
      }   
    }
    assert(itr!=_agtsall.end());
  }

}


void 
PerturbTopo(vector<Agent>& _as, double _r_perturb, uint _seed){
  
  srand48(_seed);

  double perturb_angle;
  for(uint i=0; i<_as.size(); i++){
    perturb_angle = 2*PI*drand48(); 
    _as[i].pose.x += _r_perturb*cos(perturb_angle);
    _as[i].pose.z += _r_perturb*sin(perturb_angle);
    //_as[i].pose.x += (_r_perturb*drand48())*cos(perturb_angle);
    //_as[i].pose.z += (_r_perturb*drand48())*sin(perturb_angle);
    if(!_as[i].waypoints.empty()){
      _as[i].waypoints[0]=_as[i].pose;
    }
  }

}



/////////////////////////////////////////////////////////////////


void 
SplitByStates(const vector<Agent*>& _agtsall, vector<Agent*>& _body, vector<Agent*>& _starts, vector<Agent*>& _goals){
  
  _body.clear(); 
  _starts.clear();
  _goals.clear();
  for(vector<Agent*>::const_iterator itr=_agtsall.begin(); itr!=_agtsall.end(); itr++){
    if((*itr)->state==BODY)
	_body.push_back(*itr);
    else if((*itr)->state==START)
	_starts.push_back(*itr);
    else if((*itr)->state==GOAL)
	_goals.push_back(*itr);
  }

}


void
GenerateNeighbors(vector<Agent*>& _agts, double radius){

  uint num= _agts.size();
  for(uint i=0; i<num; i++)
    for(uint j=i+1; j<num; j++){
      double d = Distance(_agts[i]->pose.x, _agts[i]->pose.y, _agts[i]->pose.z,
                _agts[j]->pose.x, _agts[j]->pose.y, _agts[j]->pose.z);
      if(d<=radius && i!=j){
        //_agts[i].neighbors.insert(j); // equivalent
        _agts[i]->neighbors.insert(_agts[j]->id);
        //_agts[j].neighbors.insert(i);
        _agts[j]->neighbors.insert(_agts[i]->id);
      }
    }

}


void
GenerateNeighbors2(vector<Agent*>& _news, vector<Agent*>& _body, double radius){

  for(uint i=0; i<_news.size(); i++)
    for(uint j=0; j<_body.size(); j++){
      double d = Distance(_news[i]->pose.x, _news[i]->pose.y, _news[i]->pose.z,
                _body[j]->pose.x, _body[j]->pose.y, _body[j]->pose.z);
      if(d<=radius && d>EPSILON){
        _news[i]->neighbors.insert(_body[j]->id);
      }
    }
}

void
GenerateNeighborsAll(vector<Agent*>& _agtsall, double radius){

  vector<Agent*> agts_p, starts_p, goals_p;
  SplitByStates(_agtsall, agts_p, starts_p, goals_p);
  //for body
  GenerateNeighbors(agts_p, radius);

/*
  vector<Agent*> all_p = agts_p;
  all_p.insert(all_p.end(), starts_p.begin(), starts_p.end());
  all_p.insert(all_p.end(), goals_p.begin(), goals_p.end());
*/
  // comment below to disable insertion
  //for starts
  GenerateNeighbors2(starts_p, _agtsall, radius);
  //for goals
  GenerateNeighbors2(goals_p, _agtsall, radius);

}

void 
CleanNeighbors(vector<Agent*>& _agts){

  for(uint i=0; i<_agts.size(); i++){
    _agts[i]->neighbors.clear();
  }

}



// 2d vectors version
vector<vector<double> >
MorphingPairsInsert(vector<Agent*>& _agtsall, double lambda, double radius){

  assert(_agtsall.size());

  vector<Agent*> _starts, _goals, _body;
  SplitByStates(_agtsall, _body, _starts, _goals); 

  //also assume the starts and goals are equal sized, NO
  //assert(_starts.size() == _goals.size());

  vector<vector<double> > m;
  uint row_size=_body.size()+_starts.size();
  uint col_size=_body.size()+_goals.size();
  m.resize(row_size);

  //first generate the symmetric matrix of stationary nodes
  for(uint i=0; i<_body.size(); i++){
    m[i].resize(col_size, -(int)INF); //init with -oo
    for(uint j=0; j<_body.size(); j++){
      if(_body[i]->neighbors.find(_body[j]->id) != _body[i]->neighbors.end() &&
		_body[i]->id != _body[j]->id)
        m[i][j] = - Distance(_body[i]->pose.x, _body[i]->pose.y, _body[i]->pose.z,
                _body[j]->pose.x, _body[j]->pose.y, _body[j]->pose.z);
      else
	m[i][j] = -(int)INF;
    }
  }

  //double check whether symmetric
  for(uint i=0; i<_body.size(); i++)
    for(uint j=0; j<i; j++){
      //assert(fabs(m[i][j] - m[j][i]) <EPSILON);
      if(fabs(m[i][j] - m[j][i]) >=EPSILON)
        cout<<"m["<<i<<"]["<<j<<"]: "<<m[i][j]<<", m["<<j<<"]["<<i<<"]: "<<m[j][i]<<endl;
    }

  //get the largest for each row, and put in diagonal
  for(uint i=0; i<_body.size(); i++){
    double max_elt = *max_element(m[i].begin(), m[i].end());
    m[i][i] = lambda*max_elt;
  }
  
  //now add the starts and goals
  //first add the new rows
  for(uint i=0; i< _starts.size(); i++){
    m[_body.size()+i].resize(col_size);
    for(uint j=0; j<_body.size(); j++){
      if(_starts[i]->neighbors.find(_body[j]->id) != _starts[i]->neighbors.end())
        m[_body.size()+i][j] = - Distance(_starts[i]->pose.x, _starts[i]->pose.y, _starts[i]->pose.z, _body[j]->pose.x, _body[j]->pose.y, _body[j]->pose.z);
      else
	m[_body.size()+i][j] = -(int)INF;
    }
  }

  //then add the new cols
  // combine the agents together
  vector<Agent*> all = _body;
  all.insert(all.end(), _starts.begin(), _starts.end());
  assert(all.size()==row_size);

  for(uint i=0; i< _goals.size(); i++)
    for(uint j=0; j<all.size(); j++){
      if(_goals[i]->neighbors.find(all[j]->id) != _goals[i]->neighbors.end()){
        m[j][_body.size()+i] = - Distance(_goals[i]->pose.x, _goals[i]->pose.y, _goals[i]->pose.z, all[j]->pose.x, all[j]->pose.y, all[j]->pose.z);
      }
      else{
	m[j][_body.size()+i] = -(int)INF;
      }
    }

  return m;

}


void
WriteTopo(vector<Agent>& as){
  ofstream output;
  //if(system("mkdir data"));
  //output.open("data/topo.txt", ios::app);
  output.open("data/topo.txt");
  if(!output)
    cerr << "Cannot open \"topo.txt\"!"<<endl;

  output<<"% This is the topology of mult-robot system"<<endl
	<<"% Each row consists a.pose.tion of x, y, z."<<endl;

  for(vector<Agent>::iterator itr= as.begin(); itr!=as.end(); itr++){
    output<<" "<<(*itr).pose.x<<"\t"<<(*itr).pose.y<<"\t"<<(*itr).pose.z<<endl;
  }

  output.close();

} 

void
WriteMatrix(const vector<vector<double> >& _m){
  ofstream output;
  //if(system("mkdir data"));
  //output.open("data/topo.txt", ios::app);
  output.open("data/adj_matrix_mod.txt");
  if(!output)
    cerr << "Cannot open \"adj_matrix_mod.txt\"!"<<endl;

  output<<"# This is the adjusted adj_matrix"<<endl;

  output.precision(10);
  for(uint i=0; i<_m.size(); i++){
    for(uint j=0; j<_m[i].size(); j++)
      output<<_m[i][j]<<" ";
    output<<endl;
  }

  output.close();

} 


vector<vector<AgtID> >
RetrievePath(const vector<Agent*>& _agtsall){

  //read data from file
  ifstream input_file("data/assignment_vector.txt");
  assert(input_file.is_open());
  string line;
  getline (input_file,line);

  vector<uint> data;
  stringstream parse(line);
  string word;
  while(parse >> word)
    data.push_back(atoi(word.c_str()));

  cout<<"Assignment vector:"<<endl;
  copy(data.begin(), data.end(), ostream_iterator<uint>(cout, " "));
  cout<<endl<<endl;

  vector<Agent*> _starts, _goals, _body;
  SplitByStates(_agtsall, _body, _starts, _goals);

  vector<vector<AgtID> >paths(_starts.size());
  //a projection between the matrix index and real agt IDs
  vector<AgtID> pj;
  for(uint i=0; i<_body.size(); i++)
    pj.push_back(_body[i]->id);
  for(uint i=0; i<_starts.size(); i++)
    pj.push_back(_starts[i]->id);
  assert(pj.size()==data.size());

  //retrieve the paths
  for(uint i=0; i<_starts.size(); i++){
    // the row index from the matrix, i.e., the starting idx of insertion agts
    uint start_m_id= _body.size()+i;
    uint idx = start_m_id;
    paths[i].push_back(pj[idx]);
    uint next;
    uint cnt = 0;
    while(1){
      next = data[idx];  
      paths[i].push_back(pj[next]);
      idx = next;
      if(++cnt>=data.size()){ 
	cerr<<"Graph is not connected, can not find a path!"<<endl; exit(0);}
      bool found = false;
      if(next>=_body.size()){
	  found = true;
	  break;
      }
      if(found) break; //break the while
    }; //while

    //project to real IDs for goals (goals are also regarded as agts)
    //"next" records the last of the path
    AgtID g = _goals[next-_body.size()]->id;
    paths[i].back()=g;
    
    cout<<"Path["<<i<<"] starting from "<<_starts[i]->id<<":"<<endl;
    copy(paths[i].begin(), paths[i].end(), ostream_iterator<uint>(cout, " "));
    cout<<endl;
  }//for

  return paths;
}


void WritePaths(vector<vector<AgtID> >& _paths){

  ofstream output;
  output.open("data/paths.txt");
  if(!output)
    cerr << "Cannot open \"paths.txt\"!"<<endl;

  for(uint i=0; i<_paths.size(); i++){
    output<<"%Path["<<i<<"] starting from "<<_paths[i][0]<<":"<<endl;
    for(vector<AgtID>::iterator itr= _paths[i].begin(); itr!=_paths[i].end(); itr++){
      output<<" "<<*itr;
    }
    output<<endl;
  }

  output.close();

}

void 
AddWayPoints(vector<Agent*>& _agtsallptrs, const vector<vector<AgtID> >& _paths, bool _update_pose){

  for(uint i=0;i<_paths.size(); i++){
    for(uint j=0; j<_paths[i].size()-1; j++){
      _agtsallptrs[_paths[i][j] ]->waypoints.push_back(_agtsallptrs[_paths[i][j+1] ]->pose);
      if(_update_pose)
        _agtsallptrs[_paths[i][j] ]->pose = _agtsallptrs[_paths[i][j+1] ]->pose;
    }
  }

}


void
Resume2StartPoints(vector<Agent*>& _agtsall){

  for(uint i=0;i<_agtsall.size(); i++){
    _agtsall[i]->pose = _agtsall[i]->waypoints.front(); 
  }

}


void
UpdateStartStates(vector<Agent*>& _agtsall, set<AgtID>& _s_set){

  //split the copy into agts and starts
  for(uint i=0; i<_agtsall.size(); i++){
    if(_s_set.find(_agtsall[i]->id) != _s_set.end()){
      _agtsall[i]->state=START;
    }
    else if(_agtsall[i]->state==START){
      //_agtsall[i]->state = (_agtsall[i]->state==GOAL) ? GOAL : BODY;
      _agtsall[i]->state = BODY;
    }
    
  }
}


void 
RemoveFinishedGoals(vector<Agent*>& _agtsall, const vector<vector<AgtID> >& _paths){

  for(uint i=0;i<_paths.size(); i++){
    int last = _paths[i].size()-1;
    //_agtsall[_paths[i][last]]->state = _agtsall[_paths[i][last-1]]->state;
    _agtsall[_paths[i][last]]->state = REMOVED;
  }

}


void 
DisplayAgentInfo(Agent& a){

  cout<<"Agent "<<a.id<<":"<<endl;
  cout<<"\tPose: ("<<a.pose.x<<","<<a.pose.y<<","<<a.pose.z<<")"<<endl;
  cout<<"\tState: ";
  if(a.state==START) cout<<"start"<<endl;
  if(a.state==GOAL) cout<<"goal"<<endl;
  if(a.state==BODY) cout<<"body"<<endl;
  if(a.state==REMOVED) cout<<"removed!"<<endl;
  //other info..

}


