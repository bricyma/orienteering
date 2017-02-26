
///////////////////////////////////////////////////////////////////////////////
// File name: test.cpp
// Just for testing, the Hungarian algorithm is splitted into modules.
// Lantao Liu, Nov 1, 2009
///////////////////////////////////////////////////////////////////////////////

#include "Assignment.h"
#include "Hungarian.h"
#include "BipartiteGraph.h"
 
class Test{
public:
  Test(){}
  ~Test(){}

public:
  void test(void);
  void testHungarian(void);
 
};

void
Test::testHungarian(void){

  //test BipartiteGraph()
/*
input matrix is:
8 10 4 
0 12 6 
18 3 5 
*/
  cout<<"----------------------------------------"<<endl;
  cout<<"BipartiteGraph:"<<endl;
  
  ifstream myfile("example.test");
  Matrix m;
  Assignment a(0);
  m=a.ImportAssignment(myfile);
  a.DisplayMatrix(m);

  BipartiteGraph _bg(m.size(),m[0].size());
  _bg.ConstructBG(m); 
  _bg.DisplayLabels();
  _bg.CheckFeasibility();
  _bg.GetMatrix(0,2)->SetWeight(12);
  a.DisplayMatrix(*_bg.GetMatrix());
  _bg.CheckFeasibility();
  _bg.GetMatrix(0,2)->SetWeight(4);
  _bg.CheckFeasibility();
  
  cout<<endl<<"Checking DisplayLabels, CheckFeasibility pass."<<endl<<endl;


  //test Hungarian
  cout<<"----------------------------------------"<<endl;
  cout<<"Hungarian:"<<endl;
 
  Hungarian h(_bg);
  
  bool test_perfect;
  test_perfect = h.IsPerfect();
  cout<<"Perfect? "<<test_perfect<<endl;
  
  VID u;
  u = h.PickFreeAgent();
  h.InitNewRoot(u);
  cout<<"Free agent: "<<u<<endl;

  bool test_needLabel;
//  test_needLabel = h.NeedReLabel();
//  cout<<"Need re-label? "<<test_needLabel<<endl;

  h.RefreshBG();
  test_needLabel = h.NeedReLabel();  //before needRelabel, refresh first, so that EG is updated 
  cout<<"Need re-label? "<<test_needLabel<<endl;

  VID y;
  y = h.PickAvailableTask();
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;

  vector<EID> p = h.BFSAugmentingPath(u, y);

  h.AugmentPath(p);
  h.RefreshBG();  //after augment path, must refresh BG, since one new match in EG

  u = h.PickFreeAgent();
  h.InitNewRoot(u);
  test_needLabel = h.NeedReLabel();
  cout<<"Need re-label? "<<test_needLabel<<endl;
  
  y = h.PickAvailableTask();
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;
  h.ExtendTree(y);

  test_needLabel = h.NeedReLabel();
  cout<<"Need re-label? "<<test_needLabel<<endl;
 
  double delta = h.UpdateLabels();
  cout<<"delta: "<<delta<<endl;
  h.RefreshBG();  //after relabelling, must refresh BG

  test_needLabel = h.NeedReLabel();
  cout<<"Need re-label? "<<test_needLabel<<endl;

  y = h.PickAvailableTask();
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;
 
  p = h.BFSAugmentingPath(u, y);
  h.AugmentPath(p);
  h.RefreshBG();

  u = h.PickFreeAgent();
  h.InitNewRoot(u);
  test_needLabel = h.NeedReLabel();
  cout<<"Need re-label? "<<test_needLabel<<endl;

  y = h.PickAvailableTask();
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;
  h.ExtendTree(y);

  test_needLabel = h.NeedReLabel(); //need relabel updated N
  cout<<"Need re-label? "<<test_needLabel<<endl;

  y = h.PickAvailableTask();
  cout<<"Free task: "<<y<<endl;
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;
  h.ExtendTree(y);

  test_needLabel = h.NeedReLabel(); //need relabel updated N
  cout<<"Need re-label? "<<test_needLabel<<endl;

  delta = h.UpdateLabels();
  h.RefreshBG();

  test_needLabel = h.NeedReLabel();
  cout<<"Need re-label? "<<test_needLabel<<endl;
  y = h.PickAvailableTask();
  cout<<"y matched? "<<h.GetBG()->GetTask(y)->GetMatched()<<endl;
  
  p = h.BFSAugmentingPath(u, y);

  h.AugmentPath(p);
  h.RefreshBG();
  h.DisplayData(1,1,1,1,1);

  cout<<"Perfect? "<<h.IsPerfect()<<endl;

  //=======================================
  //Plot testing
  PlotGraph plot;
  //plot.PlotBipartiteGraph(_bg, h.S, h.T, h.N, h.EG, h.M);

}



void
Test::test(){

  cout<<endl<<"=====  Below is testing file  ==== "<<endl<<endl;
  
  //test Hungarian::PickAvailableTask @DONE
  vector<size_t> a, b;
  for(int i=0; i<5; i++) a.push_back(i);
  for(int j=7; j>0; j--) b.push_back(j);
  
  cout<<"----------------------------------------"<<endl;
  cout <<"Hungarian::PickAvailableTask"<<endl;
  Hungarian h;
  int y;
  y = h.PickAvailableTask(a, b);
  cout <<"y: "<<y<<endl;
  cout <<"Hungarian::PickAvailableTask @DONE"<<endl<<endl;
  
  //test Hungarian::FindNeighbors() 
  cout<<"----------------------------------------"<<endl;
  cout <<"Hungarian::FindNeighbors"<<endl;
  vector<EID> e;
  vector<VID> n;
  a.clear();
  a.push_back(0);
  a.push_back(1);
  a.push_back(2);
  e.push_back(EID(0,0));
  e.push_back(EID(1,0));
  e.push_back(EID(0,2));
  e.push_back(EID(1,2));
  e.push_back(EID(2,0));   

  n= h.FindNeighbors(e, a);
   
  for(vector<VID>::iterator itr = n.begin(); itr != n.end(); itr++)
    cout<<*itr<<" ";
  cout<<endl;
  cout <<"Hungarian::FindNeighbors @DONE"<<endl<<endl;

  //test Hungarian::NeedReLabel
  cout<<"----------------------------------------"<<endl;
  cout <<"Hungarian::NeedReLabel"<<endl;
  vector<VID> c;
  c.push_back(0);
  c.push_back(2);
  bool r;
  
  n.clear();
  r = h.NeedReLabel(c, n);
  cout<<"Result: "<<r<<endl;
  cout <<"Hungarian::NeedReLabel @DONE"<<endl<<endl;
  
}
