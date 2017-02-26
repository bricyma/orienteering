#include "op.h"
#define MAXINF 100000
#define MININF 0.00001

//int start=1, end=8; 

/*
void 
Orienteering::importNodes(){
  //include x,y,score
}*/


void
Orienteering::createNodes(uint _size, uint _seed, double radius)
{

  nodes=RandomTopo(_size, BOUND_X, BOUND_Y, _seed, radius);    

}

void 
Orienteering::init_recent_time(void){
  vector<double> vec(nodes.size(),0);
  recent_time.resize(nodes.size());
  cur_recent_time.resize(nodes.size());
  for (int i=0; i<nodes.size(); i++){
    recent_time[i].push_back(0);
    update_time.push_back(vec);
    cur_recent_time[i].resize(nodes.size());
     
  }
}

void
Orienteering::init(void){
  //start and end node are routed at first
//  vector<double> vec(nodes.size(),0);
//  recent_time.resize(nodes.size());
//  cur_recent_time.resize(nodes.size());
  for (int i=0; i<nodes.size(); i++){
    nodes[i].routed = false;  //for traditional op
    nodes[i].visited = false; //for my op 
    nodes[i].parent = start; //initilize each node's parent to be start
  //  recent_time[i].push_back(0);
  //  update_time.push_back(vec);
  //  cur_recent_time[i].resize(nodes.size());
  }
  nodes[start].routed = true;
  nodes[end].routed = true;  //for my op algorithm

  //initilize parent of end node
  nodes[end].parent = start;

  //set start nodes's score to be 0
  nodes[start].score = MAXINF;
  
  //initilize df, a sum of distances to the two foci(start, end) of an ellipe rank
  for (int i=0; i<nodes.size(); i++){
    nodes[i].df = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[start].pos.x(), nodes[start].pos.y(), 0) +
                  Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[end].pos.x(), nodes[end].pos.y(), 0);
                  
  }

}

//input index, output distance between two nodes
double
Orienteering::getDistance(int i, int j){
  return Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0);
}


//calculate total time cost(distance)
double
Orienteering::totalCost(void){
  int i=end;
  double s=0;
  while (i!=start){
    int j=nodes[i].parent;
    s+= Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                nodes[j].pos.x(), nodes[j].pos.y(), 0);
    i = j;
  }
  return s;
}

double 
Orienteering::totalScore(void){
  double s=0;
  for (int i=0; i<nodes.size(); i++){
    if (nodes[i].routed) s+=nodes[i].score;
  }
  return s;
}

point2d_t
Orienteering::calculateGravity(void){
  double coordx=0, coordy=0;
  double s=0;  
  point2d_t pos;
  for (int i=0; i<nodes.size(); i++){
    if (nodes[i].routed){
      coordx += nodes[i].pos.x() * nodes[i].score;
      coordy += nodes[i].pos.y() * nodes[i].score;
      s += nodes[i].score;
    }
  }
  pos.set(coordx/s, coordy/s);
  return pos;
}

//update dc
void
Orienteering::updateDistance(void){
  point2d_t cog = calculateGravity();
  for (int i=0; i < nodes.size(); i++){
    //update those haven't been routed
    if (!nodes[i].routed){
      nodes[i].dc = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                    cog.x(), cog.y(), 0);
                  
    }
  }

}

//calculate the rank of those unvisited nodes
int 
Orienteering::calculateRank(int id,  double a, double b, double c){
  //score rank, center-of-gravity rank, a sum of distances to the two foci of an ellipse rank
  int sr=0, cr=0, er=0; 
  for (int i=0; i<nodes.size(); i++){
    if (!nodes[i].routed){
      if (nodes[i].score > nodes[id].score) sr++;
      if (nodes[i].dc < nodes[id].dc) cr++;
      if (nodes[i].df < nodes[id].df) er++;
    }
  }
  return (a*sr + b*cr + c*er);
}


//insert one node in the current path in Route Construction Step
//rule: wr=a*sr+b*cr+y*er
bool
Orienteering::insert(double tmax, double a, double b, double c){
  int min_id=0, min = MAXINF;  
  //find the rank 1 node, min_id is its id
  for (int i=0; i<nodes.size(); i++){
    if (!nodes[i].routed){
      if (calculateRank(i, a, b, c) < min){
        min = calculateRank(i, a, b, c);
        min_id = i;
      }
    }
  }

  //cheapest insertion
  //find the cheapest insertion place, min_cut_id is its one side.
  //nodes[min_cut_id].parent is its other side.
  double min_cut_id, mincost = MAXINF;
  double cur = 0;
  int j;  // j is the id of nodes[i]'s parent
  for (int i=0; i<nodes.size()-1; i++){
    if (nodes[i].routed && i != start){
      j = nodes[i].parent;
      cur = Distance(nodes[min_id].pos.x(), nodes[min_id].pos.y(), 0,
                     nodes[i].pos.x(), nodes[i].pos.y(), 0) +
            Distance(nodes[min_id].pos.x(), nodes[min_id].pos.y(), 0,
                     nodes[j].pos.x(), nodes[j].pos.y(), 0) -
            Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                     nodes[j].pos.x(), nodes[j].pos.y(), 0);
      //the new insert node must be in the neighbor of i and j
      if (nodes[min_id].neighbors.find(i) !=nodes[min_id].neighbors.end() 
          && nodes[min_id].neighbors.find(j) !=nodes[min_id].neighbors.end()
          && cur < mincost){
        mincost = cur;
        min_cut_id = i;
      }
    }
  }
  if (totalCost() + mincost < tmax){
    j = nodes[min_cut_id].parent;
    nodes[min_cut_id].parent = min_id;
    nodes[min_id].parent = j;
    nodes[min_id].routed = true;
    return true;
  }  
  return false;
}


void
Orienteering::constructRoute(double tmax, double a, double b, double c){

  //cout<<"constructRoute: "<<endl;
  while (1){
    if (insert(tmax, a, b, c)) updateDistance();
    else break;
  }
  cout<<"score: "<<totalScore()<<endl;
  cout<<"cost: "<<totalCost()<<endl;
}

//2opt to improve the current path
void
Orienteering::opt(){
  bool changed=true;
  vector<int> order = path();
  while(changed){
    int ii=0,jj=0;
    double min=0;
    changed=false;
    //different from tsp, Orienteering problem has static start and end point
    for (int i=0; i<order.size()-2; i++){
       for (int j=i+2; j<order.size()-1; j++){
        int i2=(i+1),j2=(j+1);
        //new - old
        //the new edge must exist before opt
        if (nodes[order[i]].neighbors.find(order[j]) != nodes[order[i]].neighbors.end() &&
            nodes[order[i2]].neighbors.find(order[j2]) != nodes[order[i2]].neighbors.end()){
        //  cout<<getDistance(order[i],order[j]) + getDistance(order[i2],order[j2]) - 
        //      getDistance(order[i],order[i2]) - getDistance(order[j],order[j2])<<endl;
          if (getDistance(order[i],order[j]) + getDistance(order[i2],order[j2]) - 
              getDistance(order[i],order[i2]) - getDistance(order[j],order[j2]) < min){
            
            min = getDistance(order[i],order[j]) + getDistance(order[i2],order[j2]) - 
                  getDistance(order[i],order[i2]) - getDistance(order[j],order[j2]);
            ii = i;
            jj = j;
            changed = true;
   
          }  
        }
        
      }
    }
    if (changed){     
      reverse(order.begin()+(ii+1),order.begin()+(jj+1)); 
    }
  }
  //update parent information
  for (int i=order.size()-1; i>0; i--){
    nodes[order[i]].parent = order[i-1];
  }


}


//one method to improve route, the rule is based on a*sr+b*cr+y*er;
void
Orienteering::improveRoute(double tmax, double a, double b, double c){
  //cout<<"improveRoute: "<<endl;
  opt();
  while (1){
    if (insert(tmax, a, b, c)) updateDistance();
    else break;
  }
 

}

//another method to improve route, the rule is based on gravity center
/*void 
Orienteering::improveRoute2(doulbe tmax){
  opt();
  while(gravityInsert(tmax, cog));
}*/

//used to sort the ratio in decreasing order, 5,4,3,2,1
/*bool 
Orienteering::myComparison(const pair<int,double> &a, const pair<int,double> &b){
    return a.second > b.second;
}*/

//sort ratio = score(i)/t(i,g), t(i,g) is the cost from center of gravity to node i.
/*vector<pair<int, double> >
Orienteering::sortRatio(){
  point2d_t cog = calculateGravity();
  vector<pair<int, double> > vec;
  for (int i=0; i<nodes.size(); i++){
    double dis = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                          cog.x(), cog.y(), 0);
    vec.push_back(make_pair(i, nodes[i].score/dis));
  }  
  sort(vec.begin(), vec.end(), myComparison);
  return vec;
}*/

double
Orienteering::calculateRatio(int id, point2d_t cog){
  vector<pair<int, double> > vec;

  double dis = Distance(nodes[id].pos.x(), nodes[id].pos.y(), 0,
                          cog.x(), cog.y(), 0);
  return nodes[id].score/dis;   
}



//find the largest ratio node, insert it in the path, Center of Gravity Step.
bool
Orienteering::gravityInsert(double tmax, point2d_t cog){
  //find the largest ratio node which hasn't been in the path yet.
  double max=0;
  int max_id=0;
  for (int i=0; i<nodes.size(); i++){
    if (!nodes[i].routed){
      if (calculateRatio(i, cog)>max){
        max = calculateRatio(i, cog);
        max_id = i;
      }
    }
  }

  //cheapest insertion
  double min_cut_id, mincost = MAXINF;
  double cur = 0;
  int j;  // j is the id of nodes[i]'s parent
  for (int i=0; i<nodes.size()-1; i++){
    if (nodes[i].routed && i != start){
      j = nodes[i].parent;
      cur = Distance(nodes[max_id].pos.x(), nodes[max_id].pos.y(), 0,
                     nodes[i].pos.x(), nodes[i].pos.y(), 0) +
            Distance(nodes[max_id].pos.x(), nodes[max_id].pos.y(), 0,
                     nodes[j].pos.x(), nodes[j].pos.y(), 0) -
            Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                     nodes[j].pos.x(), nodes[j].pos.y(), 0);
      //the new insert node must be in the neighbor of i and j
      if (nodes[max_id].neighbors.find(i) !=nodes[max_id].neighbors.end() 
          && nodes[max_id].neighbors.find(j) !=nodes[max_id].neighbors.end()
          && cur < mincost){
        mincost = cur;
        min_cut_id = i;
      }
    }
  }

  if (totalCost() + mincost < tmax){
    j = nodes[min_cut_id].parent;
    nodes[min_cut_id].parent = max_id;
    nodes[max_id].parent = j;
    
    nodes[max_id].routed = true;
    return true;
  }  
  return false;
}


void
Orienteering::gravityStep(double tmax, double a, double b, double c){
  

  point2d_t cog = calculateGravity();
  init(); //initilize the path with only start and end nodes. 
  while (1){
    if (gravityInsert(tmax, cog)) updateDistance();
    else break;
  }
  improveRoute(tmax, a, b, c);
  
}

void
Orienteering::gravityStepIteration(double tmax, double a, double b, double c){
 // cout<<"gravityStep: "<<endl;
  double past=0, cur=0;
  vector<double> s, cost;
  int count = 0;

  while (1){
    gravityStep(tmax, a, b, c);
    cur = totalCost();
    s.push_back(totalScore());
    cost.push_back(cur);
    if (abs(cur-past)<0.1) break;  //stop if the route doesn't change
    past = cur;
    if (count++>10) break;
  }
  double max=0,max_cost;
  for (int i=0; i<s.size(); i++){
    if (s[i]>max){
      max = s[i];
      max_cost = cost[i];  
    } 
  }
/*  cout<<"max score: "<<max<<endl;
  cout<<"cost: "<<max_cost<<endl;
  cout<<"average: "<<max/max_cost<<endl;*/
}

vector<int>
Orienteering::path(void){
  //start -> end
  int i=end;
  vector<int> vec, path;
  while (i!=start){
    int j=nodes[i].parent;
    vec.push_back(i);
    i = j;
  }
  vec.push_back(start);
  for (int i=vec.size()-1; i>=0; i--)
    path.push_back(vec[i]);

  return path;
}


//my op algorithm
//based on Dijkstra

//distance
vector<vector<double> > 
Orienteering::createCostMatrix(vector<node_t> nodes){
  int n = nodes.size();
  vector<vector<double> > c;
  vector<double> cc;
  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      if (nodes[i].neighbors.find(j)!=nodes[i].neighbors.end()){
        double d = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0);
        cc.push_back(d);   
      }else cc.push_back(MAXINF);
    }
    c.push_back(cc);
    cc.clear();
  }
  return c;
}

//log shape function
double 
Orienteering::score_log(double t, double T){
  //10 is speed. in order to make the log score function look beautiful.
  
  return (1/(T+1) + 1 - 1/(T+1-t) - 1/(t+1));
}

//linear function
double
Orienteering::score_linear(double t, double T){
  return (t*T - t*t)/3000;
}

//linear function with time goes
double
Orienteering::score_linear_time(double t, double T, double t0){
  double k=0.00005;  
  return k*(-t*t + (t0+T)*t - t0*T);
}

//get most recent recnet_time
vector<double>
Orienteering::getRecentTime(double t, int k){
  vector<double> recent;
  //k=0, recent_time, my op
  //k=1, recent_time2, traditional op

  if (k==0){
    for (int i=0; i<nodes.size(); i++){
      if (recent_time[i].size()==1){
        recent.push_back(0);
        continue;
      }
      for (int j=0; j<recent_time[i].size()-1; j++){
        if (t>recent_time[i][j] && t<recent_time[i][j+1]){
          recent.push_back(recent_time[i][j]);
          break;
        }
      }
    }    
  }else{
    for (int i=0; i<nodes.size(); i++){
      if (recent_time2[i].size()==1){
        recent.push_back(0);
        continue;
      }
      for (int j=0; j<recent_time2[i].size()-1; j++){
        if (t>recent_time2[i][j] && t<recent_time2[i][j+1]){
          recent.push_back(recent_time2[i][j]);
          break;
        }
      }
    }
  }
  return recent;
}



//calculate the remain score at time t for my op
double
Orienteering::getRemainScore(double t, double T, int k){
  vector<double> vec_score;
  double remain_score=0;
  vec_score = getCurScore(nodes, t, T, k);

  for (int i=0; i<nodes.size(); i++){
    remain_score+=vec_score[i];
  }
/*  cout<<"k: "<<k<<endl;
  cout<<remain_score<<endl;*/

  return remain_score;
}

vector<double> 
Orienteering::getCurScore(vector<node_t> nodes, double t, double T, int k){
  vector<double> vec_score;
  vector<double> recent = getRecentTime(t, k);
  for (int i=0;i<nodes.size(); i++){
//    vec_score.push_back(score_linear(t,T) * nodes[i].score); 7.26
//    vec_score.push_back(score_linear_time(t,T,  recent_time[i]) * nodes[i].score);
    vec_score.push_back(0.5*(t-recent[i])*nodes[i].score);

  }
  return vec_score;
}

vector <double>
Orienteering::getRoute(vector<int> order){
  vector<double> res;
  double s=0;
  res.push_back(s);
  for (int i=0; i<order.size()-1; i++){
    if (order[i]!=order[i+1]){
      s+=getDistance(order[i], order[i+1]);
      res.push_back(s);
    }
  }
  return res;
}

//only for dijkstra 
vector<int>  
Orienteering::dijkstra(vector<node_t> nodes, double tmax, double radius){ 
  vector<vector<double> > c = createCostMatrix(nodes);
  int n=nodes.size();
/*  for (int i=0; i<n; i++){
    nodes[i].parent = -1;
  }*/
  vector<int> parent(n),order;
  vector<bool> visited(n,0);
  vector<float> time(n),value(n),s(n);   // function value(i)=t(i)*t(i)/s(t(i))
  float ss=0;
  float T=tmax;
  for (int i=0; i<n; i++){
    value[i]= MAXINF;
  }
  nodes[start].routed = false;
  value[start]=0;
  time[start]=0;
  double cur_res;
  for (int i=0; i<n; i++){
    int cur = -1;
    for (int j=0; j<n; j++){
      if (visited[j]) continue;
      if (cur==-1 || value[j] < value[cur]){
        cur = j;
      }
    }
    if (cur==-1 || cur==end) break;
     visited[cur] = true;
    for (int j=0; j<n; j++){
      if (visited[j]) continue;
      float nscore= MAXINF;
      if (c[cur][j]<radius){
        nscore = value[cur] + (c[cur][j] * c[cur][j])/(nodes[j].score * score_linear_time(time[cur]+c[cur][j], T, recent_time[j][recent_time[j].size()-1]));
        if (nscore < value[j] && ( time[cur]+c[cur][j] <= T )){
          s[j] = nodes[j].score * score_linear_time(time[cur]+c[cur][j], T,recent_time[j][recent_time[j].size()-1]);
          value[j] = nscore;       //update value
          parent[j] = cur;         //update parent
          nodes[j].parent = cur;   //update parent
          time[j] = time[cur]+c[cur][j];   //update time
        }

      }
    }
    
  }
  int i = end;
  while (i!=start){
    cout<<i<<" ";
    recent_time[i].push_back(time[i]);  //update node cur's recent visited time
 
    order.push_back(i);
    ss+=s[i];
    i = parent[i];
  }
  //cout<<endl;
  order.push_back(start);
  vector<int> order2;
  for (int i=order.size()-1; i>=0; i--){
    recent_time[order[i]].push_back(MAXINF);  //add MAXINF at the end of rencent_time[i]
    order2.push_back(order[i]);
  }
  average = ss/time[end]; //calculate average, keep it in average.
  return order2;
}


//for multiple dijkstra 
vector<int>  
Orienteering::dijkstra2(vector<node_t> nodes, double tmax, double radius){ 
  vector<vector<double> > c = createCostMatrix(nodes);  //c was distance matrix before, and then is time matrix
  int n=nodes.size();

  for (int i=0; i<n; i++){
    for (int j=0; j<n; j++){
      c[i][j]/=v;   //distance->time
    }
  }

  vector<int> parent(n,start),order;  //very important!!!!!!!!!!! parent(n,start)
  vector<bool> visited(n,0);
  vector<float> time(n),value(n),s(n);   // function value(i)=t(i)*t(i)/s(t(i))
  float ss=0;
  float T=tmax;
  for (int i=0; i<n; i++){
    value[i]= MAXINF;
  }
  value[start]=0;

  int start_len = recent_time[start].size();
  time[start]=recent_time[start][start_len-1];   //could update it to its recent_time
  double cur_res;
  for (int i=0; i<n; i++){
    int cur = -1;
    for (int j=0; j<n; j++){
      if (nodes[j].visited) continue;
      if (visited[j]) continue;
      if (cur==-1 || value[j] < value[cur])
        cur = j;  
    }
    if (cur==-1 || cur==end) break;
    visited[cur] = true;
    for (int j=1; j<n; j++){
      if (nodes[j].visited) continue;
      if (visited[j]) continue;
      float nscore= MAXINF;
      if (c[cur][j]<radius){
        //????
        nscore = value[cur] + (c[cur][j] * c[cur][j])/(nodes[j].score * score_linear_time(time[cur]+c[cur][j], T, recent_time[j][recent_time[j].size()-1]));
        if (nscore < value[j] && ( time[cur]+c[cur][j] <= T )){
          //update score
          s[j] = nodes[j].score * score_linear_time(time[cur]+c[cur][j], T, recent_time[j][recent_time[j].size()-1]);
          value[j] = nscore;
          parent[j] = cur;
          time[j] = time[cur]+c[cur][j];  //update time
        }
      }
    }
    
  }
  int i = end;
  int count=0;
  while (i!=start){
    order.push_back(i);
    ss+=s[i];
    i = parent[i];
  }
  order.push_back(start);
  vector<int> order2;
  for (int i=order.size()-1; i>=0; i--){
    order2.push_back(order[i]);
    cur_recent_time[end][order[i]]=time[order[i]];
  }
  average = ss/(time[end]-time[start]); //calculate average, keep it in average.  
  if (time[end]>tmax) order2.clear(); //return 0
  return order2;
}

vector<int>  
Orienteering::op(vector<node_t> nodes, double tmax, double radius){ 
  vector<vector<double> > c = createCostMatrix(nodes);

  int n=nodes.size();
/*  for (int i=0; i<n; i++){
    nodes[i].parent = -1;
  }*/
  vector<int> parent(n),order;
  vector<bool> visited(n,0);
  vector<float> time(n),value(n),s(n);   // function value(i)=t(i)*t(i)/s(t(i))
  float ss=0;
  float T=tmax;

  for (int i=0; i<n; i++){
    value[i]= MAXINF;
//    visited[i]=false;
  }
//  visited[start]=true;
  nodes[start].routed = false;
  value[start]=0;
  time[start]=0;
//  int cur = start;
  double cur_res;
  for (int i=0; i<n; i++){
    int cur = -1;
  //  cur_res = MAXINF;
    for (int j=0; j<n; j++){
//      if (nodes[j].routed) continue;
      if (visited[j]) continue;
      if (cur==-1 || value[j] < value[cur]){
        cur = j;
    //    cur_res = value[j];
      }
    }
    if (cur==-1 || cur==end) break;
    update_time[end][cur] = time[cur];  //update node cur's recent visited time
//    nodes[cur].routed = true;
    visited[cur] = true;
    for (int j=0; j<n; j++){
      if (visited[j]) continue;
      float nscore= MAXINF;
      if (c[cur][j]<radius){
          //nodes[j].score is k, score_log is x. score=kx.  rule: t*t/s, speed is 1 m/s
      //    nscore = value[cur] + (c[cur][j] * c[cur][j])/(nodes[j].score * score_linear(double(time[cur]+c[cur][j]),T));
          //latest version 7.26

          nscore = value[cur] + (c[cur][j] * c[cur][j])/(nodes[j].score * score_linear_time(time[cur]+c[cur][j], T, recent_time[j][recent_time[j].size()-1]));

//        nscore = value[cur] + (c[cur][j] * c[cur][j])/nodes[j].score; //latest version
//          nscore = value[cur] + (c[cur][j])/RandNo();
//        path = value[cur] + c[cur][j];
        if (nscore < value[j] && ( time[cur]+c[cur][j] <= T )){
          //update j's neighbor
          //s[j] is score, value[j] is the sum of t*t/s from start node
      //    s[j] = nodes[j].score * score_linear(double(time[cur]+c[cur][j]),T);
          s[j] = nodes[j].score * score_linear_time(time[cur]+c[cur][j], T, time[j]);
          value[j] = nscore;
          parent[j] = cur;
          nodes[j].parent = cur; 
          time[j] = time[cur]+c[cur][j];
        }

      }
    }
    
  }
  int i = end;
  while (i!=start){
 //   cout<<i<<" ";
    order.push_back(i);
    ss+=s[i];
    i = parent[i];
    
    if (i==0) {
 //     cout<<i<<" ";
      break;
    }
  //  if (i==0) break;

  }
 // cout<<start<<" "<<endl;
  order.push_back(start);
  vector<int> order2;
  for (int i=order.size()-1; i>=0; i--){
    order2.push_back(order[i]);
  }
  average = ss/time[end]; //calculate average, keep it in average.
  
/*  cout<<"T: "<<T<<endl;
  cout<<"time cost: "<<time[end]<<endl;
  cout<<"value: "<<value[end]<<endl;
  cout<<"total score: "<<ss<<endl;
  cout<<"*********average: "<<ss/time[end]<<endl;*/
  return order2;
}


//haven't update the visited function yet.
vector<int> 
Orienteering::opFindBest(double tmax, double radius){
  vector<int> res; //find the most efficient route 
//  vector<int> newvec; //current order result
  //at first, start is start, end is the best one in its neighbor
  int res_end;  
  average = 0; //init average in each iteration
  for (int i=1; i<nodes.size(); i++){

    if (nodes[start].op_neighbors.find(i) != nodes[start].op_neighbors.end()){
      setStartEnd(start, i);
      double old = average;
      vector<int> newvec = dijkstra2(nodes, tmax, radius);
//      if (newvec.size()<4) continue;
//      cout<<newvec.size()<<endl;
      if (newvec.size()==0) {
        average = old;
        continue;
      }
      if (average > old ) {
        res = newvec;  //update res
        res_end = i;
      }
      else average = old;
    }
  }
  if (res.size()==0) return res;
  for (int i=0; i<res.size()-1; i++){
    nodes[res[i]].visited = true;
  }
  for (int i=0; i<res.size()-1; i++){
    cur_t+=getDistance(res[i], res[i+1])/v;
  }
  //update recent_time
  for (int i=1; i<res.size(); i++){
    recent_time[res[i]].push_back(cur_recent_time[res_end][res[i]]);
  }  
  for (int i=0; i<nodes.size(); i++){
    cur_recent_time[i].clear();
    cur_recent_time[i].resize(nodes.size());
  }
  setStartEnd(res_end, res_end);
  return res;
}

vector<int>
Orienteering::opIteration(double tmax, double radius){
  vector<vector<int> > order;
  int count=0;
  while (1){
    cout<<"cur_t: "<<cur_t<<endl;

    order.push_back(opFindBest(tmax, radius));
    int order_size = order.size();
//    cout<<"order size: "<<order[order_size-1].size()<<endl;
    if (cur_t>tmax) break;
    if (++count>30) break;
  }
  vector<int> order_op;
  cout<<"order_op~~~~~~~~~~~~~~~~~~~~~"<<endl;
  for (int i=0; i<order.size(); i++){
    for (int j=0; j<order[i].size(); j++){
      cout<<order[i][j]<<" ";
    }
    if (order[i].size()) cout<<endl;
  }

  for (int i=0; i<order.size(); i++){
    for (int j=0; j<order[i].size(); j++){
      if (i && !j) continue;
      order_op.push_back(order[i][j]);
      cout<<order[i][j]<<" ";    
    }
    if (order[i].size()) cout<<endl;
  }
  vector<bool> flag(nodes.size(), 0);
  for (int i=0; i<order_op.size(); i++){
    if (!flag[order_op[i]]){
      recent_time[order_op[i]].push_back(MAXINF);
      flag[order_op[i]]=true;
  
    }
  }
  return order_op;
}


//update recent_time2 after gravityStepIteration
//for traditional op
void
Orienteering::updateRecentTime2(){
  recent_time2.resize(nodes.size());
  for (int i=0; i<nodes.size(); i++){
    recent_time2[i].push_back(0);
  }

  vector<int> route = path();
  double t=0;
  cout<<"original op route size: "<<route.size()<<endl;
  for (int i=0; i<route.size()-1; i++){
    if (i) recent_time2[route[i]].push_back(t);
    recent_time2[route[i]].push_back(MAXINF);
    t+=getDistance(route[i], route[i+1])/v;   //v is needed here
  }
  recent_time2[route[route.size()-1]].push_back(t);
  recent_time2[route[route.size()-1]].push_back(MAXINF);

}


//given the order, calculate the average in one stage(Tmax)
void
Orienteering::calculateAverage(double tmax, int k){
  double total_score[2]={0,0};
  for (int i=1; i<tmax; i++){
    total_score[k]+=getRemainScore(i, tmax, k);
  }
  cout<<"average-> k: "<<k<<" "<<total_score[k]/tmax<<endl;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*new op, Sept 8th
1. initilization
 a. delete long distance node
 b. remove node until the total cost is smaller than T 
2. tsp path
3. delete node, insert node


*/

//increasing order
/*bool 
Orienteering::compare_score(node_t a, node_t b){return a.score < b.score;}
*/
//decreasing order
/*bool
Orienteering::compare_eff(node_i a, node_i b){return a.eff > b.eff;}
*/


double
Orienteering::getDistanceById(int i, int j){
  int index_i = findIndex(i);
  int index_j = findIndex(j);
  return Distance(nodes[index_i].pos.x(), nodes[index_i].pos.y(), 0,
                  nodes[index_j].pos.x(), nodes[index_j].pos.y(), 0);
}


//calculate the total cost of tsp path, path!!!
double
Orienteering::pathCost(vector<int> tsp){
  double s=0;
  int k;
  for (int i=0; i<tsp.size()-1; i++)
    s+=getDistanceById(tsp[i],tsp[i+1]);
  return s;
} 


//input tsp_set and order from tsp(), output the path, from the start to end
vector<int>
Orienteering::transform(vector<int> tsp_set, vector<int> order){
  vector<int> path;
  vector<int> newpath;



  for (int i=0; i<order.size(); i++)
    path.push_back(tsp_set[order[i]]);
  if (getDistanceById(start, path[1]) > getDistanceById(start, path[path.size()-1])){
    newpath.push_back(start);
    for (int i=path.size()-1; i>=1; i--)
      newpath.push_back(path[i]);
    return newpath;
  }
  return path;
}

/*
create tsp Matrix and then output to the file tsp.txt
example
start=0
0 1 1 1
0 0 1 1
0 1 0 1
0 1 1 0
*/

void
Orienteering::createTspMatrix(vector<int> node){
  //for example, 3,56,1,0,2,4,...,12
  vector<vector<double> > mat;
  vector<double> vec;
  for (int i=0; i<node.size(); i++){
    for (int j=0; j<node.size(); j++){
      vec.push_back(getDistanceById(node[i],node[j]));  //take care, id id id !!!
    }
    mat.push_back(vec);
    vec.clear();
  }
  for (int i=0; i<node.size(); i++)
    mat[i][start]=0;
  

  ofstream output("tsp.txt"); 
  for (int i=0; i<mat.size(); i++){
    for (int j=0; j<mat.size(); j++){
      output<<mat[i][j]<<" ";
    }
    output<<endl;
  }
  output.close();
}




//return the path from start to end, end is not defined.
vector<int>  
Orienteering::opInit(double T){
  int n=nodes.size();
  vector<int> order; //order is from tsp(), for example, 2,1,4,3,0
  for (int i=0; i<n; i++){
    nodes[i].tsp_flag=false;
    nodes[i].radius_flag=true;
  }
  //delete long distance node > T
  for (int i=0; i<n; i++){
    if (i!=start){
      if (getDistanceById(i, start)>T) {
        nodes[i].radius_flag=false;
      }
    }
  }


  int k=0; //k is the index of current feasible(tsp_flag=true) smallest node.
  //delete really small score node
  //sort the node's score in increasing order
  sort(nodes.begin(), nodes.begin()+n, compare_score);  

  //create the initial set of feasible nodes for tsp, the order is increasing order
  //for example
  //5,4,3,2,1 score
  vector<int> tsp_set;
  for (int i=n-1; i>=0; i--){
    if (nodes[i].radius_flag) tsp_set.push_back(nodes[i].id);
  }

  double tsp_time=MAXINF; //tsp_time is the time cost
  vector<int> path;
  while(tsp_time>T){
    tsp_set.pop_back();  //remove the last node
    createTspMatrix(tsp_set);
    order = tsp("tsp.txt", tsp_set.size());
    path = transform(tsp_set, order);
    //get the current feasible nodes set
    //tsp(feasible set)
    tsp_time = pathCost(path);
  }


  for (int i=0; i<tsp_set.size(); i++){
    nodes[findIndex(tsp_set[i])].tsp_flag = true;
  }

  return path;  
}


vector<int>
Orienteering::getTimeClock(vector<int> path){
  vector<int> time;
  double t=0;
  int n = path.size();
  time.push_back(0);
  for (int i=0; i<n-1; i++){
    t+=getDistanceById(path[i], path[i+1]);
    time.push_back(t);  
  } 
  return time;
}


//delete node in the T/2 of the path
//insert node in the start and end of the path
vector<int>
Orienteering::opOperation(double T){
  vector<int> path2;
//  path = opInit(T);
  
  double a = 0.5, b = 0.2, c = 0.3;
  setStartEnd(0, 10);
  constructRoute(T, a, b, c);
  improveRoute(T, a, b, c);
  gravityStepIteration(T, a, b, c);
  path2 = path();

    //inilization
  for (int i=0; i<nodes.size(); i++){
    nodes[i].tsp_flag=false;
    nodes[i].radius_flag=true;
  }
  //delete long distance node > T
  for (int i=0; i<nodes.size(); i++){
    if (i!=start){
      if (getDistanceById(i, start)>T) {
        nodes[i].radius_flag=false;
      }
    }
  }
  for (int i=0; i<path2.size(); i++){
    nodes[findIndex(path2[i])].tsp_flag = true;
  }


  cout<<"path: "<<endl;
  double op_score=0;
  for (int i=0; i<path2.size(); i++){
    cout<<path2[i]<<" ";
    if (path2[i]) op_score += nodes[findIndex(path2[i])].score;
    cout<<op_score<<endl;
  }
  cout<<endl;
  cout<<"the total after op algorithm"<<endl;
  cout<<op_score<<endl; // after op, the total score of each node.
 // return path2;
  cout<<"the total time cost"<<endl;
  cout<<pathCost(path2)<<endl;
  //return path2;
  int n=path2.size(); 
  //record the time clock for each node
  vector<int> time;
 
  //calculate the average
  double s=0,average;
  //the start node is excluded
  for (int i=1; i <n; i++){
    s+=nodes[findIndex(path2[i])].score;
  }
  average = s/n;
  //remove node
  time=getTimeClock(path2);
  for (int i=0; i<n; i++){
    if (nodes[findIndex(path2[i])].score < average && (abs(time[i]-T/2) < T/6 )){
      nodes[findIndex(path2[i])].tsp_flag = false;
      cout<<nodes[findIndex(path2[i])].score<<endl;
    }
  }
  //new path after remove some small score nodes near T/2
  vector<int> newpath;
  time=getTimeClock(newpath);
  for (int i=0; i<n; i++)
    cout<<nodes[findIndex(path2[i])].tsp_flag<<" ";
  for (int i=0; i<n; i++){
    if (nodes[findIndex(path2[i])].tsp_flag) newpath.push_back(path2[i]);
  }
  n=newpath.size();
  cout<<"new path size: "<<n<<endl;
  for (int i=0; i<newpath.size(); i++)
    cout<<newpath[i]<<" ";
  //find the node index in about T/2
  int id_lowbound=0, id_highbound=0, id_t2=0;
  for (int i=0; i<n; i++)
    if (time[newpath[i]]>T/2){
      id_t2 = i;
      break;
    }
  for (int i=0; i<n; i++)
    if (time[newpath[i]>T/4]){
      id_lowbound=i;
      break;
    }
  for (int i=0; i<n; i++)
    if (time[newpath[i]]>3*T/4){
      id_highbound=i;
      break;
    }


  //we won't change the start node and end node, just for easy computation
  //get the cheapest insertion place 
  vector<double> cheapest_insert_dif; 
  vector<int> cheapest_insert_num;
  vector<bool> newpath_flag(nodes.size(), 1);
  for (int i=0; i<nodes.size(); i++){
    if (nodes[i].radius_flag && !nodes[i].tsp_flag){
      double min=MAXINF;
      int num;
      for (int j=0; j<n-1; j++){
//        if (newpath_flag[newpath[j]])
          if (getDistanceById(nodes[i].id, newpath[j]) + getDistanceById(nodes[i].id, newpath[j+1]) - getDistanceById(newpath[j], newpath[j+1]) < min){
            min = getDistanceById(nodes[i].id, newpath[j]) + getDistanceById(nodes[i].id, newpath[j+1]) - getDistanceById(newpath[j], newpath[j+1]);
            num = newpath[j];
          }
      }
    //  newpath_flag[num]=0;
      cheapest_insert_dif.push_back(min);
      cheapest_insert_num.push_back(num);
    }else{
      cheapest_insert_dif.push_back(MAXINF);
      cheapest_insert_num.push_back(0);
    }
  }

  //test

  //sort in decreasing order of k/cheap_insert_dif
/*  typedef struct Node {
    uint id;
    double score;
    double eff;
    double dis;
    int pos;
  } node_i;
*/
  vector<node_i> insertion_node;
  node_i node;
  cout<<endl;
  for (int i=0; i<nodes.size(); i++){
    node.score = nodes[i].score;
    node.dis = cheapest_insert_dif[i];
    node.eff = node.score/node.dis;
    node.id = nodes[i].id;
    node.num = cheapest_insert_num[i];
    insertion_node.push_back(node);
    //cout<<node.num<<" ";
//    cout<<"id: "<<node.id<<" score"<<node.score<<" dis: "<<node.dis<<" tsp_flag: "<<nodes[i].tsp_flag<<endl;
  }
  cout<<endl;
  //sort the node in decreasing order of score/dis
  sort(insertion_node.begin(), insertion_node.begin()+nodes.size(), compare_eff);

  //insert the tsp_flag = false node into the start and 

  double t;
  t = pathCost(newpath);
  int k = 0;


  //it seems there exists some problem in insertion.

  while(t<T && k<insertion_node.size()){
    node_i cur_node_i = insertion_node[k];
    k++;
    node_t cur_node_t = findNode(cur_node_i.id);
    if (!cur_node_t.tsp_flag && cur_node_t.radius_flag ){
      if (newpath_flag[cur_node_i.num] && t+cur_node_i.dis < T){
        int position = findPos(newpath, cur_node_i.num);
        newpath.insert(newpath.begin()+position+1, cur_node_i.id);
        newpath_flag[cur_node_i.num]=0;
        cout<<"id: "<<cur_node_i.id<<" score: "<<cur_node_i.score<<" dif: "<<cur_node_i.dis
        <<" pos: "<<position<<endl;
      }
    }
    t=pathCost(newpath);
  }
  cout<<"insertion finish"<<endl;

  op_score=0;
  for (int i=0; i<newpath.size(); i++){
    cout<<newpath[i]<<" ";
    if(newpath[i]) op_score+=nodes[findIndex(newpath[i])].score;
  }
  cout<<"the final total score"<<endl;  
  cout<<op_score<<endl;
  cout<<"the final time cost"<<endl;
  cout<<pathCost(newpath)<<endl;
  return newpath;

}

//find the index based on its id
int
Orienteering::findIndex(int ID){
  for (int i=0; i<nodes.size(); i++){
    if (nodes[i].id == ID) return i;
  }
}

node_t
Orienteering::findNode(int ID){
  for (int i=0; i<nodes.size(); i++){
    if (nodes[i].id==ID) return nodes[i];
  }
}

int 
Orienteering::findPos(vector<int> path, int num){
  for (int i=0; i<path.size(); i++){
    if (path[i]==num) return i;
  }
  return -1;

}