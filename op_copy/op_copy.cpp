#include "op.h"
#define MAXINF 100000
#define MININF 0.00001

void
Orienteering::createNodes(uint _size, uint _seed, double radius)
{

    nodes=RandomTopo(_size, BOUND_X, BOUND_Y, _seed, radius);    

}



void
Orienteering::init(void){
    for (int i=0; i<nodes.size(); i++){
        nodes[i].routed = false;  //for traditional op
        nodes[i].parent = start; //initilize each node's parent
        nodes[i].son = end;
        nodes[i].df = getDistance(start, i) + getDistance(end, i);  //the sum of distances to foci of eclipse
        nodes[i].ok = true;
    }
    nodes[start].routed = true;
    nodes[end].routed = true; 
    nodes[start].score = 0;  //because start and end nodes are fixed, there is no need to consider their score.
    nodes[end].score = 0;
}

double
Orienteering::getDistance(int i, int j){
  //cout<<i<<" "<<j<<endl;
    return Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0);
}


double
Orienteering::totalCost(void){
    int i=end;
    double s=0;
    while (i!=start){
      int j=nodes[i].parent;
      s+= getDistance(i,j);
      i = j;
    }
    return s;
}

double 
Orienteering::totalPathCost(vector<int> path){
  double s=0;
  for (int i=0; i<path.size()-1; i++){
    s+=getDistance(path[i],path[i+1]);
  }
  return s;
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


//.routed mark
double 
Orienteering::totalScore(void){
    double s=0;
    for (int i=0; i<nodes.size(); i++){
      if (nodes[i].routed) s+=nodes[i].score;
    }
    return s;
}

double
Orienteering::totalPathScore(vector<int> path){
  double s=0;
  for(int i=0; i<path.size(); i++){
    s+=nodes[path[i]].score;

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


//calculate the rank of those unvisited nodes
int 
Orienteering::calculateRank(int id,  double a, double b, double c){
  //score rank, center-of-gravity rank, a sum of distances to the two foci of an ellipse rank
    int sr=1, cr=1, er=1; 
    for (int i=0; i<nodes.size(); i++){
      if (!nodes[i].routed){
          if (nodes[i].score > nodes[id].score) sr++;
          if (nodes[i].dc < nodes[id].dc) cr++;   //cog
          if (nodes[i].df < nodes[id].df) er++;   //foci
      }
    }
    return (a*sr + b*cr + c*er);
}


//in the route construction 
int
Orienteering::getFirstRankNode(double a, double b, double c){
  double min = MAXINF; 
  int min_id = 0;
  for (int i=0; i<nodes.size(); i++){
    if (!nodes[i].routed && nodes[i].ok){
      if (calculateRank(i, a, b, c)<min){
        min = calculateRank(i, a, b, c);
        min_id = i;
      }
    }
  }
  return min_id;
}


//update dc, distance to center of gravity
void
Orienteering::updateDistance(void){
    point2d_t cog = calculateGravity();
    for (int i=0; i < nodes.size(); i++){
    //update those haven't been routed
          nodes[i].dc = sqrt( (nodes[i].pos.x()-cog.x())*(nodes[i].pos.x()-cog.x()) + 
                              (nodes[i].pos.y()-cog.y())*(nodes[i].pos.y()-cog.y()) );
        //  nodes[i].dc = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
        //                cog.x(), cog.y(), 0); 
       
          if (nodes[i].dc<0.001)
            nodes[i].dc = 0.001;       
     
    }

}

//in the construct route 
bool
Orienteering::constructRouteInsert(double tmax, double a, double b, double c){
    updateDistance();
    //find the rank 1 node, min_id is its id
    int min_id=0, min = MAXINF;  
    min_id = getFirstRankNode(a,b,c);
    if (min_id==0) return false;
    //cheapest insertion
    //find the cheapest insertion place, min_cut_id is its one side.
    //nodes[min_cut_id].parent is its other side.
    double min_cut_id, mincost = MAXINF;
    double cur = 0;
    int j;  // j is the id of nodes[i]'s parent

    for (int i=0; i<nodes.size(); i++){
      if (nodes[i].routed && i != start){
          j = nodes[i].parent;
          cur = getDistance(min_id, i) + getDistance(min_id, j) - getDistance(i,j);
          if (cur < mincost){
            mincost = cur;
            min_cut_id = i;
          }
      }
    }

    //start  j  i  end
    //start  min_cut_id.parent min_cut_id end
    //start  min_cut_id.parent min_id min_cut_id end
    if (totalCost() + mincost < tmax){
      j = nodes[min_cut_id].parent;
      nodes[min_cut_id].parent = min_id;
      nodes[min_id].parent = j;
      nodes[min_id].routed = true;
//      return true;
    }else nodes[min_id].ok = false;   
    return true;
}



void
Orienteering::constructRoute(double tmax){
  double a, b, c;
  double res_a=0.3, res_b=0.3, res_c=0.4;
  double max_score = 0;
  for (double a = 0; a <= 1.0; a+=0.1){
    for (double b= 0; b <= 1.0; b+=0.1){
      c = 1-a-b;
      if (c<0) continue;
      init();
      while(constructRouteInsert(tmax, a, b, c));
      if (totalScore() > max_score){
        max_score = totalScore();
        res_a = a;
        res_b = b; 
        res_c = c;
      }
    }
  }
  res_a = 1;
  res_b =0;
  res_c =0;
  init();
  while(constructRouteInsert(tmax, res_a, res_b, res_c));
  cout<<res_a<<" "<<res_b<<" "<<res_c<<endl;
  //improve the initial path

  while(1){
    while(constructRouteInsert(tmax, res_a, res_b, res_c));
    opt();
    if (!constructRouteInsert(tmax, res_a, res_b, res_c)) break;
  }

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
      if (changed){     
          reverse(order.begin()+(ii+1),order.begin()+(jj+1)); 
      }
    }
    //update parent information
    for (int i=order.size()-1; i>0; i--){
      nodes[order[i]].parent = order[i-1];
    }
}


// score/distance(cog, id)
double
Orienteering::calculateRatio(int id){
    return nodes[id].score/nodes[id].dc;   
}





vector<int>
Orienteering::gravityInsertOrder(){
  updateDistance();
  vector<node_o> vec;
  node_o node;
  for (int i=0; i<nodes.size(); i++){
    if (i!=start && i!=end){
      node.ratio = nodes[i].score/nodes[i].dc;
      node.id = i;
      vec.push_back(node);
    }
  }
  sort(vec.begin(), vec.begin()+vec.size(), compare_ratio);
  vector<int> order;
  for (int i=0; i<vec.size(); i++){
    order.push_back(vec[i].id);
  //  cout<<vec[i].ratio<<" "<<nodes[vec[i].id].score<<" "<<nodes[vec[i].id].dc<<endl;
  }
  return order;
}


//find the largest ratio node, insert it in the path, Center of Gravity Step.
bool
Orienteering::gravityInsert(double tmax, int id){
    int max_id = id;

  //cheapest insertion
    double min_cut_id, mincost = MAXINF;
    double cur = 0;
    int j;  // j is the id of nodes[i]'s parent
   // cout<<start<<" "<<end<<endl;
    for (int i=0; i<nodes.size(); i++){
      if (nodes[i].routed && i != start){
          j = nodes[i].parent;  
          cur = getDistance(max_id, i) + getDistance(max_id, j) - getDistance(i,j);
          if (cur < mincost){
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
Orienteering::gravityStep(double tmax){
  vector<int> order = gravityInsertOrder();
  init();  //clear the parent and routed information
  int k=0;
  vector<int> order_flag;
  for (int i=0; i<order.size(); i++)
    order_flag.push_back(1);
  for (int i=0; i<order.size(); i++){
    k=i;
   // if (i<order.size()-1 && !gravityInsert(tmax, order[i]) && i<order.size()-2)
  //  if (!gravityInsert(tmax, order[i])) continue;
    //gravityInsert(tmax, order[++i]);
    bool a = gravityInsert(tmax, order[i]);
    if (a) order_flag[i]=0;
  }
  k=0;
  int count=0;
//  cout<<"aaaa"<<endl;
  while(count<10){
    count++;
    opt();
    while(k<order.size() && order_flag[k] && !gravityInsert(tmax, order[k])){   //choose the proper order[k], no duplicate
      k++;
    }
    k=0;
  } 
//  cout<<"bbb"<<endl;
}


vector<int> 
Orienteering::gravityStepIteration(double tmax){
  init();
  vector<int> bestpath;
  int max_score=0, count=0; 
  constructRoute(tmax);
  vector<int> initialpath;
  double initialscore=0;
  initialpath=path();
  initialscore=totalScore();
  cout<<endl;
  cout<<"path size: "<<path().size()<<endl;
  cout<<"totoal cost: "<<totalCost()<<endl;
  cout<<"score: "<<totalScore()<<endl;
  cout<<"construct route finish"<<endl;
  int node_num=0;
  for (int i=0; i<nodes.size(); i++)
    if (nodes[i].routed) node_num++;
  if (node_num==nodes.size()) return path();
  
  for (int i=0; i<path().size(); i++)
    cout<<path()[i]<<" ";
  cout<<"begin gravityStep "<<endl;
  while(count<10){
    gravityStep(tmax);
    if (totalScore()>max_score){
      max_score = totalScore();
      bestpath = path();      
    }    
    count++;
  }
  cout<<"path size: "<<bestpath.size()<<endl;
  cout<<"total cost: "<<totalPathCost(bestpath)<<endl; 
  cout<<"score: "<<totalPathScore(bestpath)<<endl;
  
  if (totalPathScore(bestpath)<initialscore)
    bestpath = initialpath;  
  cout<<"best path:"<<endl;
  for (int i=0; i<bestpath.size(); i++)
    cout<<bestpath[i]<<" ";  
  init();
  for(int i=1; i<bestpath.size(); i++)
    nodes[bestpath[i]].parent = bestpath[i-1];
  for (int i=0; i<bestpath.size()-1; i++)
    nodes[bestpath[i]].son = bestpath[i+1];
  return bestpath;
}

/*
before swap
start->...->i->j->...->end
start->...i's parent->i->j->j's son->...->end
after swap
start->...->j->i->...->end
start->...i's parent->j->i->j's son->...->end

i->j
*/
void
Orienteering::swap(int i, int j){
  int j_son, i_parent;
  j_son = nodes[j].son;
  i_parent = nodes[i].parent;

  //swap operation
  nodes[j_son].parent = i;
  nodes[i].parent = j;
  nodes[j].parent = i_parent;

  nodes[i_parent].son = j;
  nodes[j].son = i;
  nodes[i].son = j_son;
}


vector<int> 
Orienteering::improveOpRoute(double tmax){
  vector<int> finalpath = path();
  vector<double> time = timeClock(finalpath);
  //total profit before swap 
  cout<<endl;
  cout<<"total profit before swap: "<<totalProfit(finalpath, tmax)<<endl;
  cout<<"total cost: "<<totalPathCost(finalpath)<<endl;
  int id_t2=0;   //the number of the node in the path
  double min_dif_time=MAXINF;
  for (int i=0; i<time.size(); i++){
    
    if (abs(time[i]-tmax/2) < min_dif_time){
      min_dif_time = abs(time[i]-tmax/2);
      id_t2 = i;
    }
  }
  cout<<"id_t2: "<<id_t2<<endl;

/*
start->i_parent->i->j->j_son->...->path[id_t2]...->end
start->i_parent->j->i->j_son->...->path[id_t2]...->end

nodes[i].score > nodes[j].score
*/
  double cost=0;    // cost = dis(i_parent, j)+dis(i,j_son)-dis(i_parent,i)-dis(j,j_son)
  double benefit=0; // score = nodes[i].score - nodes[j].score
  int i_parent, j_son;
  for (int i=1; i<id_t2-1; i++){
    int j=i+1;
    finalpath=path();
    i_parent = nodes[finalpath[i]].parent;
    j_son = nodes[finalpath[j]].son;
    cost = getDistance(i_parent, finalpath[j]) + getDistance(finalpath[i], j_son) 
          -getDistance(i_parent, finalpath[i]) + getDistance(finalpath[j], j_son);
    if (totalCost()+cost>tmax) continue;
    benefit = nodes[finalpath[i]].score - nodes[finalpath[j]].score;
    if (benefit/cost > 1) swap(finalpath[i], finalpath[j]);
  }

/*
start->...->path[id_t2]->...->i_parent->i->j->j_son->...->end
start->...->path[id_t2]->...->i_parent->j->i->j_son->...->end

nodes[i].score < nodes[j].score
*/
  for (int i=id_t2+1; i<finalpath.size()-2; i++){
    int j=i+1;
    finalpath=path();
    i_parent = nodes[finalpath[i]].parent;
    j_son = nodes[finalpath[j]].son;
    cost = getDistance(i_parent, finalpath[j]) + getDistance(finalpath[i], j_son) 
          -getDistance(i_parent, finalpath[i]) + getDistance(finalpath[j], j_son);
    if (totalCost()+cost>tmax) continue;
    benefit = nodes[finalpath[j]].score - nodes[finalpath[i]].score;
    if (benefit/cost > 1) swap(finalpath[i], finalpath[j]);
  }


  cout<<path().size()<<endl;
  //total profit after swap 
  cout<<endl;
  cout<<"total profit after swap: "<<totalProfit(path(), tmax)<<endl;
  cout<<"total cost: "<<totalPathCost(path())<<endl;
  for (int i=0; i<path().size(); i++)
    cout<<path()[i]<<" ";
  cout<<"ff"<<endl;
  return path();
}

double
Orienteering::score_func(int id, double t, double tmax){
  return (nodes[id].score*(-t*t+tmax*t))/100;
}

vector<double>
Orienteering::timeClock(vector<int> path){
  vector<double> time;
  double t=0;
  time.push_back(t);
  for (int i=0; i<path.size()-1; i++){
    t+=getDistance(path[i],path[i+1]);
    time.push_back(t);
  }
  return time;
}

double
Orienteering::totalProfit(vector<int> path, double tmax){
  vector<double> time = timeClock(path);
  double s=0;
  for (int i=1; i<path.size()-1; i++){
    s+=score_func(path[i], time[i], tmax);
  }
  return s; 
}

