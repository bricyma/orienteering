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

int 
Orienteering::getDistance(int i, int j){
    return int(Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0));
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


vector<int> 
Orienteering::getPath(){
  int a[]={4,2,5,0,3,6};
  vector<int> p(a,a+1);
  return p;

}

int 
Orienteering::edge_len(int i, int j, int t){
  return int(getDistance(i,j));
}

node2 
Orienteering::findNodeByIdT(int node_id, int node_t){

  for (int i=0; i<node.size(); i++){
    if (node[i].id==node_id && node[i].t==node_t)
      return node[i];
  }

}


//index [i][t]
void 
Orienteering::initGraph(){
  node2 cur_node;
  int n=nodes.size(); 
  int k=0;
  for (int i=0; i<n; i++){
    for (int j=0; j<=T; j++){
      cur_node.id = i;  //id is the real index of the nodes, logical
      cur_node.index = k++; //index is the index of the node
      cur_node.t = j;
      cur_node.visited = false;
      cur_node.dist = MAXINF;
      cur_node.prev = -1;
      cur_node.neighbors = nodes[i].neighbors;
      cur_node.score = nodes[i].score*(-j*j+j);
      node.push_back(cur_node);
    }
  }

  for (int i=0; i<n; i++){
    for (int j=0; j<nodes[i].neighbors.size(); j++){
      cout<<nodes[i].neighbors[j]<<" ";
    }
    cout<<endl;
  }
}


void 
Orienteering::dijkstra(){
  initGraph();
  int source_id = findNodeByIdT(start, 0).id;
  node[source_id].dist = 0;

  int n=node.size(); //n = nodes.size() * T
  for (int i=0; i<n; i++){
    int min_index,min=MAXINF;
    for (int j=0; j<n; j++){
      
      if (!node[j].visited && node[j].dist<min){
        min=node[j].dist;
        min_index=j;
      }
    }
    if (min==MAXINF) break;
    
    node[min_index].visited = true;


    cout<<"id: "<<min_index<<" real id: "<<node[min_index].id<<" dist: "<<min<<endl;
    for (int j=0; j<node[min_index].neighbors.size(); j++){
      //neighbor_id is of the ids in nodes
      int neighbor_id = node[min_index].neighbors[j];
      int cur_edge_len = edge_len(node[min_index].id, neighbor_id, min);
      int newvalue = node[min_index].dist + cur_edge_len;
      if (newvalue>=T) break;
      node2 neighbor = findNodeByIdT(neighbor_id, newvalue);
      int neighbor_index = neighbor.index;
      if (newvalue < neighbor.dist && !neighbor.visited){
        
        node[neighbor_index].dist = newvalue;
        node[neighbor_index].prev = min_index;
      }

    }


  }
    //find the least time for the end node
  int min_res_id, min_res=MAXINF;
  cout<<end<<endl;


  node2 end_node = findNodeByIdT(end,0); //n=10, s=2, why add this line???? TODO
  int end_index = findNodeByIdT(end, 0).index;
  for (int i=0; i<=T; i++){
    if (node[end_index+i].dist < min_res){
      min_res_id = end_index+i;
      min_res = node[end_index+i].dist;
    }
  }
 
  int i=min_res_id;
  vector<int> res;
  cout<<node[i].id<<endl;
  while(node[i].id!=start){
    res.push_back(node[i].id);
    i = node[i].prev;
    cout<<i<<endl;
  }
  res.push_back(start);
  cout<<endl;
  for (int i=res.size()-1; i>=0; i--)
    cout<<res[i]<<" ";
  cout<<endl;
  cout<<"min_res: "<<min_res<<endl;  
}


//op construct route without time variant

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


//calculate the all the nodes' cog
point2d_t
Orienteering::calculateGravityAll(void){
    double coordx=0, coordy=0;
    double s=0;  
    point2d_t pos;
    for (int i=0; i<nodes.size(); i++){
        coordx += nodes[i].pos.x() * nodes[i].score;
        coordy += nodes[i].pos.y() * nodes[i].score;
        s += nodes[i].score;
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
    point2d_t cog;
    if (first){
      cog = calculateGravityAll();
      first = false;  
    } else cog = calculateGravity();
    for (int i=0; i < nodes.size(); i++){
    //update those haven't been routed
          nodes[i].dc = sqrt( (nodes[i].pos.x()-cog.x())*(nodes[i].pos.x()-cog.x()) + 
                              (nodes[i].pos.y()-cog.y())*(nodes[i].pos.y()-cog.y()) );
        //  nodes[i].dc = Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
        //                cog.x(), cog.y(), 0); 

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
  double res_a=0.3, res_b=0.1, res_c=0.6;
  double max_score = 0;
  for (double a = 0; a <= 1.0; a+=0.05){
    for (double b= 0; b <= 1.0; b+=0.05){
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

  init();
  while(constructRouteInsert(tmax, res_a, res_b, res_c));
  cout<<res_a<<" "<<res_b<<" "<<res_c<<endl;
  //improve the initial path

  while(1){
    while(constructRouteInsert(tmax, res_a, res_b, res_c));
    opt();
    if (!constructRouteInsert(tmax, res_a, res_b, res_c)) break;
  }

  cout<<"total score in the construct route: "<<totalScore()<<endl;
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

double 
Orienteering::totalScore(void){
    double s=0;
    for (int i=0; i<nodes.size(); i++){
      if (nodes[i].routed) s+=nodes[i].score;
    }
    return s;
}


/*GENETIC ALGORITHMS solving incomplete graph OP 
1. select 
2. crossover
3. mutation

*/
void 
Orienteering::genetic_init(){
  for (int i=0; i<nodes.size(); i++)
    nodes[i].visited = false;
}

void 
Orienteering::select(int tmax){
  int psize = 100, count = 0;
  int cur, before; 
  vector<vector<int> > path;
  vector<int> curpath;
  while(count++<psize){
    genetic_init();
    int t = 0;
    cur = start;
    curpath.push_back(cur);
    nodes[start].visited = true;
    while (t <= 0.5*tmax){
      vector<int> cur_neighbor;
      for (int i=0; i<nodes[cur].neighbors.size(); i++){
        if (!nodes[ nodes[cur].neighbors[i] ].visited)
          cur_neighbor.push_back(nodes[cur].neighbors[i]);
      }
      srand(time(NULL)+count);
      if (cur_neighbor.size()==0) break;
      int next = rand()%(cur_neighbor.size());
      before = cur;
      cur = cur_neighbor[next];
      curpath.push_back(cur);
      nodes[cur].visited = true;
      t += int(getDistance(cur, before));
    }
    if (t>0.5*tmax) {
      curpath.pop_back();
      nodes[cur].visited = false; 
    }
    int cursize = curpath.size();
    for (int i=cursize-2; i>=0; i--)
      curpath.push_back(curpath[i]);
    path.push_back(curpath);
    curpath.clear();
  }
  for (int i=0; i<path.size(); i++){
    for (int j=0; j<path[i].size(); j++)
      cout<<path[i][j]<<" ";
    cout<<endl;
  }


}

