/*
set other nodes (indegree=0) except source node v[0], score to be -1000000

*/
#include <dag.h>

void
Dag::createNodes(uint _size, uint _seed, double radius)
{
   nodes=RandomTopology(_size, BOUND_X, BOUND_Y, _seed, radius);   
//    cout<<"T: "<<T<<endl;
}

int 
Dag::getDistance(int i, int j){
	//important, must >=1, or it will lack a next 
	//add resolution important
	if (Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0)/resolution + 0.5 < 1){
		/*cout<<"dis: "<<i<<" "<<j<< " "<<nodes[i].pos.x()<<" "<<nodes[i].pos.y()<<" "<< Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0)<<endl;*/
		return 1;
		
	}
	return int(Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0)/resolution + 0.5); //important +0.5
    //si she wu ru, not sure yet, TODO
}

double
Dag::getRealDistance(int i, int j){
	return Distance(nodes[i].pos.x(), nodes[i].pos.y(), 0,
                  nodes[j].pos.x(), nodes[j].pos.y(), 0);
}
void 
Dag::init(){
	v.clear(); //init important!!! for multiple calculation
	per_res.clear();
	nodes[start].pos.x() = -49;
	nodes[start].pos.y() = 0;
//	nodes[end].pos.x() = 49;
//	nodes[end].pos.y() = 0;
	


	nodes[start].score = 0;
	nodes[end].score = 0;

}
void
Dag::init_Graph(){
	init();
	int n = nodes.size();
	vertex cur_v;
	int num = n*(T/resolution+1);
	cout<<n<<endl;
	cout<<resolution<<endl;
	cout<<(T+1)<<endl;
	cout<<num<<endl;
	dpath.resize(num);
	cout<<"dpath size: "<<dpath.size()<<endl;
	dpath[0].push_back(0); //dpath[id].push_back(index)
	for (int i=0; i<n; i++){
		cur_v.index = i;
		for (double t=0; t<=T+SMALL; t +=resolution){
			cur_v.id = i*(T/resolution+1) + t/resolution;
			cur_v.pos = nodes[i].pos;
			cur_v.t = t;	
			//cur_v.score = double((-t*t + t*T + T*T))/double(T*T); //TODO
			int x=cur_v.pos.x(), y=cur_v.pos.y();
		/*	if (x<0 && y>0){
				if (t<T/2)
					cur_v.score = 10*nodes[i].score;
				else
					cur_v.score = 1*nodes[i].score; 
			}else if(x<0 && y<0){
				if (t<T/2)
					cur_v.score = 1*nodes[i].score;
				else 
					cur_v.score = 5*nodes[i].score;
			}else if(x>0 && y>0){
				if (t<T/2)
					cur_v.score = 5*nodes[i].score;
				else 
					cur_v.score = 1*nodes[i].score;
			}else{
				if (t<T/2)
					cur_v.score = 1*nodes[i].score;
				else
					cur_v.score = 10*nodes[i].score;	
			}*/
		
			//cur_v.score = nodes[i].score * double(t)/double(T);
			//cur_v.score = nodes[i].score * double((-t*t + t*T + T*T))/(T*T);
			cur_v.score = scoreFunc(i, t);
			//nodes[i].score * exp(t)/T;
			//nodes[i].score * exp(-t)/T;
			cur_v.sum = NINF;
			cur_v.degree = 0;
			cur_v.parent = -1;
			//add next neighbor
				for (int k=0; k<n; k++){
					if (k==i || k==start) continue;  //important start node has no inner degree
					double next_t = t + getDistance(i, k)*resolution; //real t
					if (next_t <= T+SMALL && next_t > t){    //important next_t > t
						int next_id = k*(T/resolution+1) + next_t/resolution;
						cur_v.next.push_back(next_id);
					}

				}

			v.push_back(cur_v);
			cur_v.next.clear();
		}
	}

	//calculate in degree
	for (int i=0; i<v.size(); i++){
		for (int j=0; j<v[i].next.size(); j++){
			int next_id = v[i].next[j];
			v[next_id].degree++;
		}
	}
/*	cout<<"degree: "<<endl;
	for (int i=0; i<v.size(); i++)
		cout<<v[i].degree<<" ";*/

	//output next


/*	cout<<"01: "<<getDistance(0,1)<<endl;
	cout<<"02: "<<getDistance(0,2)<<endl;
	cout<<"01 real: "<<getRealDistance(0,1)<<endl;
	cout<<"02 real: "<<getRealDistance(0,2)<<endl;
	cout<<" ************next"<<endl;
	for (int i=0; i<v.size(); i++){
		cout<<v[i].index<<" "<<v[i].t<<": ";
		for (int j=0; j<v[i].next.size(); j++){
			cout<<v[v[i].next[j]].index<<"-"<<v[v[i].next[j]].t<<", ";
		}
		cout<<endl;
	}
*/
}


//Kahn's algorithm to get topological sort
vector<int>
Dag::topOrder(){
	vector<int> L;  //the sorted elements
	queue<int> S;
	v[0].sum = 0; //important, only source node [0][0] can be regarded as start, sum = 0
	for (int i=0; i<v.size(); i++)
		if (v[i].degree==0){
			S.push(i); 
		}
	cout<<"S size: "<<S.size()<<endl;
	while (!S.empty()){
		int cur_id = S.front();  
		S.pop();
		L.push_back(cur_id);
		vertex cur_v = v[cur_id];
		for (int i=0; i<cur_v.next.size(); i++){
			int next_id = cur_v.next[i];
			if (v[next_id].degree>0){
				v[next_id].degree--;
				if (v[next_id].degree==0)
					S.push(next_id);
			}
		}
	}

	return L;

}

vector<int>
Dag::MaximumPath(){
	vector<int> top;
	top = topOrder();
	for (int i=0; i<top.size(); i++){
		vertex cur_v = v[top[i]];
		for (int j=0; j<cur_v.next.size(); j++){
			int next_id = cur_v.next[j];
			//check distinct
			bool flag = true;
			for (int k=0; k<dpath[top[i]].size(); k++){
				if (v[next_id].index == dpath[top[i]][k]){
					flag = false;
					break;
				}
			}
	//		cout<<"flag: "<<flag<<endl;
			if (flag && v[next_id].sum < v[top[i]].sum + v[next_id].score){ //TODO <= or <
				v[next_id].sum = v[top[i]].sum + v[next_id].score;
				v[next_id].parent = top[i];
				dpath[next_id] = dpath[top[i]];
				dpath[next_id].push_back(v[next_id].index);
			}/*else if (flag && v[next_id].sum == v[top[i]].sum + v[next_id].score 
					       && v[next_id].parent !=-1 
					       && v[v[next_id].parent].t > v[top[i]].t){
				v[next_id].sum = v[top[i]].sum + v[next_id].score;
				v[next_id].parent = top[i];
				dpath[next_id] = dpath[top[i]];
				dpath[next_id].push_back(v[next_id].index);
				cout<<"da sha bi "<<endl;

			}*/
			//TODO 11/8
			
		}
	}
/*	int max_t=0, start_index = end*(T/resolution+1), max_id;
	double max_res=0;
	for (int i=start_index; i<=start_index+T/resolution; i++){
		if (v[i].sum>max_res) {
			max_res = v[i].sum;
			max_t = (i-start_index)*resolution;
			max_id = i;
		}
	}

	int t=max_t;  //TODO int->double
	int cur_id = max_id;
	vector<int> path, res;

	while (t!=0){
		path.push_back(v[cur_id].index);
		cur_id = v[cur_id].parent;
		t = v[cur_id].t;
	}
	path.push_back(start);
	cout<<endl;
	for (int i=path.size()-1; i>=0; i--){
		cout<<path[i]<<" ";
		res.push_back(path[i]);
	}
	cout<<endl;
	for (int i=0; i<res.size(); i++){
		cout<<nodes[res[i]].score<<" ";
	}
	cout<<endl;

	cout<<"distance between two nodes: "<<endl;
	for (int i=0; i<res.size()-1; i++)
		cout<<getDistance(res[i], res[i+1])*resolution<<" ";
	cout<<endl;

	double real_cost=0;
	for (int i=0; i<res.size()-1; i++)
		real_cost+=getRealDistance(res[i],res[i+1]);

	cout<<"max_res: "<<max_res<<endl;
	cout<<"max_t: "<<max_t<<endl;

	cout<<"real_max_res: "<<calculateScore(res)<<endl; 
	cout<<"real_cost: "<<real_cost<<endl;
	*/
	return findPath();
}

//find maximal collected score path
vector<int>
Dag::findPath(){
	int start_index = end*(T/resolution+1), max_id;
	double max_res=0, max_t;

	vector<int>  max_res_vec;
	for (int i=0; i<v.size(); i++){
		vector<int> path, res;		
		int cur_id = i;
		double time = v[i].t;
		while (time>SMALL){
			path.push_back(v[cur_id].index);
			cur_id = v[cur_id].parent;
			if (cur_id == -1) break;
			time = v[cur_id].t;
		}
		path.push_back(start);
		for (int i=path.size()-1; i>=0; i--){
			res.push_back(path[i]);
		}
		if (v[i].sum>max_res && calculateCost(res)<=T+SMALL) {
			max_res = v[i].sum;
			max_res_vec = res;
			max_t = v[i].t;
			max_id = i;
		}
	}
	cout<<"max_res: "<<max_res<<endl;
	cout<<"max_t: "<<max_t<<endl;

	cout<<"real_max_res: "<<calculateScore(max_res_vec)<<endl; //TODO
	cout<<"real_cost: "<<calculateCost(max_res_vec)<<endl;

		cout<<"distance between two nodes: "<<endl;
	for (int i=0; i<max_res_vec.size()-1; i++)
		cout<<getRealDistance(max_res_vec[i],max_res_vec[i+1])<<" ";
	cout<<endl;
	for (int i=0; i<max_res_vec.size(); i++)
		cout<<max_res_vec[i]<<" ";
	return max_res_vec;

}



void
Dag::numerate(vector<int>& flag, vector<int> nums, int level ){
    if (level==nums.size()){
        per_res.push_back(cur);
        return;
    }
    for (int i=0; i<nums.size(); i++){
        if (flag[i]==0){
            cur.push_back(nums[i]);
            flag[i]=1;
            numerate(flag, nums, ++level);
            level--;
            cur.pop_back();
            flag[i]=0;
        }
    }
    return;
}

void
Dag::permute(vector<int>& nums) {
    vector<int> flag(nums.size(),0);
    numerate(flag, nums,  0);
}

vector<int>
Dag::OptimizedSol(){

	cout<<"OptimizedSol: **************** "<<endl;
	int n=nodes.size();
	double maxs=0;
	vector<int> best;
	vector<int> vec;
	for (int i=1; i<n; i++)
		vec.push_back(i);
	permute(vec);

	vector<int> candidate;
	for (int i=0; i<per_res.size(); i++){
		candidate.clear();
		candidate.push_back(0);
		for (int j=0; j<per_res[i].size(); j++)
			candidate.push_back(per_res[i][j]);
		double sum = calculateScore(candidate);
		if (sum > maxs){
			maxs = sum;	
			best = candidate;
		}
	}
	cout<<"max score: "<<maxs<<endl;
	vector<int> res;
	double cost=0;
	for (int i=0; i<best.size(); i++){
//		if (best[i]!=end){
		res.push_back(best[i]);

		if (cost+getRealDistance(best[i],best[i+1])>T)
			break;
		else cost+=getRealDistance(best[i],best[i+1]);
//		}else break;

	}
//	res.push_back(end);
	cout<<"cost: "<<cost<<endl;

	cout<<"best route: "<<endl;	
	for (int i=0; i<res.size(); i++)
		cout<<res[i]<<" ";

	cout<<"distance between two nodes: "<<endl;
	for (int i=0; i<res.size()-1; i++)
		cout<<getRealDistance(res[i],res[i+1])<<" ";
//	res.push_back(end);

	//data for matlab draw 3d DAG
	cout<<"************"<<endl;
  	/*for (int i=0; i<v.size(); i++)
  		cout<<v[i].pos.x()<<" ";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		cout<<v[i].pos.y()<<" ";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		cout<<v[i].t<<" ";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		for (int j=0; j<v[i].next.size(); j++)
  			cout<<i+1<<" ";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		for (int j=0; j<v[i].next.size(); j++)
  			cout<<v[i].next[j]+1<<" ";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		cout<<"'["<<v[i].index<<"]["<<v[i].t<<"]',";
  	cout<<endl;
  	for (int i=0; i<v.size(); i++)
  		cout<<"'',";*/
	return res;
}

double
Dag::calculateScore(vector<int> path){
	double sum=0, t=0;
	for (int i=0; i<path.size()-1; i++){
		/*if (path[i]==end)
			break;*/
		
		sum += scoreFunc(path[i], t);
		t += getRealDistance(path[i], path[i+1]);
//		if (t>T) return -1;   //TODO, 11.17
		if (t>T) break;
	}
	if (t<=T) sum += scoreFunc(path[path.size()-1], t);
	return sum;
}

double
Dag::calculateCost(vector<int> path){
	double s=0;
	for (int i=0; i<path.size()-1; i++)
		s+=getRealDistance(path[i], path[i+1]);
	return s;
}	
double
Dag::scoreFunc(int i, int t){
	//return nodes[i].score;
	return nodes[i].score * double(t)/double(T);
	//return nodes[i].score * double((-t*t + t*T + T*T))/(T*T);
	//return nodes[i].score * log(t+1);
	//return nodes[i].score * exp(t)/T;
	//return nodes[i].score * exp(-t)/T;

}