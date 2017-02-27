#include <data_collect.h>
int test_collect = 33;


vector<real_vertex> data_collect(){
	ifstream infile("/home/bricy/Desktop/lab/gps_pos.txt");
	
	if (!infile){
		cout<<"wrong file"<<endl;
	}

	test_collect = 44;
	cout<<"vertex collecting>>>>>>>>>>>>>>>>>>"<<endl;

	int index, hour, min, x, y, s;
	int node_num = 50;
	int count = 0;
	vector<real_vertex> vec_v;
	
	while (infile >> index >> x >> y){
		real_vertex v; 
		v.id = index;
		double tx = x/400;
		double ty = y/400;
		v.pos.x() = tx-5;
		v.pos.y() = ty+10;
		v.score = 50;
	    v.visited = false;
		vec_v.push_back(v);	
	}
	vec_v[32].pos.x()+=10;
	return vec_v;
}


vector<vector<vector<int> > > collect_score(){
	ifstream infile("/home/bricy/Desktop/lab/gps_data.txt");
	
	if (!infile){
		cout<<"wrong file"<<endl;
	}

	test_collect = 44;
	cout<<"score func collecting>>>>>>>>>>>>>>>>>>"<<endl;

	int index, hour, min, x, y, s;
	int node_num = 50;
	int count = 0;
	vector<real_vertex> vec_v;
	vector<vector<vector<int> > > taxi_score_func;
	vector<vector<int> > score_hour;
	vector<int> score_min;
	for (int i=0; i<60; i++)
		score_min.push_back(0);
	for (int i=0; i<24; i++)
		score_hour.push_back(score_min);
	for (int i=0; i<node_num; i++)
		taxi_score_func.push_back(score_hour);

	while (infile >> index >> hour >> min >> x >> y >> s){
		taxi_score_func[index][hour][min] = s;
		cout<<s<<endl;
	}
	cout<<"score func collected"<<endl;
	return taxi_score_func;
}