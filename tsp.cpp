#include "tsp.h"
#define MAXINT 1000000
using namespace std;

vector<int> assignment;
vector<vector<double> > mat;
//nearest neighbor initilization
//mat is cost matrix, ii=MAXINT
void loadMatrix(string matrix_file, int size){
	assignment.resize(size);
	mat.resize(size, vector<double>(size, 0));
	ifstream in(matrix_file.c_str());
	if (!in){
		cout<< "cannot open file"<<endl;
	}
	for (int i=0; i<size; i++){
		for (int j=0; j<size; j++){
			in>>mat[i][j];
		}
	}
	in.close();
}

void init(){
	int n=mat.size();
	bool flag[200];
	for (int i=0; i<200; i++){
		flag[i]=false;
	}
	flag[0]=true;
	int i=0,count=0,k;
	double min;
	while(count<n-1){
		min=MAXINT;
		for (int j=0; j<n; j++){
			if (!flag[j] && mat[i][j]<min && i!=j){
				min=mat[i][j];
				k=j;
			}
		}
		assignment[i]=k;
		flag[k]=true;
		i=k;
		count++;
	}
	assignment[i]=0;
	flag[0]=true;
}

//2 opt

void tsp_opt(){
	vector<uint> order;
	int m=0,n=1;
	while(n!=0){
	 	n=assignment[m];
	  	m=n;
	  	order.push_back(m);
	}
	bool changed=true;
	while(changed){
	    int ii=0,jj=0;
	    double min=0;
	    changed=false;
	    //different from tsp, Orienteering problem has static start and end point
	    for (int i=0; i<order.size()-2; i++){
	       for (int j=i+2; j<order.size()-1; j++){
	        int i2=(i+1),j2=(j+1);
	     
		        if (mat[order[i]][order[j]]+mat[order[i2]][order[j2]] -
		        	mat[order[i]][order[i2]]-mat[order[j]][order[j2]]<min){
		            min = mat[order[i]][order[j]]+mat[order[i2]][order[j2]] -
		        		  mat[order[i]][order[i2]]-mat[order[j]][order[j2]];
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

  	for (int i=0; i<order.size()-1; i++){
	  	assignment[order[i]]=order[i+1];
	}
	assignment[order[order.size()-1]]=order[0];
 
}


//tsp 
//input cost matrix, output assignment, it's a circle.
vector<int> tsp(string matrix_file, int size){
	loadMatrix(matrix_file,  size);
	
	init();
	tsp_opt();
	
	vector<int> path;
	int start=0;
	int count=0;
	int m=start;
	
	while(count++<size){
		path.push_back(m);
		m=assignment[m];
	}
	
	/*cout<<"path: "<<endl;
	for (int i=0; i<path.size(); i++){
		cout<<path[i]<<" ";
	}
	cout<<endl;*/
	return path; //for op.cpp
//	return assignment;   //for matlab draw
}

