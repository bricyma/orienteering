#include <algorithm>
#include <vector>
#include "tsp.h"

using namespace std;
typedef struct Node {
  uint id;
  double score;
} node_t;

bool acompare(node_t a, node_t b){return a.score<b.score;}
int main(){
	/*ofstream output("tsp2.txt"); 
	for (int i=0; i<12; i++){
		for (int j=0; j<12; j++){
			output << i+j << " ";
		}
		output<<endl;
	}*/
	vector<int> path;
/*	path=tsp("tsp.txt", 15);
	for (int i=0; i<15; i++){
		cout<<path[i]<<" ";
	}*/
/*	vector<node_t> node;
	node_t t;
	int a[5]={5,4,3,2,1};
	for (int i=0; i<5; i++){
		t.score=a[i];
		t.id=i;
		node.push_back(t);
	}
	sort(node.begin(), node.begin()+5, acompare);

	for (int i=0; i<node.size(); i++){
		cout<<node[i].id<<endl;
	}
		*/
//vector insert
/*	int a[5]={5,3,2,1,0};
	vector<int> b(a,a+5);
	b.insert(b.begin()+5, 4);
	for (int i=0; i<b.size(); i++){
		cout<<b[i]<<" ";
	}*/
	int a[2]={1,2};
	int b[3]={1,2,3};
	int c[4]={1,2,3,4};
	vector<int> p1(a,a+2);
	vector<int> p2(b,b+3);
	vector<int> p3(c,c+3);
	path = p3;
	path = p2;
	path = p1;
	return 0;

}