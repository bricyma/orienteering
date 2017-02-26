//https://vijos.org/p/1794
#include <iostream>
#include <stack>
#define MAX 1000000
using namespace std;
int N, K, M, S, T;
int c[101], a[101][101], pai[101],cost[101][101]={0};
bool flag[101]={0}, visited[101]={0}; //check culture is already visited.
void init(){
	cin>>N>>K>>M>>S>>T;
	for (int i=1; i<=N; i++){
		cin>>c[i];
	}
	//a[i][j]=1, paichi
	for (int i=1; i<=K; i++){
		for (int j=1; j<=K; j++){
			if (i==j) a[i][j]=1;
			else cin>>a[i][j];
		}
	}
	for (int i=1; i<=M; i++){
		int u,v,d;
		cin>>u>>v>>d;
		if (cost[u][v]==0 || cost[u][v]>d){
			cost[u][v]=d;
			cost[v][u]=d;
		}
		//keep only the smallest cost between u,v
	}
}



int main(){
	init();
	for (int k=1; k<=N; k++){
		for (int i=1; i<=N; i++){
			for (int j=1; j<=N; j++){
				if (!a[c[i]][c[j]]){
					cost[i][j]=min(cost[i][j], cost[i][k]+cost[k][j]);
				}
			}
		}
	}
	if (cost[S][T]<MAX) cout<<cost[S][T];
	else cout<<-1<<endl;
	return 0;

}