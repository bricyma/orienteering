#include <iostream>
using namespace std;
int score1[5]={93, 73, 85, 20, 49};
int score2[4]={93, 85, 73, 49};
double t1[5]={13.4, 9.7, 8.0, 7.9, 4};
double t2[4]={13.4, 8.3, 8.0, 10.0};
double T=50;

double leftValue(int score, double t0){
	return 0.5*score*T*T - score*t0*(T-t0);
}
int main(){
	double recent1[10]={0}, recent2[10]={0};
	double sum1=0,sum2=0;
	for (int i=0; i<5;i++){
		if (i) recent1[i]=recent1[i-1]+t1[i];
		else recent1[i]=t1[i];

		if (i) recent2[i]=recent2[i-1]+t2[i];
		else recent2[i]=t2[i];
	}

	for (int i=0; i<5; i++){
		cout<<recent2[i]<<endl;
	}

	for (int i=0; i<5; i++){
		sum1+=leftValue(score1[i], recent1[i]);
		cout<<"1: "<<leftValue(score1[i], recent1[i])<<endl;
	}

	for (int i=0; i<4; i++){
		sum2+=leftValue(score2[i], recent2[i]);
		cout<<"2: "<<leftValue(score2[i], recent2[i])<<endl;
	}
	sum2+=0.5*20*T*T;
	cout<<"sum1: "<<sum1<<endl;
	cout<<"sum2: "<<sum2<<endl;
	return 0;
}