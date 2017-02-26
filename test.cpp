#include <iostream>
#include <vector>
using namespace std;
//pointer
//change the content in address
void swap(int *a, int *b){
	int temp=*a;
	*a=*b;
	*b=temp;
}
//reference
//change the value
void swap2(int &a, int &b){
	int p=a;
	a=b;
	b=p;
}
int main(){
	int a=1,b=2;
	cout<<&a<<" "<<&b<<endl;
	swap(&a, &b);
	cout<<&a<<" "<<&b<<endl;

	int c=1,d=2;
	cout<<c<<" "<<d<<endl;
	swap2(c, d);
	cout<<c<<" "<<d<<endl;
	/*int c=1;
	int *a;
	a=&c;
	int *&p=a;
	cout<<a<<" "<<*a<<endl;
	cout<<p<<" "<<*p<<endl;*/
	return 0;
	
}