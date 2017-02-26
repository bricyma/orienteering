#include <string>
#include <iostream>
#include <cmath>
using namespace std;
int lcs(string source, string target){
	    int l1=source.length(), l2=target.length();
        int a[l1][l2];
        a[0][0] = source[0]==target[0]? 1:0;
        for (int i=1; i<l1; i++)
            if (source[i]==target[0])
                a[i][0]=a[i-1][0]+1;
            else
                a[i][0]=a[i-1][0];
        for (int i=1; i<l2; i++)
            if (source[0]==target[i])
                a[0][i]=a[0][i-1]+1;
            else
                a[0][i]=a[0][i-1];
        for (int i=1; i<l1; i++){
            for (int j=1; j<l2; j++){
                if (source[i]==target[j])
                    a[i][j]=a[i-1][j-1]+1;
                else
                    a[i][j]=a[i-1][j]>a[i][j-1]? a[i-1][j]:a[i][j-1];
            }
        }
        if (l1>=l2) return l1-a[l1-1][l2-1];
        else return l2-a[l1-1][l2-1];
}

int main(){
	string s1="bc",s2="ab";
	cout<<lcs(s1,s2)<<endl;

}