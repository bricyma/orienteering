
/******************************************************************************
* File: UtilFunc.h
* Description: Utility functions in aid of other complex funcitons.
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef UTIL_H
#define UTIL_H

#include <assert.h>
#include <vector>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include <sstream>
//#include <Eigen/Dense>
#include "defines.h"
#include "geometry_utils/Vector3.h"
#include "geometry_utils/Vector2.h"


using namespace std;
using namespace geometry_utils;

// convert Eigen matrix to 2d vectors, double type
//vector<vector<double> > EigenMat2Vecs(const Eigen::MatrixXd& m);


// calculate Euclidean distance
double Distance(double x1, double y1, double z1, double x2, double y2, double z2);

// to adjust the angle to be within (-PI, PI], so it's monotonous
double AngleNormalize(double _angle);

// import a matrix from file, copied the code from Assignment.h
vector<vector<double> > ImportMatrix(ifstream& input_file);

// negate matrix
void NegMatrix(vector<vector<double> >& _m);

// retrun a Gaussian distributed number, m=mean, s=standard deviation
double GaussianDistribution(double m, double s);

// method in class need to declare static to be a function object, for sorting
//inline 
bool LessThan(const pair<uint, double>&, const pair<uint, double>&);
bool GreaterThan(const pair<uint, double>&, const pair<uint, double>&);

template <typename T1, typename T2>
void AgentsWrapper(const vector<T1>& _as1, vector<T2>& _as2){

  if(!_as2.size())
    _as2.resize(_as1.size());

  assert(_as1.size()==_as2.size());
  for(unsigned int i=0; i<_as1.size(); i++){
    _as2[i].id = _as1[i].id;
    _as2[i].pos = _as1[i].pos;

    _as2[i].move = _as1[i].move;
    _as2[i].neighbors = _as1[i].neighbors;
  }//for

}

/*
// templated operation of copying a queue to a vector
template <typename T>
void CopyQ2Vec(const queue<T>& _q, vector<T>& _v){
  _v.clear();
  queue<T> q = _q;
  while(q.size()){
    _v.push_back(q.front());
    q.pop();
  }
  if(_v.size() != _q.size()){
    cerr<<"Error in copying queue to vector! "<<endl;
    exit(0);
  }
}
*/


#endif



