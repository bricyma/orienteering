/******************************************************************************
* File: paths.h
* Description: main file for computing the paths
* Author: Lantao Liu
* Date: 6/2010; Last update: 2014
******************************************************************************/

#ifndef PATHS_H
#define PATHS_H

#include "viz_tool/canvas.h"
#include "viz_tool/shadow.h"
#include "viz_tool/floor.h"
#include "viz_tool/gldraw.h"
#include "utils/util.h"
#include "utils/topology.h"
#include <set>


using namespace std;

extern size_t     RandSeed; 
extern size_t     Num_agents; 
extern GLfloat    Scale;
extern double   Weight;


//functions

void agentImportInit(void);
void agentImportInit2(void);
void agentRandomInit(void);

// after only body is pulled out and put into matrix, agent id vs matrix id match 
uint mapMatrixIndex2BodyAgts(vector<Agent*>& _agts, Agent* _a);
// compute the total lengths of path segments for the last relaying robot
double lengthLastRelay(vector<vector<AgtID> >& _paths, vector<Agent*>& _agts_special);
double avgLengthWithRelay(vector<vector<AgtID> >& _paths, vector<Agent*>& _agts_special);


#endif


