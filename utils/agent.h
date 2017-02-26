/******************************************************************************
* File: agent.h
* Description: data structure for agent and its pose 
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef AGENT_H
#define AGENT_H

#include <set>
#include <queue>
#include <geometry_utils/Vector2.h>

typedef unsigned int AgtID;

//typedef geometry_utils::Vector2d point2d_t;  

typedef struct _pose {

  // in GL, here the height y is not used, x and z define robot's plane 
  double x;
  double y;
  double z;
  double a; //angle, in degree

} Pose;

typedef struct _agent {
  uint id;
  char state;
  Pose pose;
  std::set<uint> neighbors;
  std::deque<Pose> waypoints;  //to-go waypoints
} Agent;



#endif


