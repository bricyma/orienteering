/******************************************************************************
* File: define.h
* Description: global definitions and macros 
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef DEFINE_H
#define DEFINE_H

#define _LINUX_OS_

//#define _MAC_OS_

#define PI 3.1415926
#define INF 10e7 
#define EPSILON 1e-5
const  double RADIUS = 12;

const  double BOUND_X = 50;
const  double BOUND_Y = 50;
//const  double BOUND_Z = 27;

typedef unsigned int uint;

enum STATES{BODY='a', START, GOAL, REMOVED}; //REMOVED means finished goals

#endif

