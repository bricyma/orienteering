/******************************************************************************
* File: argparser.h
* Description: command parser.
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef ARGPARSER_H
#define ARGPARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "defines.h"

//class ArgParser{
//public:

//extern char*    Model_File;
extern size_t   RandSeed;
extern size_t   Num_agents;
extern float    Radius;
extern float    Scale;
extern double 	Lambda;
extern double 	Weight;
extern bool 	Wireframe;
extern bool 	Gui_off;
extern int	StartID;
extern int	EndID;

void argParser(int argc, char** argv);

//};


#endif

