/******************************************************************************
* File: render.h
* Description: rendering all on screen 
* Author: Lantao Liu
* Date: 6/2010; Last update: 2013
******************************************************************************/

#ifndef RENDER_H
#define RENDER_H

#include "viz_tool/canvas.h"
#include "viz_tool/shadow.h"
#include "viz_tool/floor.h"
#include "viz_tool/gldraw.h"
#include "viz_tool/glfunc.h"
#include "utils/util.h"
#include "utils/topology.h"
#include "paths.h"
#include "op.h"
#include "dag.h"
#include <set>
#include <ctime>

using namespace std;
using namespace viz_tool;

//for view/camera position
extern GLdouble viz_tool::pan_x,
                viz_tool::pan_y,
                viz_tool::pan_z;

extern char*      Model_File;


void drawTopo1(void);
void drawTopo2(void);


void drawAll();

void display(void);

void idle(void);

int render(int argc, char** argv);

#endif


