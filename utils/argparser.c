#include "argparser.h"
#include <string.h>

//char*      Model_File = (char*)"objs/create_model.obj";
size_t     RandSeed = 1;
size_t     Num_agents = 6;
float	   Radius = RADIUS;
float      Scale = 1;
double 	   Lambda = 0;
double 	   Weight = 0;
bool 	   Wireframe = false;
bool 	   Gui_off = false;
int	   StartID = 0;
int	   EndID = 1;

void argParser(int argc, char** argv){

    for (int i = 1; i < argc; i++) {
      if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "-input")) {
        i++; assert (i < argc);
        //Model_File = argv[i];
      } else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "-seed")) {
        i++; assert (i < argc);
        RandSeed = atoi(argv[i]);
      } else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "-size")) {
        i++; assert (i < argc);
        Num_agents = atoi(argv[i]);
      } else if (!strcmp(argv[i], "-r") || !strcmp(argv[i], "-radius")) {
        i++; assert (i < argc);
        Radius = atof(argv[i]);
      } else if (!strcmp(argv[i],"-scale")) {
        i++; assert (i < argc);
        Scale = atof(argv[i]);
      } else if (!strcmp(argv[i], "-l") || !strcmp(argv[i], "-lambda")) {
        i++; assert (i < argc);
        Lambda = atof(argv[i]);
      } else if (!strcmp(argv[i], "-w") || !strcmp(argv[i], "-weight")) {
        i++; assert (i < argc);
        Weight = atof(argv[i]);
      } else if (!strcmp(argv[i], "-start") ) {
        i++; assert (i < argc);
        StartID = atoi(argv[i]);
      } else if (!strcmp(argv[i], "-end") ) {
        i++; assert (i < argc);
        EndID = atoi(argv[i]);
      } else if (!strcmp(argv[i],"-wireframe")) {
        //i++; assert (i < argc); // just a flag, no value followed
        Wireframe = true;
      } else if (!strcmp(argv[i], "-g") || !strcmp(argv[i], "-guioff")) {
        //i++; assert (i < argc);
        Gui_off = true;
      } else {
        printf ("whoops error with command line argument %d: '%s'\n",i,argv[i]);
        assert(0);
      }
    }

}


