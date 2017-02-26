
#include "assignment/CmdParser.h"
#include "assignment/Assignment.h"
#include "render.h"
#include "op.h"
#include <pthread.h>

using namespace std;
int main(int argc, char** argv)
{

  int i=pthread_getconcurrency();
  /*
   Above line is useless but just fix the seg-fault caused by nvidia driver, see 
   https://bugs.launchpad.net/ubuntu/+source/nvidia-graphics-drivers-319/+bug/1248642 
  */

  //define a matrix;
  Matrix m;

  //parse command line and generate an assignment
/*  CmdParser parser(argc, argv);
  parser.DisplayInput();

  Assignment as;
  if(parser.GetInputFileName().empty()){
    if(parser.GetSeed())
      as.SetSeed(parser.GetSeed());
    else
      as.SetSeed(time(NULL));
    cout<<endl<<"  *Seed for random generator: "<<as.GetSeed()<<endl;
    m=as.RandomGenerate( parser.GetAssignmentSize(), 
       parser.GetAssignmentSize(), 
       MAX_RANDOM, 
       parser.GetSeed() );
  }
  else{
    ifstream myfile(parser.GetInputFileName().c_str());
    m=as.ImportAssignment(myfile);
  }
  as.DisplayMatrix(m);*/

  //Orienteering(m);

  argParser(argc, argv);

 // agentRandomInit();
  render(argc, argv);

  return 0;
}

