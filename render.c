#include "render.h"
#include "PathPlanning.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

// ascii codes for various special keys 
#define ESCAPE 27
#define PAGE_UP 73
#define PAGE_DOWN 81
#define UP_ARROW 72
#define DOWN_ARROW 80
#define LEFT_ARROW 75
#define RIGHT_ARROW 77


//GLMmodel*  model_robot;
//GLMmodel*  model_cone;
//GLuint model_list_robot, model_list_cone;

GLfloat smoothing_angle = 90.0;
GLfloat orig_scale;

//static GLfloat floorPlane[4];
//static GLfloat floorShadow[4][4];

bool _Pause_flag = true;
int flag_draw = 1;

GLfloat f_w = 100; //floor width
static GLfloat floorVertices[4][3] = {
  { -f_w, 0.0, f_w},
  { f_w, 0.0, f_w},
  { f_w, 0.0, -f_w},
  { -f_w, 0.0, -f_w},
};

GLuint texture2[2][3]; // storage for textures, each 3 modes
//SECTOR  sec;      // sec for walls

//defined in paths.h
extern vector<Agent> AgtsAll;
extern vector<Agent*> AgtsAllPtrs; //ptr to strictly ordered agts, 0, 1, 2, ..n-1
extern vector<Agent*> AgtsSpecial; // selected set for scaling weights


Orienteering OP;
Dag DAG;
double T=100;
vector<real_vertex> nodes;
vector<node_t> nodes_copy;
vector<int> endnodes;
using namespace viz_tool;

vector<vector<unsigned int>> order2;
vector<vector<int>> order;
vector<int> order_op0; //gravity heuristic based op
vector<int> order_op;  //dijkstra based op
vector<int> order_op3; //traditional op
vector<vector<int> > order_op2;
vector<double> curscore; //score(t)
vector<double> routeState; //store the distance;
clock_t t_start;
double cur_t;
double average0=0, average1=0;
int remain0=0, remain1=0; //remain0,my op;   remain1, traditional op
void drawAll(void) {
  //TODO next edition 12/18/2016
  //draw those special nodes, in cube
  for (uint i = 1; i < nodes.size(); i++) {
    glPushMatrix();
    //glTranslatef(nodes[i].pos.x(), -0.2, nodes[i].pos.y());
    glTranslatef(nodes[i].pos.x(), nodes[i].pos.y(), 0);
    glColor3f(0.5, 0.5, 1);
    glutSolidSphere(0.015 * nodes[i].score, 20, 20);   //last edition without time variant score
//    glutSolidSphere(0.01 * curscore[i], 20, 20);
    // gluCylinder(quadratic,5.0f,0.1f,3.0f,32,32);
    //glTranslatef(4,0,12);
    //glColor3f(0.5, 0.5, 1);
   
    //glScalef(1,1,5);
    //glutWireCube(5);
    glPopMatrix();
  }

  //TODO next edition 12/18/2016
/*  for (uint i = 1; i < nodes.size(); i++) {
    glPushMatrix();
    //glTranslatef(nodes[i].pos.x(), -0.2, nodes[i].pos.y());
 
      glColor3f(0.5, 0.5, 1);
      glTranslatef(nodes[i].pos.x()+4, nodes[i].pos.y(), 5+6*nodes[i].score*double(60.7)/double(150)/double(20));
    
      glColor3f(0.5, 0.5, 1);
     
      glScalef(1,1,5*nodes[i].score*double(60.7)/double(150)/double(20));
      glutSolidCube(5);
      glPopMatrix();

    
  }
*/

  //starting points
  for (unsigned int i = 0; i < endnodes.size(); i++) {
    glPushMatrix();
    //glTranslatef(nodes[i].pos.x(), -0.2, nodes[i].pos.y());
    glTranslatef(nodes[i].pos.x(), nodes[i].pos.y(), 0);
    glColor3f(1, 0.5, 0.5);
    int size = time(NULL)%10;
//    cout<<size<<endl;
    glutSolidSphere(0.2*size, 20, 20);
    glLineWidth(3);
    drawCircle(1.8, 20, false);
    glPopMatrix();
  }

  //goal points
  for (unsigned int i = 0; i < endnodes.size(); i++) {
    glPushMatrix();
    glTranslatef(nodes[endnodes[i]].pos.x(), nodes[endnodes[i]].pos.y(), 0);
    glColor3f(0.5, 1, 0.5);
    glutSolidSphere(1, 20, 20);
    drawSquare(3.6, false);
    glPopMatrix();
  }

  // vector<node_t> nodes = OP.getNodes();
  for (uint i = 0; i < nodes.size(); i++) {
    glPushMatrix();
    char text[4];
//        sprintf(text, "%d", int(nodes[i].score));
    sprintf(text, "%d", int(i));
    glTranslatef(nodes[i].pos.x() + 1, nodes[i].pos.y() + 1, 0); //0
    glutPrint(0, 0, glutFonts[4], text, 0.f, 0.f, 0.f, 1.0f);
    glPopMatrix();

  }

//draw link edge
/*  glEnable(GL_COLOR_MATERIAL);
  glColor3f(1.0f, 1.0f, 0.1f);
  for (uint i = 0; i < nodes.size(); i++) {
    for (uint j = 0; j < nodes[i].neighbors.size(); j++){
      draw2DThickLine(nodes[i].pos, nodes[ nodes[i].neighbors[j]].pos, 0.05);
    }
  }*/
//  cout<<"remain score: "<<cur_t<<" "<<OP.getRemainScore(cur_t, T)<<endl;
  double cur_score0 = 0; //OP.getRemainScore(cur_t, T, 0);
  double cur_score1 = 0; //OP.getRemainScore(cur_t, T, 1);
  remain0+=int(cur_score0);
  remain1+=int(cur_score1);
//  cout<<remain<<endl;
  average0 = remain0/(cur_t*10);
  average1 = remain1/(cur_t*10);
//  cout<<"time: "<<cur_t<<" average: my op: "<<average0<<" traditional op: "<<average1<<endl;
/*  glPopMatrix();
  char text[4];
  sprintf(text, "%d", int(OP.getRemainScore(cur_t, T)));
  glTranslatef( nodes[1].pos.x() + 1, nodes[1].pos.y() + 1, 0);
  glutPrint(0, 0, glutFonts[4], text, 0.f, 0.f, 0.f, 1.0f);
  glPopMatrix();*/

  //_Pause_flag = false;

}

void
drawTopo1(void) {

  // draw paths
  glEnable(GL_COLOR_MATERIAL);
  glColor3f(1.0f, 0.0f, 0.0f);
  for (uint i = 0; i < order_op0.size()-1; i++) {
    draw2DThickLine(nodes[order_op0[i]].pos, nodes[order_op0[i+1]].pos, 0.3);
  }
}

void
drawTopo2(void) {
  // draw paths
  glEnable(GL_COLOR_MATERIAL);
  //op's algorithm
  glColor3f(0.0f, 0.9f, 0.1f);
  for (uint i = 0; i < order_op.size()-1; i++) {
      draw2DThickLine(nodes[order_op[i]].pos, nodes[order_op[i+1]].pos, 0.2);
  }
}



void
drawTopo3(void) {
  // draw paths
//  double cur_time = (clock()-t_start)/100000;
  glEnable(GL_COLOR_MATERIAL);
  glColor3f(0.1f, 1.0f, 0.1f);

//  double current = (clock()-t_start)/(100/88*60000)/10.0;  //100000
//  cout<<current<<endl;
  
  for (uint i = 0; i < order_op3.size()-1; i++) {
    draw2DThickLine(nodes[order_op3[i]].pos, nodes[order_op3[i+1]].pos, 0.3);
  }
/*  for (uint i = 0; i < order_op.size()-1; i++) {
      if (current>routeState[i] && current<=routeState[i+1]){      
        point2d_t start_node = nodes[order_op[i]].pos;
        point2d_t end_node = nodes[order_op[i+1]].pos;
        double percent = (current-routeState[i])/(routeState[i+1]-routeState[i]);
        point2d_t curpos = start_node + (end_node - start_node) * percent; 
        draw2DThickLine(start_node, curpos, 0.3);
      }
  }*/

}

void
display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glEnable(GL_COLOR_MATERIAL);

  // sky color, dark sky needs a better translucent floor
  glClearColor(1.0, 1.0, 1.0, 1.0); // white
  //glClearColor(0.1, 0.1, 0.1, 1.0); // dark

  //glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT0);

  //Tell GL new light source position to create shading, for darker shadow!
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  //glutSolidSphere(2, 30, 30); //test light

  glPushMatrix();
  //glTranslatef(pan_x, pan_y, 0.0);
  glTranslatef(pan_x, pan_y, pan_z);

  gltbMatrix();

  //draw the real objects
  if (flag_draw == 1)
    drawTopo1();
  else if(flag_draw == 2 )
    drawTopo2();
  /*else
    drawTopo3();*/
  drawAll();

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.9, 1.0, 1.0, 0.5);
  glPushMatrix();
  glRotatef(90, 1.0, 0.0, 0.0);
  glTranslatef(0, -1, 0);
  drawFloor(floorVertices, 16);
  glPopMatrix();
  glDisable(GL_BLEND);

  glPopMatrix();

  glutSwapBuffers();
}

void
thiskey(unsigned char key, int x, int y) {
  // avoid thrashing this procedure 
  usleep(100);

  switch (key) {
    case ESCAPE: 
      exit(1);
      break;// kill everything.
    case 'a':
      flag_draw = 1;
      break;
    case 'b':
      flag_draw = 2;
      break;
    case 'c':
      flag_draw = 3;
      break;
    case 'q':
    case 'Q':
      exit(1);
      break; // redundant.
    case 'p':
    case 'P':
      _Pause_flag = !_Pause_flag;
      break;
    default:
      printf("Key %d pressed. No action there yet.\n", key);
      break;
  }

}

void
idle(void) {
  display();
  glutPostRedisplay();
}

int
render(int argc, char** argv) {
  remain0=0;
  remain1=0;
  double v = 1.0;
  int StartID = 0, EndID = 1, node_num = ::Num_agents;
  DAG.setStartEnd(StartID, EndID);
  DAG.setT(150);
  DAG.setResolution(1);
  DAG.createNodes(node_num, ::RandSeed, 200.0);
  
  DAG.init_Graph();
  nodes = DAG.getNodes();
  order_op0 = DAG.MaximumPath();
//  order_op0.clear();    
//  order_op = DAG.OptimizedSol();   //optimized solution

  double r[6]={0.1,0.5,1.0,2,5,10};
/*
//statistics of resolution
  vector<double> our_sum;
  for (int i=0; i<6; i++){
    double s=0;
    for (int j=0; j<20; j++){
      DAG.setResolution(r[i]);
      cout<<r[i]<<endl;
      DAG.createNodes(50, j*10, 200);
      DAG.init_Graph();
      order_op0 = DAG.MaximumPath();
      nodes = DAG.getNodes();
      s += DAG.calculateScore(order_op0);
    }
    our_sum.push_back(s/10);
  }
  cout<<"our sum: "<<endl;
  for (int i=0; i<our_sum.size(); i++)
    cout<<our_sum[i]<<" ";
  cout<<endl;
//for loop, calculate average
  double s=0;
  vector<int> cur_path;
  vector<double> time_cost;
  vector<double> ours_sum;
  vector<vector<int> > end_node;*/

//compare with op
/*  end_node.resize(195);
  for (int i=6; i<=200; i++){
    s=0;
    for (int j=0; j<5; j++){
      DAG.createNodes(i, j, 200);
      DAG.init_Graph();
      nodes = DAG.getNodes();
      order_op0 = DAG.MaximumPath();
      s+=DAG.calculateScore(order_op0);
      end_node[i-6].push_back(order_op0[order_op0.size()-1]);
    }
    ours_sum.push_back(s/5);  
  }

  for (int i=0; i<ours_sum.size(); i++)
    cout<<ours_sum[i]<<" ";
  cout<<"end node: "<<endl;
  for (int i=0; i<195; i++){
    for (int j=0; j<5; j++){
      cout<<end_node[i][j]<<" ";
    }
    cout<<endl;
  }*/
/*  
  //resolution
  for (int i=0; i<5; i++){
    DAG.createNodes(node_num, i, 200);
    DAG.init_Graph();
    nodes = DAG.getNodes();
    order_op0 = DAG.MaximumPath();
    s+=DAG.calculateScore(order_op0);
  }
  cout<<"average score: "<<s/5<<endl;*/

/*  for (int i=7; i<200; i++){
    clock_t begin_t = clock();
 
    DAG.createNodes(i, i, 200.0);//size=node_num, _seed=RandSeed, radius=200.0
    DAG.init_Graph();
    nodes = DAG.getNodes();
    cur_path = DAG.MaximumPath();
    //cur_path = DAG.OptimizedSol();
//    ours_sum.push_back(DAG.calculateScore(cur_path));
//    s+= DAG.calculateScore(cur_path);
    clock_t end_t = clock();
 
    time_cost.push_back(double(end_t-begin_t)/ CLOCKS_PER_SEC);
  }
  cout<<"total sum: "<<s<<endl;
  cout<<"average: "<<s/50<<endl;
//  cout<<"time cost: "<< double(end_t-begin_t)/ CLOCKS_PER_SEC<<endl;
  
  for(int i=0; i<time_cost.size(); i++)
    cout<<time_cost[i]<<" ";

  order_op0.push_back(0);*/
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL | GLUT_MULTISAMPLE);

  glutInitWindowSize(1024, 1024);
  glutCreateWindow("Simu");
  canvasInit();
  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(thiskey);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  /*
    GLsizei wh2d = 500; // initial height of window
    GLsizei ww2d = 500; // initial width of window
    int Win1, SubWin;

    //main window
    glutInitWindowSize( ww2d, wh2d ); // window size
    glutInitWindowPosition ( 0, 0 ); // & position on screen
    Win1 = glutCreateWindow( "Main" );
    glutSetWindow(Win1);
    canvasInit();
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(thiskey);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    makeCheckerFloorTexture();

    //sub window
    glutInitWindowPosition (ww2d+10, 0); // & position on screen
    SubWin = glutCreateWindow("Sub");
    glutSetWindow(SubWin);
    canvasInit();
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(thiskey);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
   */

  makeCheckerFloorTexture();
  //makeCircleFloorTexture();

  glutIdleFunc(idle);

  glutMainLoop();
  return 0;
}



/*
void moveAgts(Agent* a, float velocity){

  Pose next_pose = a->waypoints.front();
  double orientation = atan2(next_pose.z-a->pose.z, next_pose.x-a->pose.x);
  //update the agent to face the moving direction
  a->pose.a =180 -(orientation*180/PI - 90); //why this? atan?
  if(sqrt((a->pose.x-next_pose.x)*(a->pose.x-next_pose.x)+(a->pose.z-next_pose.z)*(a->pose.z-next_pose.z))>velocity ){
    a->pose.x += velocity*cos(orientation);
    a->pose.z += velocity*sin(orientation);
  }
  else{
    //delete this waypoint
    a->waypoints.pop_front();
    //update next pose
    next_pose = a->waypoints.front();
  }

}



void moveAgts(Agent& a, float velocity, float orientation, Pose goal){

  if(sqrt((a.pose.x-goal.x)*(a.pose.x-goal.x)+(a.pose.z-goal.z)*(a.pose.z-goal.z))>velocity ){
    a.pose.x += velocity*cos(orientation);
    a.pose.z += velocity*sin(orientation);
  }
  else{
    //delete this waypoint
    a.waypoints.pop_front();
  }

}
 */
