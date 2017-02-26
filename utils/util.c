
#include "util.h"

/*
vector<vector<double> > 
EigenMat2Vecs(const Eigen::MatrixXd& m){

  vector<vector<double> > _m(m.rows());
  for(uint i=0; i<m.rows(); i++){
    for(uint j=0; j<m.row(i).size(); j++)
	_m[i].push_back(m(i, j));
  }

  return _m;
}
*/


double Distance(double x1, double y1, double z1, double x2, double y2, double z2){

  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );

}

double AngleNormalize(double _angle){
  double angle = _angle;
  if(_angle < -PI)
    angle = _angle + 2*PI;
  if(_angle >= PI)
    angle = _angle - 2*PI;
  return angle;
}


vector<vector<double> >
ImportMatrix(ifstream& input_file){

  vector<vector<double> > matrix;
  string line;
  vector<double> numstream;

  size_t num_rows = 0;
  size_t num_cols = 0;

  if (input_file.is_open())
  {
    while (!input_file.eof() )
    {
      getline (input_file,line);

      size_t local_num_cols=0;
      vector<double> local_numstream;
      local_numstream.clear();
      string word;

      stringstream parse(line);
      while(parse >> word){
	//if comment line, ignore it
	if(word[0] == '#')
	  break;
	//numstream.push_back(atoi(word.c_str()));  //if not number, then convert to zero
	numstream.push_back(atof(word.c_str()));  //if not number, then convert to zero
	//local_numstream.push_back(atoi(word.c_str()));
	local_numstream.push_back(atof(word.c_str()));

	//matrix(num_rows, num_cols)= atoi(word.c_str()); 	
	local_num_cols++;
      } //end inner while

      //judge if the matrix format is correct or not
      if(num_cols && local_num_cols && num_cols!=local_num_cols){
	cerr<<endl<<"Please input a correct matrix format!"<<endl<<endl;
	exit(-1);
      }
      //update column number if everything looks normal
      if(local_num_cols)
	num_cols = local_num_cols;
      //update row number if everything looks normal
      if(line.length()&&local_numstream.size())
        num_rows++;

    } //end out layer while

    input_file.close();

    //update class data members
    //this->num_agents = num_rows;
    //this->num_tasks = num_cols;

    //put elements into matrix
    //matrix.resize(num_rows, num_cols);
    matrix.resize(num_rows);
    for(unsigned int i=0; i<num_rows; i++)
      matrix[i].resize(num_cols);

    vector<double>::iterator itr = numstream.begin();
    for(unsigned int i=0; i<num_rows; i++)
      for(unsigned int j=0; j<num_cols; j++)
	matrix[i][j] = *itr++;
  } //end outmost if
  else{ 
    cerr <<endl<<"Error: Unable to open file! Stopped."<<endl<<endl; 
    exit(-1);
  }

  return matrix;
}


void 
NegMatrix(vector<vector<double> >& _m){

  for(uint i=0; i<_m.size(); i++)
    for(uint j=0; j<_m[i].size(); j++)
      _m[i][j] = - _m[i][j]; 

}



/*Modified from code of Everett F. Carter Jr.*/
double
GaussianDistribution(double m, double s)      
{                                       // mean m, standard deviation s 
        double x1, x2, w, y1;
        static double y2;
        static int use_last = 0;

        if (use_last)                   // use value from previous call 
        {
                y1 = y2;
                use_last = 0;
        }
        else
        {

//   rand() generates integers in the range [0,RAND_MAX] inclusive therefore, (double)rand())/RAND_MAX returns a double-point number in [0,1], then multiply it by 2 and add it to -1, we get [-1, 1], uniformly.
                do {

                        x1 = -1 + 2*((double)rand())/RAND_MAX;
                        x2 = -1 + 2*((double)rand())/RAND_MAX;
                        w = x1 * x1 + x2 * x2;
                } while ( w >= 1.0 );

                w = sqrt( (-2.0 * log( w ) ) / w );
                y1 = x1 * w;
                y2 = x2 * w;
                use_last = 1;
        }

        return( m + y1 * s );
} 
//inline 
bool
LessThan(const pair<uint, double>& _a, const pair<uint, double>& _b){
  return (_a.second < _b.second);
}


bool
GreaterThan(const pair<uint, double>& _a, const pair<uint, double>& _b){
  return (_a.second > _b.second);
}


