#include <data_collect.h>

void data_collect(){
	ifstream infile("/home/bricy/Desktop/lab/gps_data.txt");
	// hour_pickup	min_pickup	weekday_pickup	center_long	center_lat	passenger_count
	double hour_pickup, center_lat, center_long, passenger_count;
	double min_pickup, weekday_pickup;
	
	vector<real_vertex> nodes;
	int count = 0;
	cout<<"fffffffffff"<<endl;
	if (!infile){
		cout<<"wrong file"<<endl;
	}

	vector<real_vertex> output;
	string csvLine;
    // read every line from the stream
    // while( getline(infile, csvLine) )
    // {
    //     istringstream csvStream(csvLine);
    //     vector<string> csvColumn;
    //     string csvElement;

    //     real_vertex v;
    //     int count = 1;
    //     while( getline(csvStream, csvElement, ',') ){
    //     	switch (count) {
	   //      case 1: v.id = 
	   //      case 2: v.
	   //      case 3: std::cout << "3";
	   //      case 4:
	   //      case 5: std::cout << "45";
	   //              break;              //execution of subsequent statements is terminated
	   //      case 6: std::cout << "6";
    // }
    //     	if (count==0):
    //         csvColumn.push_back(csvElement);
    //         cout<<stod(csvElement)<<endl;
    //     }
    //     output.push_back(csvColumn);
    // }
    // cout<<output.size()<<endl;
    // cout<<output[0].size()<<endl;

	// gps_file.write('%d %d %d %d %d\n' %(i, j, center_x, center_y, s))


	// int index, t, s, x, y;
	// while (infile >> index >> t >> x >> y >> s){
	// 	real_vertex v; 
	// 	v.id = count++;
	// 	double er = 6378137.0;
	// 	double tx = x;
	// 	double ty = y;
	// 	v.pos.x() = tx;
	// 	v.pos.y() = ty;
	// 	v.score = s;
	// 	cout<<"count: "<<count<<endl;
	// }


}