/*
This code simply generates a point cloud for a rectangular shaped
corridor with closed ends. It uses PCL to internally represent the
structure and finally saves it to a '.pcd' file to be later loaded
from other nodes.

*/

#include<cmath>
#include<string>
#include<cstdlib>
#include<iostream>
#include<algorithm>

#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/file_io.h>

using namespace std;

int main(int argc, char* argv[]){
	// - Get the dimensions of the corridor.
	// - output file name
	string output_path, format, comp;
	double diameter, inclination, length, res;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	if(argc != 8){
		cout << "Usage: generate_tunnel <diameter> <inclination in deg> <length> <resolution> <output.pcd> <format> <compression>" << endl;
		cout << "<format> is one of : (P)CD or P(L)Y" << endl;
		cout << "<compression> is one of : '(a)scii', (b)inary, 'binary-(c)ompressed'" << endl;
		exit(-1);
	}

	//for(int i = 0 ; i < argc ; i++)
	//	cout << "argv[" << i << "] : " << argv[i] << endl;

  diameter      = atof(argv[1]);
	inclination   = atof(argv[2]);
	length        = atof(argv[3]);
	res           = atof(argv[4]);
	output_path   = string(argv[5]);
	format        = string(argv[6]);
	comp          = string(argv[7]);
	std::transform(format.begin(), format.end(), format.begin(), ::tolower);
	std::transform(comp.begin()  , comp.end()  , comp.begin()  , ::tolower);

	assert(format == "p" || format == "l");
	assert(comp   == "a" || comp   == "b" || comp == "c");

	if(format == "l"){
		cout << "PLY file format is not supported. Exiting." << endl;
		exit(-1);
	}

	cout << ">> diameter    : " << diameter    << endl;
	cout << ">> inclination : " << inclination << endl;
	cout << ">> length      : " << length      << endl;
	cout << ">> res         : " << res         << endl;
	cout << ">> output_path : " << output_path << endl;
	cout << ">> format : <" << format << ">" << endl;
	cout << ">> compression : <" << comp << ">" << endl;

  double radius = diameter / 2;
  double dth = acos(1 - res * res / (2 * radius * radius));
  double PI = 3.14159265359;
  double c  = cos(inclination / 180 * PI);
  double s  = sin(inclination / 180 * PI);

	for(double len = -length ; len < length ; len+=res){
    for(double th = 0 ; th < 2 * PI ; th += dth){
      double x = len;
      double y = radius * cos(th);
      double z = radius * sin(th);
			cloud.points.push_back(pcl::PointXYZ(x * c - z * s, y, x * s + z * c));
    }
	}
	cloud.width = cloud.points.size();
	cloud.height = 1;

	if(format == "p"){
		if(comp == "a")
			pcl::io::savePCDFileASCII(output_path, cloud);
		else if(comp == "c")
			pcl::io::savePCDFileBinaryCompressed(output_path, cloud);
		else if(comp == "b")	
			pcl::io::savePCDFileBinary(output_path, cloud);
	} /* else if(format == "l"){
		if(comp == "a")
			pcl::io::savePLYFileASCII(output_path, cloud);
		else if(comp == "c")
			pcl::io::savePLYFileBinaryCompressed(output_path, cloud);
		else if(comp == "b")	
			pcl::io::savePLYFileBinary(output_path, cloud);
	}*/
	return 0;
}   
