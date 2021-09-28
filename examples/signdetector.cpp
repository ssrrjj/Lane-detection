#include "traffic_sign.h" 
#include "LasStream.h"
#include <filesystem>

using namespace std::filesystem;
int main(int argc, char* argv[]) {
	std::string filename(argv[1]);
	cout << "curb detection " << filename << endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::io::loadPCDFile(filename, *cloud);
	FindTrafficSignByROI(cloud);

}

