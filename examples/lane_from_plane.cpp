#include "lanedetection.h"
#include "open3d/Open3D.h"
#include "utils.h"

int main(int argc, char* argv[])
{
    
	//cv::Mat uimage = cv::imread(argv[1], cv::IMREAD_ANYDEPTH);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	custom_pcshow(cloud);
	LanePar par(argv[2]);
	extractLine(cloud, par);
	return 0;

}

