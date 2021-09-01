
#include "lanedetection.h"
#include "open3d/Open3D.h"
#include "utils.h"
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include "polyline.h"
using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //viewer->setBackgroundColor(0, 0, 0);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //for (int i = 0; i < 10; i++) {
    //    cloud->points.push_back(pcl::PointXYZ(i, i, i));
    //}
    //cloud->height = 1;
    //cloud->width = 10;
    //viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    //viewer->addLine(cloud->points[0], cloud->points[9], 255,0,0,"line");
    //while (!viewer->wasStopped())
    //{
    //    viewer->spinOnce(100);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //}
    // -----------------------------------------------------------------------------------------------
	//cv::Mat uimage = cv::imread(argv[1], cv::IMREAD_ANYDEPTH);
    // -----------------------------------------------------------------------------------------------
    
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 32; i < 51; i++) {
        pcl::io::loadPCDFile("lanemark_"+to_string(i)+".pcd", *cloud);
        PolyLine polyline(cloud);
    }
    return 0;

}

