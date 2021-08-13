#ifndef INCLUDE_POLYLINE_H_
#define INCLUDE_POLYLINE_H_
#include "utils.h"
class PolyLine {
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr points;
	PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};

#endif