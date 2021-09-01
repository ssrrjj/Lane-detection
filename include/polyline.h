#ifndef INCLUDE_POLYLINE_H_
#define INCLUDE_POLYLINE_H_
#include "utils.h"
class PolyLine {
public:
	vector<pcl::PointXYZ>points;
	PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};

#endif