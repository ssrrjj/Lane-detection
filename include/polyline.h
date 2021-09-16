#ifndef INCLUDE_POLYLINE_H_
#define INCLUDE_POLYLINE_H_
#include "utils.h"
class PolyLine {
public:
	vector<pcl::PointXYZ>points;
	PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float length_threshold = 20, bool downsample = true);
	PolyLine();
};
void show(CloudPtr cloud, std::vector<pcl::PointXYZ> points);
#endif