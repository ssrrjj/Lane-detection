#ifndef INCLUDE_POLYLINE_H_
#define INCLUDE_POLYLINE_H_
#include "utils.h"
class PolyLine {
public:
	vector<pcl::PointXYZ>points;
	vector<int>cuts;
	PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float length_threshold = 20, bool downsample = false);
	PolyLine();
	void smooth();
	void writeSHP(string name);
};
void show(CloudPtr cloud, std::vector<pcl::PointXYZ> points);
#endif