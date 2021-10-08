#ifndef INCLUDE_POLYLINE_H_
#define INCLUDE_POLYLINE_H_
#include "utils.h"
class PolyLine {
public:
	vector<pcl::PointXYZ>points;
	vector<int>cuts;
	float length;
	PolyLine(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float length_threshold = 20, bool downsample = false);
	PolyLine();
	void smooth();
	void add(pcl::PointXYZ p);
	void writeSHP(string name);
};
void show(CloudPtr cloud, std::vector<pcl::PointXYZ> points);

void custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<PolyLine>& polylines);


#endif