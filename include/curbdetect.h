#pragma once
#include "utils.h"
#include "pcfitplane.h"
#include <pcl/features/normal_3d.h>
class Voxel {
public:
	float max_z, min_z;
	CloudPtr points;
	vector<int> idx;


	void add(pcl::PointXYZI p, int i);
	Voxel();
	float getnorm(Eigen::Vector4f& plane_parameters);
	

};
void FindCurbByROI(CloudPtr cloud, CloudPtr result);
CloudPtr FindCurb(CloudPtr cloud);