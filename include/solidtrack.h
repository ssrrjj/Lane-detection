#pragma once
#include "utils.h"
#include "polyline.h"
#include <string>
void solidtrack(CloudPtr cloud, vector<float> p1, vector<float> p2, vector<vector<float>>& ret);

PolyLine solidtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, pcl::PointXYZ p1, pcl::PointXYZ p2);