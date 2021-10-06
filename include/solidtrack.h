#pragma once
#include "utils.h"
#include "polyline.h"
#include <string>
int solidtrack(string cloud_file, vector<float> p1, vector<float> p2, string save_file);

PolyLine solidtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, pcl::PointXYZ p1, pcl::PointXYZ p2);