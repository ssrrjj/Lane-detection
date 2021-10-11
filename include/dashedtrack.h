#pragma once
#include "utils.h"
#include "polyline.h"
void dashedtrack(CloudPtr cloud, vector<float> p1, vector<float> p2, vector<float> p3, vector<vector<float>>& ret);
PolyLine dashedtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3);