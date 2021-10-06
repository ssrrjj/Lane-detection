#pragma once
#include "utils.h"
#include "polyline.h"
int dashedtrack(string cloud_file, vector<float> p1, vector<float> p2, vector<float> p3, string save_file);
PolyLine dashedtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3);