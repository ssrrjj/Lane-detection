#pragma once
#include "utils.h"
#include "polyline.h"

PolyLine dashedtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, cv::Vec3f p1, cv::Vec3f p2, cv::Vec3f p3);