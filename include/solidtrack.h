#pragma once
#include "utils.h"
#include "polyline.h"

PolyLine solidtrack(CloudPtr cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree, pcl::PointXYZ p1, pcl::PointXYZ p2);