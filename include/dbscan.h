/*
 * dbscan.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_DBSCAN_H_
#define INCLUDE_DBSCAN_H_
#include "utils.h"
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>

#include <iostream>
#include <vector>
#include <ctime>



/**
 * \brief This class implements the DBSCAN algorithm based on density of the point cloud.
 */
class DBSCAN
{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  void
  setInputCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud);
  void
  setInputCloud (pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud);
  void
  regionQuery (pcl::PointXYZ &searchPoint, double radius,
	       std::vector<int> &pointIdxRadiusSearch);
  void
  expandCluster (int PtIdx, std::vector<int> &neighborPts,
		 std::vector<int> &cluster, float eps, int minPts);
  std::vector<int>
  segment (float eps, int minPts);
  std::vector<std::vector<int>> clusters;
};
#endif /* INCLUDE_DBSCAN_H_ */
