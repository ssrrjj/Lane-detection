/*
 * lanedetection.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_LANEDETECTION_H_
#define INCLUDE_LANEDETECTION_H_

#include <vector>
#include <pcl/point_cloud.h>

#include "pcfitplane.h"
#include "dbscan.h"
#include "utils.h"
#include "lanepar.h"

//#define DEBUG 1
//#define REC_DETECT 1
void
findLanesInPointcloud(string pcdfile);

void
findLanesInPointcloud(string pcdfile, string parfile);

//std::vector<int>
//findLanesByConfig(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float INTENSITY_THRESHOLD, float eps);

std::vector<int>
findLanesByConfig(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float INTENSITY_THRESHOLD, float eps, LanePar=LanePar());

//std::vector<int>
//findLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

std::vector<int>
findLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, LanePar=LanePar());

//std::vector<int>
//findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string="x");

std::vector<int>
findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string="x", LanePar=LanePar());

int evalLaneCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &cluster, float laneW);

#endif /* INCLUDE_LANEDETECTION_H_ */
