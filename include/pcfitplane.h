/*
 * pcfitplane.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_PCFITPLANE_H_
#define INCLUDE_PCFITPLANE_H_

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <string>

using namespace std;
#define DEBUG
pcl::PointCloud<pcl::PointXYZI>::Ptr
pcfitplane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float distThreshold);

pcl::PointCloud<pcl::PointXYZI>::Ptr
pcfitplane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>&, float distThreshold);

void
pcfitplaneByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int>&, float, string="x");


#endif /* INCLUDE_PCFITPLANE_H_ */
