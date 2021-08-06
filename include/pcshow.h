/*
 * pcshow.h
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#ifndef INCLUDE_PCSHOW_H_
#define INCLUDE_PCSHOW_H_

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void custom_pcshow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr whole, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, cv::Vec6f line1, cv::Vec6f line2);
#endif /* INCLUDE_PCSHOW_H_ */
