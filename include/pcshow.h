/*
 * pcshow.h
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */

#ifndef INCLUDE_PCSHOW_H_
#define INCLUDE_PCSHOW_H_
#include "utils.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
//#include "polyline.h"

//#include "lanemark.h"
using namespace std;
void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void custom_pcshow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr whole, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, cv::Vec6f line1, cv::Vec6f line2);

void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr c1, pcl::PointCloud<pcl::PointXYZI>::Ptr c2);
void toRGB(pcl::PointCloud<pcl::PointXYZI>::Ptr gray, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb, cv::Vec3b color);
void
custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, pcl::PointXYZ l1p1, pcl::PointXYZ l1p2, pcl::PointXYZ l2p1, pcl::PointXYZ l2p2);
//void custom_pcshow(vector<LaneMark*>& marks);
void custom_pcshow(pcl::PointCloud<pcl::PointXYZ>::Ptr tmp, vector<pcl::PointXYZ> points);

//void custom_pcshow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<PolyLine>& polylines);


#endif 
