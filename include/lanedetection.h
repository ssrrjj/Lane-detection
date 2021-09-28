/*
 * lanedetection.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_LANEDETECTION_H_
#define INCLUDE_LANEDETECTION_H_
#include "utils.h"
#include <vector>
#include <pcl/point_cloud.h>

#include "pcfitplane.h"
#include "dbscan.h"

#include "lanepar.h"
#include "lanemark.h"

//#define DEBUG 1
//#define REC_DETECT 1


void
LaneDetection(string pcdfile, string save_path, function<void(double)> progress);

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
findLanes_adp(vector<LaneMark*>& marks, pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud, Eigen::Vector4d plane_model, float grid_size, LanePar par);


std::vector<int>
findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string="x", LanePar=LanePar());

int evalLaneCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &cluster, float laneW);


vector<int> findLaneByImage(vector<LaneMark*>& marks, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4d plane_model, float grid_size, LanePar par);

void extractLine(vector<LaneMark*>& grouped, cv::Mat lane_mark, vector<vector<int>>& pixel2cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr whole, LanePar par);
void extractLine(CloudPtr cloud, LanePar par);
cv::Mat removeFalsePostive(cv::Mat lane_mark, LanePar par);
#endif /* INCLUDE_LANEDETECTION_H_ */
