/*
 * utils.h
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_
#include <open3d/Open3D.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>

#include <string>
#include "opencv2/opencv.hpp"
#include <math.h>
#include <algorithm>
#include "pcshow.h"
#include "dbscan.h"
#include <math.h>

#include<boost/make_shared.hpp>

#include "lanepar.h"

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 
extern int VERBOSE;

using namespace std;


//
using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr ;

void savepcd(CloudPtr cloud, string filename);

vector<float>
getXLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

vector<float> getXLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

vector<float> getYLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

vector<float> getYLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

vector<float>
getILimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr
select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr &inliers);

pcl::PointCloud<pcl::PointXYZI>::Ptr
select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &inliers);

pcl::PointCloud<pcl::PointXYZ>::Ptr select(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr& inliers);

pcl::PointCloud<pcl::PointXYZ>::Ptr select(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& inliers);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string fieldname, float lb, float ub);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int>&, string fieldname, float lb, float ub);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& , float, float = 1.0);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& , vector<int>&, float, float = 1.0);

vector<int> indexMapping(vector<int>&, vector<int>&);

float mean(vector<float>&);
float maxvalue(vector<float>&);
float stdv(vector<float>&);

void drawHist(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int n_bins = 50, int highlight = -1);
double Otsu_thresholding(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int n_bins);

pcl::PointCloud<pcl::PointXYZI>::Ptr
OtsuFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, string fieldname);

pcl::PointCloud <pcl::PointXYZI>::Ptr
regionGrowSeg(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<int> & idx);



bool isRect(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

// convert pcl::pointcloud to open3d::pointcloud
std::shared_ptr<open3d::geometry::PointCloud> pclToO3d(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);


cv::Mat findLaneInImage(cv::Mat uimage);
vector<string> SplitFilename(const std::string& str);

cv::Mat toImage(CloudPtr cloud, Eigen::Vector4d plane_model, float grid_size, vector<vector<int>>& pixel2cloud);
vector<vector<int>> ImageDbscan(cv::Mat& image, vector<int>& cloud2pixel, float eps = 3.0, int min_pts = 20);

void pca(CloudPtr cloud, vector<Eigen::Vector3f>& eigenvalues, float radius = 0.1);

#endif /* INCLUDE_UTILS_H_ */
