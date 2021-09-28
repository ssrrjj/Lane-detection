#include "utils.h"
#include "pcfitplane.h"

#include <cmath>

void plane_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int> & inlier, vector<float> & param) {
  //cout << "plane filter" <<endl;
  inlier.clear();
  float x, y, z, n;
  for (int i = 0; i < cloud->points.size(); i++) {
    x = cloud->points[i].x;
    y = cloud->points[i].y;
    z = cloud->points[i].z;
    n = pow(param[0]*param[0] + param[1]*param[1]+ param[2]*param[2], 0.5);
    if (abs(param[0]*x + param[1]*y + param[2]*z + param[3])/n < 0.3)
      inlier.push_back(i);
  }
}



pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points(vector<float> xlimit, vector<float>ylimit, vector<float> param) {
  float a,b,c,d;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  
  a = param[0];
  b = param[1];
  c = param[2];
  d = param[3];
  float n = 100;
  cout<<a <<" "<<b<<" "<<c<<" " << d<< endl;
  cloud->width  = (int)n;
  cloud->height = (int)n;
  cloud->points.resize (cloud->width * cloud->height);
  float stepx = (xlimit[1] - xlimit[0])/ n, stepy = (ylimit[1] - ylimit[0])/n;
  int k = 0;
  for (int i = 0; i < n; i++)  {
    for (int j = 0 ; j < n; j++) {
      float x, y, z;
      x = xlimit[0] + stepx * i;
      y = ylimit[0] + stepy * j;
      z = (-d - a * x - b * y)/c;
      cloud->points[k].x = x;
      cloud->points[k].y = y;
      cloud->points[k].z = z;
      cloud->points[k].intensity = 0.5;
      
      k ++;
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr pcfitplane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float distThreshold)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_LMEDS);
  seg.setDistanceThreshold (distThreshold); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (NULL);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = select(cloud, inliers);
  return inlierCloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr pcfitplane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& indset, float distThreshold)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setMaxIterations(1000);
  seg.setProbability(0.1);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distThreshold); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration
  seg.setOptimizeCoefficients(true);
  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  
  //cout<<seg.getMaxIterations()<<endl;
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (NULL);
  }

#ifdef DEBUG
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = select(cloud, inliers);
  for (int i = 0 ; i < inliers->indices.size(); i++) 
    cloud->points[inliers->indices[i]].intensity = 1;
#ifdef DEBUG
  cout<<"first"<<endl;
  custom_pcshow(inlierCloud);
  custom_pcshow(cloud);
#endif
  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
  seg.setDistanceThreshold (0.3);
  seg.setInputCloud(inlierCloud);
  seg.segment(*inliers2, *coefficients);
  vector<int>close_to_plane;
  //vector<float>o3dp{0.02204448, 0.00582288, 0.99974003, 1.17171320};
  plane_filter(cloud, close_to_plane, coefficients->values);
  //plane_filter(cloud, close_to_plane, o3dp);
  inlierCloud = select(cloud, close_to_plane);
#ifdef DEBUG
  cout<<"second"<<endl;
  custom_pcshow(inlierCloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr plane = plane_points(getXLimits(cloud), getYLimits(cloud), coefficients->values);
  *plane = *inlierCloud + *plane;
  custom_pcshow(plane);
#endif
  // return indset by reference
  indset.clear();
  // indset.assign(inliers->indices.size(), -1);
  // for(int i=0; i<inliers->indices.size(); i++){
  //     indset[i]=inliers->indices[i];
  // }
  indset.assign(inliers->indices.size(), -1);
  for(int i=0; i<inliers->indices.size(); i++){
      indset[i]=inliers->indices[i];
  }
  return inlierCloud;
}


void
pcfitplaneByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int>& indset, Eigen::Vector4d & plane_model, float distThreshold, string fieldname)
{
  //custom_pcshow(cloud);
  float piece_width = 50;
  vector<float> range;
  if(fieldname=="x"){
      range = getXLimits(cloud);
  }
  if(fieldname=="y"){
      range = getYLimits(cloud);
  }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setMaxIterations(1000);
  seg.setProbability(0.1);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distThreshold); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration
  seg.setOptimizeCoefficients(true);
  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
/*
  auto cloud_ptr = pclToO3d(cloud);
  if (VERBOSE == 1) {
      cout << "fitplane input " << endl;
      custom_pcshow(cloud);
  }
  std::tuple<Eigen::Vector4d, std::vector<size_t>>plane;
  plane = cloud_ptr->SegmentPlane(distThreshold, 3, 1000);
  plane_model = get<0>(plane);
  std::vector<size_t> inliers = get<1>(plane);
  //if (VERBOSE == 1) {
  //    cout << "fitplane output" << endl;
  //    std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = std::make_shared<open3d::geometry::PointCloud>();
  //    inlier_cloud = cloud_ptr->SelectByIndex(inliers, FALSE);
  //    open3d::visualization::DrawGeometries({ inlier_cloud });
  //}

  indset.clear();
  for (int i = 0; i < inliers->indices.size(); i++) {
      indset.push_back((int)inliers->indices[i]);
  }
  //
}
