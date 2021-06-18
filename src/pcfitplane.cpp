#include "pcfitplane.h"
#include "utils.h"


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
  seg.setMethodType (pcl::SAC_RANSAC);
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
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distThreshold); // pcfitplane use 0.05 as the maxDistance, here use 0.15, need some calibration

  seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

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

  // return indset by reference
  indset.clear();
  indset.assign(inliers->indices.size(), -1);
  for(int i=0; i<inliers->indices.size(); i++){
      indset[i]=inliers->indices[i];
  }
  return inlierCloud;
}

void
pcfitplaneByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int>& indset, float distThreshold, string fieldname)
{
  float piece_width = 10;
  vector<float> range;
  if(fieldname=="x"){
      range = getXLimits(cloud);
  }
  if(fieldname=="y"){
      range = getYLimits(cloud);
  }
  int N = (range[1]-range[0]+1)/piece_width;

  for(int i=0; i<N;i++){
      std::vector<int> indsetROI;
      std::vector<int> indsetPlane;
      float lb = range[0]+piece_width*i;
      float ub = range[0]+piece_width*(i+1);
      if(i==N-1) ub = range[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud =
	pcfitplane(filterByField(cloud, indsetROI, fieldname, lb,ub), indsetPlane, distThreshold);
      vector<int> ind = indexMapping(indsetROI, indsetPlane);
      indset.insert(indset.end(), ind.begin(), ind.end());
  }
}
