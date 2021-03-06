/*
 * test_pcfitplane.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */
#include "utils.h"
#include "pcfitplane.h"
#include "pcshow.h"


void test_pcfitplane(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("../test/data/ptcROI_data00000_X347X407.pcd", *cloud);
#ifdef DEBUG
  std::cout<<"cloud has "<<cloud->points.size()<<" total points."<<std::endl;
  custom_pcshow(cloud);
#endif

  // Visualization
  // custom_pcshow(cloud);

  // Step 1: Extract road plane
  float distThreshold = 0.3;
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = pcfitplane(cloud, distThreshold);
#ifdef DEBUG
  std::cout<<"inlierCloud has "<<inlierCloud->points.size()<<" total points."<<std::endl;
  custom_pcshow(inlierCloud);
#endif

  // custom_pcshow(inlierCloud);

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("extracted_plane6.pcd", *inlierCloud, false);

  // Step 2: filter by intensity
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = filterByIntensity(inlierCloud, 0.1); // TODO
  writer.write<pcl::PointXYZI> ("extracted_plane_filtered.pcd", *cloud_filtered, false);

  // custom_pcshow(cloud_filtered);

  // Step3: Clustering
}

void test_pcfitplane_bypiece(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("../test/data/ptcROI_data00000_X347X407.pcd", *cloud);
#ifdef DEBUG
  std::cout<<"cloud has "<<cloud->points.size()<<" total points."<<std::endl;
  custom_pcshow(cloud);
#endif

  // Visualization
  // custom_pcshow(cloud);

  // Step 1: Extract road plane
  float distThreshold = 0.15;
  for(int i=0; i<6;i++){
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = pcfitplane(filterByField(cloud, "x", 347+10*i,357+10*i), distThreshold);
#ifdef DEBUG
  std::cout<<"inlierCloud has "<<inlierCloud->points.size()<<" total points."<<std::endl;
  custom_pcshow(inlierCloud);
#endif
  }
}

void test_pcfitplane_byROI(){
  cout<<"read pointcloud " <<endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile ("../../test/data/ptcROI_debug.pcd", *cloud);
  //custom_pcshow(cloud);
#ifdef DEBUG
  std::cout<<"cloud has "<<cloud->points.size()<<" total points."<<std::endl;
  custom_pcshow(cloud);
#endif

  float distThreshold = 0.8;
  cout<<"disThreshold: "<<distThreshold<<endl;
  vector<int> indset;
  cout<<"fit by roi"<<endl;
  Eigen::Vector4d plane_model;
  pcfitplaneByROI(cloud, indset, plane_model, distThreshold, "y");
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptcPlane = select(cloud, indset);
  
  
#ifdef DEBUG
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("extracted_plane_bypieces.pcd", *ptcPlane, false);
  custom_pcshow(ptcPlane);
#endif
}

int
 main (int argc, char** argv)
{
  test_pcfitplane_byROI();

  return (0);
}
