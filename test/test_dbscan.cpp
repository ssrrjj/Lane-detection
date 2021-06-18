/*
 * test_dbscan.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "dbscan.h"
#include "pcshow.h"
#include "lanedetection.h"

void
test_dbscan ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI (
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloudXYZI);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (*cloudXYZI, *cloud);

  DBSCAN dbscan;
  dbscan.setInputCloud (cloud);
  std::vector<int> dbclustering = dbscan.segment (0.6, 10);

  pcl::PointCloud<pcl::PointXYZL>::Ptr cloudXYZL (
      new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud (*cloud, *cloudXYZL);

  for(int i = 0; i<cloud->points.size (); i++){
      cloudXYZL->points[i].label = dbclustering[i];
  }

  // Write the clustered point cloud to disk
  pcl::PCDWriter writer;
  writer.write < pcl::PointXYZL > ("dbscan_clustered.pcd", *cloudXYZL, false);
}

void
test_dbscan2 ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI (
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_filtered.pcd", *cloudXYZI);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud (*cloudXYZI, *cloud);

  DBSCAN dbscan;
  dbscan.setInputCloud (cloud);
  std::vector<int> dbclustering = dbscan.segment (0.1, 10);

  int numClusters  = 0;
  for(int i=0;i<cloud->points.size ();i++)
    if(numClusters<dbclustering[i]) numClusters = dbclustering[i]+1;

  for(int i = 0; i<cloud->points.size (); i++){
      cloudXYZI->points[i].intensity = (dbclustering[i]+1.0)/(1.0+numClusters);
  }

  // Write the clustered point cloud to disk
  pcl::PCDWriter writer;
  writer.write < pcl::PointXYZI > ("dbscan_clustered_intensity.pcd", *cloudXYZI, false);

  custom_pcshow(cloudXYZI);
}

void test_dbscan3(){

  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *planeCloud);

  int minPts = 10; // TODO

  // Filter by intensity to get lane point
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(planeCloud, indsetFiltered, 0.1);
  // pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(planeCloud, indsetFiltered, 0.05);
#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
#endif

  // clustering
  DBSCAN dbscan;
  dbscan.setInputCloud (ptc);
  std::vector<int> dbclustering = dbscan.segment (0.2, minPts); // std::vector<int> dbclustering = dbscan.segment (0.4, minPts);
  std::vector<std::vector<int>> & clusters = dbscan.clusters;

#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
  int tmp_cnt=0;
  for(const auto & p: dbclustering){
      if(p>=0) tmp_cnt++;
  }
  std::cout<<"In findLanesByConfig: number of inlier points tmp_cnt= "<<tmp_cnt<<std::endl;
#endif

#ifdef DEBUG
  vector<int> indset;
  for(const auto & p: dbclustering){
      if(p>=0) indset.push_back(indsetFiltered[p]);
  }
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("afterDbscan.pcd", *select(planeCloud, indset), false);


  custom_pcshow(select(planeCloud, indset));
#endif
}

int
main ()
{
  test_dbscan3();
}

