/*
 * test_lanedetection.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "lanedetection.h"
#include "pcshow.h"

void
test_findLanes(){
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("../test/data/ptcROI_data00000_X347X407.pcd", *cloud);
  // pcl::io::loadPCDFile ("extracted_plane_data0000_X250X300.pcd", *cloud);
  // pcl::io::loadPCDFile ("data00000_X250X300_compressed.pcd", *cloud);

  // Step 1: Extract road plane
  float distThreshold = 0.15; // float distThreshold = 0.05;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = pcfitplane(cloud, distThreshold);

  vector<int> indset;
  pcfitplaneByROI(cloud, indset, distThreshold);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud = select(cloud, indset);

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
  indset = findLanes(cloud_filtered);
  // std::vector<int> indset = findLanesByConfig(cloud_filtered, 0.1, 0.1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud_filtered, indset);

  writer.write<pcl::PointXYZI> ("detected_lanes.pcd", *lane_ptc, false);

  custom_pcshow(lane_ptc);
}

void
test_findLanesByROI(){
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("point_cloud_00000.pcd", *cloud);

  vector<float> roi(2,0.0);
  roi[0]=250.0; roi[1]=300.0;
  roi[0]=347.0; roi[1]=407.0;
  std::vector<int> indset = findLanesByROI(cloud, roi);
  pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud, indset);
  custom_pcshow(lane_ptc);
}

void
test_findLanesAfterPlaneFit(){
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *planeCloud);

  std::cout<<"planeCloud->points.size() = "<<planeCloud->points.size()<<std::endl;

  std::vector<int> indsetPlane = findLanes(planeCloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(planeCloud, indsetPlane);

  std::cout<<"lane_ptc->points.size() = "<<lane_ptc->points.size()<<std::endl;
  custom_pcshow(lane_ptc);
}

void
test_findLanesByConfig_debug(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *planeCloud);
  std::cout<<"In test_findLanesByConfig: planeCloud->points.size() = "<<planeCloud->points.size()<<std::endl;

  int minPts = 10; // TODO
  float INTENSITY_THRESHOLD = 0.1;
  float eps = 0.2;
  // Filter by intensity to get lane point
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(planeCloud, indsetFiltered, INTENSITY_THRESHOLD);
  std::cout<<"In test_findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;

  // clustering
  DBSCAN dbscan;
  dbscan.setInputCloud (ptc);
  std::vector<int> dbclustering = dbscan.segment (eps, minPts);
  std::vector<std::vector<int>> & clusters = dbscan.clusters;

  std::cout<<"In test_findLanesByConfig: clusters.size() = "<<clusters.size()<<std::endl;
  int tmp_cnt=0;
  for(const auto & p: dbclustering){
      if(p>=0) tmp_cnt++;
  }
  std::cout<<"In findLanesByConfig: number of inlier points tmp_cnt= "<<tmp_cnt<<std::endl;

  std::cout<<"ZZZ after DBSCAN: clusters[79]: "<<std::endl;
  for(int i=0;i<clusters[79].size(); i++){
      std::cout<<clusters[79][i]<<":"<<dbclustering[clusters[79][i]]<<":"<<indsetFiltered[i]<<","<<std::endl;
  }

  // detect lanes
  int lanept_cnt=0;
  float laneW = 0.2; // float laneW = 0.15; // TODO read from config, may use 0.2
  int numClusters = clusters.size();
  for(int i=0; i<numClusters; i++){
      std::cout<<"ZZZ0: cluster "<<i<<" has " <<clusters[i].size() <<" points."<<std::endl;
      // filter small clusters with less than minClusterPts
      int minClusterPts = 100; // TODO read from config
      if (clusters[i].size()<minClusterPts){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  continue;
      }
      int flag = evalLaneCluster(ptc, clusters[i], laneW); // return 0 if it's a possible lane
      // int flag = evalLaneCluster(planeCloud, clusters[i], laneW); // return 0 if it's a possible lane
      if(flag){
	  std::cout<<"evalLaneCluster: cluster "<<i<<" with " <<clusters[i].size() <<" points is not a possible lane."<<std::endl;
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
#ifdef DEBUG
	  // std::cout<<"cluster "<<i<<" is not a lane, clusters[i].size() = "<<clusters[i].size()<<std::endl;
	  // pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptc = select(cloud, clusters[i]);
	  // custom_pcshow(cluster_ptc);
#endif
      }else
	lanept_cnt+=clusters[i].size();
  }
  std::cout<<"ZZZ: test_findLanesByConfig has " <<lanept_cnt <<" lane points."<<std::endl;

  for(int i=0;i<clusters[79].size(); i++){

      std::cout<<clusters[79][i]<<":"<<dbclustering[clusters[79][i]]<<":"<<indsetFiltered[i]<<","<<std::endl;
  }
  std::cout<<std::endl<<"ZZZ: clusters[79]: "<<std::endl;

  std::vector<int> indset;
  for(int i=0; i<dbclustering.size(); i++){
      if(dbclustering[i]>=0) {
	  // DEBUG
	  if(dbclustering[i]==79)
	  std::cout<<"ZZZ: indsetFiltered[i] = "<<indsetFiltered[i]<<", i= "<<i<<std::endl;
	  indset.push_back(indsetFiltered[i]); // indset.push_back(i);
      }
  }
  custom_pcshow(select(planeCloud, indset));
}

void
test_findLanesByConfig(float INTENSITY_THRESHOLD = 0.10, float eps = 0.2){
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *planeCloud);
  std::cout<<"In test_findLanesByConfig: planeCloud->points.size() = "<<planeCloud->points.size()<<std::endl;

  int minPts = 10; // TODO
  // Filter by intensity to get lane point
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(planeCloud, indsetFiltered, INTENSITY_THRESHOLD);
  std::cout<<"In test_findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;


  // clustering
  DBSCAN dbscan;
  dbscan.setInputCloud (ptc);
  std::vector<int> dbclustering = dbscan.segment (eps, minPts);
  std::vector<std::vector<int>> & clusters = dbscan.clusters;

  std::cout<<"In test_findLanesByConfig: clusters.size() = "<<clusters.size()<<std::endl;
  int tmp_cnt=0;
  for(const auto & p: dbclustering){
      if(p>=0) tmp_cnt++;
  }
  std::cout<<"in findLanesByConfig: number of inlier points tmp_cnt= "<<tmp_cnt<<std::endl;

  // detect lanes
  int lanept_cnt=0;
  float laneW = 0.2; // float laneW = 0.15; // TODO read from config, may use 0.2
  int numClusters = clusters.size();
  for(int i=0; i<numClusters; i++){
      // std::cout<<"ZZZ0: cluster "<<i<<" has " <<clusters[i].size() <<" points."<<std::endl;
      // filter small clusters with less than minClusterPts
      int minClusterPts = 100; // TODO read from config
      if (clusters[i].size()<minClusterPts){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  continue;
      }
      int flag = evalLaneCluster(ptc, clusters[i], laneW); // return 0 if it's a possible lane
      // int flag = evalLaneCluster(planeCloud, clusters[i], laneW); // return 0 if it's a possible lane
      if(flag){
	  std::cout<<"evalLaneCluster: cluster "<<i<<" with " <<clusters[i].size() <<" points is not a possible lane."<<std::endl;
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
      }else
	lanept_cnt+=clusters[i].size();
  }
  std::cout<<"ZZZ: test_findLanesByConfig has " <<lanept_cnt <<" lane points."<<std::endl;

  std::vector<int> indset;
  for(int i=0; i<dbclustering.size(); i++){
      if(dbclustering[i]>=0) {
	  // DEBUG
	  if(dbclustering[i]==79)
	  std::cout<<"ZZZ: indsetFiltered[i] = "<<indsetFiltered[i]<<", i= "<<i<<std::endl;
	  indset.push_back(indsetFiltered[i]); // indset.push_back(i);
      }
  }
  custom_pcshow(select(planeCloud, indset));
}

void test_findLanes2(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *cloud);
  std::cout<<"In test_findLanesByConfig: planeCloud->points.size() = "<<cloud->points.size()<<std::endl;

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);
  std::cout<<"numPts = "<<numPts<<std::endl;

  // find lanes with multiple configurations
  // intensity_threshold=0.1, eps = 0.2
  // std::vector<int> indset1 = findLanesByConfig(cloud, 0.1, 0.2);
  std::vector<int> indset1 = findLanesByConfig(cloud, 0.05, 0.4);
  std::cout<<"indset1.size() = "<<indset1.size()<<std::endl;
  custom_pcshow(select(cloud, indset1));
}

void
test_findLanesByROI_inBatch(string pcdfile, string fieldname ){
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (pcdfile, *cloud);
  // pcl::io::loadPCDFile ("point_cloud_00007.pcd", *cloud);

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);

  float piece_width = 50;
  // string fieldname = "x";
  vector<float> range;
  if(fieldname=="x"){
      range = getXLimits(cloud);
  }
  if(fieldname=="y"){
      range = getYLimits(cloud);
  }
  int N = (range[1]-range[0]+1)/piece_width;

  vector<float> roi(2,0.0);
  roi[0]=347.0; roi[1]=407.0;

  for(int i=0; i<N; i++){
      roi[0] = range[0]+piece_width*i-10;
      roi[1] = range[0]+piece_width*(i+1);
      if(i==N-1) roi[1] = range[1];
      std::vector<int> indset = findLanesByROI(cloud, roi, fieldname);
      pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud, indset);
      // custom_pcshow(lane_ptc);
      for(const auto & p: indset)
        lanepts[p]=1;
  }
  std::vector<int> lane_indset;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) lane_indset.push_back(i); // if(lanepts[i]>0) indset.push_back(i);
  }

//#ifdef DEBUG
  // Write the extracted plane to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("point_cloud_00006_lanepts.pcd", *select(cloud, lane_indset), false);
//#endif
  custom_pcshow(select(cloud, lane_indset));
}

int main(int argc, char* argv[])
{
  // test_findLanes2();
  // test_findLanesByConfig();
  // test_findLanesAfterPlaneFit();
  // test_findLanesByROI();
  // test_findLanes();
  // test_findLanesByROI_inBatch("point_cloud_00006.pcd", "y");
  // findLanesInPointcloud("point_cloud_00007.pcd");
  string gcdfile = "point_cloud_00017.pcd";
  if(argc>=2){
      gcdfile = argv[1];
  }
  findLanesInPointcloud(gcdfile);
  return (0);
}


