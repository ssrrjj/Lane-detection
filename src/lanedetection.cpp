/*
 * lanedetection.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "lanedetection.h"
#include <vector>
#include <math.h>
#include "pcshow.h"
#include <filesystem>

using namespace std;
using namespace std::filesystem;


int
evalLaneCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr& fullcloud, std::vector<int> &cluster, float laneW)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = select(fullcloud, cluster);
  // custom_pcshow(fullcloud);
  // TODO May have a direct API for PointCloud in PCL
  vector<float> xlimits = getXLimits(cloud);
  vector<float> ylimits = getYLimits(cloud);
  static int cloud_idx = 0;
  int xOriented = 1; // road travel along the x Axis, 0 for y Axis otherwise
  if(abs(ylimits[1]-ylimits[0]) > abs(xlimits[1]-xlimits[0]))
    xOriented = 0;

  vector<float> binWidth;
  int nbins = 0; // check the lane width in each bin
  if(xOriented){
      float lb = xlimits[0];
      // Calculate y width, TODO Outlier handling
      nbins = ceil((xlimits[1]-xlimits[0])/laneW);
      binWidth.assign(nbins, 0.0);
      for(int i =0; i<nbins; i++){
	  float ub = lb+laneW-0.01;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr bin_pc = filterByField(cloud, "x", lb, ub);
	  vector<float> bin_ylimits = getYLimits(bin_pc);
	  binWidth[i]=bin_ylimits[1]-bin_ylimits[0];
	  lb = ub;
      }
  }else{
      float lb = ylimits[0];
      // Calculate x width, TODO Outlier handling
      nbins = ceil((ylimits[1]-ylimits[0])/laneW);
      binWidth.assign(nbins, 0.0);
      for(int i =0; i<nbins; i++){
	  float ub = lb+laneW-0.01;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr bin_pc = filterByField(cloud, "y", lb, ub);
	  vector<float> bin_xlimits = getXLimits(bin_pc);
	  binWidth[i]=bin_xlimits[1]-bin_xlimits[0];
	  lb = ub;
      }
  }

  int laneflag = 1;
  // Heuristic to check if cloud is possibly a lane mark.
  if(mean(binWidth)<3*laneW && maxvalue(binWidth)<5*laneW && stdv(binWidth)<laneW)
    laneflag=0;

  //TODO if(sum(binWidth>5*laneW)/length(binWidth)<0.1 & sum(binWidth<3*laneW)/length(binWidth)>0.8)
  //    flag = 1;
  int cnt5W=0, cnt3W=0;
  for(const auto& w: binWidth){
    if(w>5*laneW) cnt5W++;
    if(w<3*laneW) cnt3W++;
  }
  if(cnt5W<0.1*binWidth.size() && cnt3W>0.8*cnt3W){
      laneflag=0;
  }
#ifdef REC_DETECT
  if (laneflag == 0) {

    // toImage(cloud, "lane_"+std::to_string(cloud_idx)+".jpg");
    // cloud_idx += 1;
    if (!isRect(cloud))
      laneflag = 1;
  }
// 2D projection of input point cloud. Check if it is a rectangle. TODO: set resolution. We need to use opencv. how to compile opencv with cmake
  // std::cout << "show the image" << std::endl;
  // int density = 10;
  // int h = ceil(density*(ylimits[1] - ylimits[0])), w = ceil(density*(xlimits[1] - xlimits[0]));
  // std::cout << h << ", " << w << ", " << cloud->points.size() <<", "<< laneflag<<std::endl;
  // // custom_pcshow(cloud);
  // cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
  // int row, col;
  
  // for (int idx = 0; idx < cloud->points.size(); ++idx) {
  //     row = round(density*(cloud->points[idx].y - ylimits[0]));
  //     if (row < 0)
  //         row = 0;
  //     if (row >= h)
  //         row = h-1;

  //     col = round(density*(cloud->points[idx].x - xlimits[0]));
  //     if (col < 0)
  //         col = 0;
  //     if (col >= w)
  //         col = w-1;
  //     img.at<cv::Vec3b>(row,col)[0] = 255;
  // }
  //cv::imshow("img", img);
  //cv::waitKey(0);
#endif
  return laneflag;
}

std::vector<int>
findLanesByConfig(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float INTENSITY_THRESHOLD, float eps, LanePar par)
{
  int minPts = par.dbscan_minpts; // int minPts = 10;

  // Filter by intensity to get lane point
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(cloud, indsetFiltered, INTENSITY_THRESHOLD);

#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
  custom_pcshow(ptc);
#endif

  // clustering
  DBSCAN dbscan;
  dbscan.setInputCloud (ptc);
  std::vector<int> dbclustering = dbscan.segment (eps, minPts);
  std::vector<std::vector<int>> & clusters = dbscan.clusters;

#ifdef DEBUG
  std::cout<<"In findLanesByConfig: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
  int tmp_cnt=0;
  for(const auto & p: dbclustering){
      if(p>=0) tmp_cnt++;
  }
  std::cout<<"In findLanesByConfig: number of inlier points tmp_cnt= "<<tmp_cnt<<std::endl;
#endif

  // detect lanes
  int lanept_cnt=0;
  float laneW = par.lane_width; // float laneW = 0.2; // float laneW = 0.15; // TODO read from config, may use 0.2
  int numClusters = clusters.size();
  for(int i=0; i<numClusters; i++){
      // filter small clusters with less than minClusterPts
      int minClusterPts = par.lanemark_minpts; // TODO read from config
      if (clusters[i].size()<minClusterPts){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  continue;
      }
      int flag = evalLaneCluster(ptc, clusters[i], laneW); // TODO ptc instead of cloud .return 0 if it's a possible lane
      if(flag){
	  for(auto& v: clusters[i]){
	      dbclustering[v]=-2-i;  // negative means outlier
	  }
	  //DEBUG
	  // std::cout<<"cluster "<<i<<" is not a lane, clusters[i].size() = "<<clusters[i].size()<<std::endl;
	  // pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptc = select(cloud, clusters[i]);
	  // custom_pcshow(cluster_ptc);
      }
      else lanept_cnt+=clusters[i].size();
  }

#ifdef DEBUG
  std::cout<<"Find " <<lanept_cnt <<" lane points."<<std::endl;
#endif

  std::vector<int> indset;
  for(int i=0; i<dbclustering.size(); i++){
      if(dbclustering[i]>=0) {
	  indset.push_back(indsetFiltered[i]); // indset.push_back(i);
      }
  }
  // DEBUG
#ifdef DEBUG
  std::cout<<"dbclustering.size() = "<<dbclustering.size()<<", indset.size() = "<<indset.size()<<std::endl;
#endif
  return indset;
}

std::vector<int>
findLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud, LanePar par)
{

  std::vector<int> indset;
  vector<int> indsetFiltered;

#ifdef DEBUG
  std::cout<<"DEBUG is on, and visualize the point clouds for debug in findLanes." <<std::endl;
  custom_pcshow(inCloud);
#endif
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = filterByIntensity(inCloud, indsetFiltered, 0.000); // TODO
// #ifdef DEBUG
//   custom_pcshow(cloud);
// #endif

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);
  vector<float> ilimits = getILimits(cloud);
#ifdef DEBUG
  std::cout<<"numPts = "<<numPts<<std::endl;
#endif
  // find lanes with multiple configurations
  // intensity_threshold=0.1, eps = 0.2
  custom_pcshow(cloud);
  drawHist(cloud, 100);
  std::vector<int> indset1 = findLanesByConfig(cloud, 0.1, 0.2, par);
#ifdef DEBUG
  std::cout<<"indset1.size() = "<<indset1.size()<<std::endl;
  custom_pcshow(select(cloud, indset1));
#endif
  std::vector<int> indset2 = findLanesByConfig(cloud, 0.12, 0.1, par);
#ifdef DEBUG
  std::cout<<"indset2.size() = "<<indset2.size()<<std::endl;
  custom_pcshow(select(cloud, indset2));
#endif
//  std::vector<int> indset3 = findLanesByConfig(cloud, 0.05, 0.4);
//  // std::vector<int> indset3 = findLanesByConfig(cloud, 0.1, 0.4);
//  // std::vector<int> indset3 = findLanesByConfig(cloud, 0.09, 0.2);
//#ifdef DEBUG
//  std::cout<<"indset3.size() = "<<indset3.size()<<std::endl;
//  custom_pcshow(select(cloud, indset3));
//#endif

//#ifdef DEBUG
//  custom_pcshow(select(cloud, indset1));
//  custom_pcshow(select(cloud, indset2));
//  custom_pcshow(select(cloud, indset3));
//#endif


  // Combine the results together
  for(const auto & p: indset1)
    lanepts[p]=1;
  for(const auto & p: indset2)
    lanepts[p]=1;
//  for(const auto & p: indset3)
//    lanepts[p]=1;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) indset.push_back(indsetFiltered[i]); // if(lanepts[i]>0) indset.push_back(i);
  }

//  // reclustering for the laneset
//  pcl::PointCloud<pcl::PointXYZI>::Ptr laneptc = select(cloud, indset);
//  std::vector<int> indset_final = findLanesByConfig(cloud, 0.01, 0.4);
//  return indset_final;

  return indset;
}

std::vector<int>
validateMark(CloudPtr inCloud) {
    DBSCAN dbscan;
    dbscan.setInputCloud(inCloud);
    std::vector<int> dbclustering = dbscan.segment(0.1, 5);
    std::vector<std::vector<int>>& clusters = dbscan.clusters;



    // detect lanes
    int lanept_cnt = 0;
    float laneW = 0.2; // float laneW = 0.2; // float laneW = 0.15; // TODO read from config, may use 0.2
    int numClusters = clusters.size();
    for (int i = 0; i < numClusters; i++) {
        // filter small clusters with less than minClusterPts
        int minClusterPts = 100; // TODO read from config
        if (clusters[i].size() < minClusterPts) {
            for (auto& v : clusters[i]) {
                dbclustering[v] = -2 - i;  // negative means outlier
            }
            continue;
        }
        int flag = evalLaneCluster(inCloud, clusters[i], laneW); // TODO ptc instead of cloud .return 0 if it's a possible lane
        if (flag) {
            for (auto& v : clusters[i]) {
                dbclustering[v] = -2 - i;  // negative means outlier
            }
            //DEBUG
            // std::cout<<"cluster "<<i<<" is not a lane, clusters[i].size() = "<<clusters[i].size()<<std::endl;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptc = select(cloud, clusters[i]);
            // custom_pcshow(cluster_ptc);
        }
        else lanept_cnt += clusters[i].size();
    }

#ifdef DEBUG
    std::cout << "Find " << lanept_cnt << " lane points." << std::endl;
#endif

    std::vector<int> indset;
    for (int i = 0; i < dbclustering.size(); i++) {
        if (dbclustering[i] >= 0) {
            indset.push_back(i); // indset.push_back(i);
        }
    }
    // DEBUG
#ifdef DEBUG
    std::cout << "dbclustering.size() = " << dbclustering.size() << ", indset.size() = " << indset.size() << std::endl;
#endif


    return indset;
}
std::vector<int>
findLanes_adp(pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud, Eigen::Vector4d plane_model, float grid_size, LanePar par) {
    


    std::vector<int> indset;
    vector<int> indsetFiltered;

#ifdef DEBUG
    std::cout << "DEBUG is on, and visualize the point clouds for debug in findLanes." << std::endl;
    custom_pcshow(inCloud);
#endif
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = filterByIntensity(inCloud, indsetFiltered, 0.000); // TODO
  // #ifdef DEBUG
  //   custom_pcshow(cloud);
  // #endif

    int numPts = cloud->points.size();
    std::vector<int> lanepts(numPts, 0);
#ifdef DEBUG
    std::cout << "numPts = " << numPts << std::endl;
#endif
    vector<int> mark_idx = findLaneByImage(inCloud, plane_model, grid_size, par);

    CloudPtr mark_cloud = select(inCloud, mark_idx);
    if (VERBOSE) {
        cout << "show mark cloud" << endl;
        custom_pcshow(mark_cloud);
    }
    //vector<int>valid_mark_idx = validateMark(mark_cloud);
    //vector<int>& valid_mark_idx = mark_idx;
    //CloudPtr valid_mark_cloud = select(mark_cloud, valid_mark_idx);
    //cout << "show valid mark cloud" << endl;
    //custom_pcshow(valid_mark_cloud);
    //for (const auto& p : valid_mark_idx) 
    //    lanepts[mark_idx[p]] = 1;
    for (const auto& p : mark_idx) {
        lanepts[p] = 1;
    }
    for (int i = 0; i < numPts; i++) {
        if (lanepts[i] > 0) indset.push_back(indsetFiltered[i]); 
    }
    
    return indset;

}

std::vector<int>
findLanesByROI(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> roi, string fieldname, LanePar par)
{
  pcl::PCDWriter writer;
  // get point cloud in ROI
  vector<int> indsetCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptcROI =
  filterByField(cloud, indsetCloud, fieldname, roi[0], roi[1]);

  std::cout<<"INFO: process points in range "<<fieldname<<": "<<roi[0]<<", "<<roi[1]<<std::endl;
  //custom_pcshow(ptcROI);
#ifdef DEBUG
  // Write the extracted plane to disk
  std::cout<<"INFO: process points in range "<<fieldname<<": "<<roi[0]<<", "<<roi[1]<<std::endl;
  custom_pcshow(ptcROI);
  writer.write<pcl::PointXYZI> ("ptcROI_debug.pcd", *ptcROI, false);
  //custom_pcshow(ptcROI);
#endif
  writer.write<pcl::PointXYZI>("ptcROI_debug.pcd", *ptcROI, false);
  // Step 1: Extract road plane
  float distThreshold = par.plane_dist_threshold; // float distThreshold = 0.05; // float distThreshold = 0.15;
  vector<int> indsetROI;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = pcfitplane(ptcROI, indsetROI, distThreshold);

  Eigen::Vector4d plane_model;
  pcfitplaneByROI(ptcROI, indsetROI, plane_model, distThreshold, fieldname);
  

  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = select(ptcROI, indsetROI);
  //toImage(planeCloud, plane_model, 0.1);
  // std::cout<<"plane cloud"<<std::endl;
  // custom_pcshow(planeCloud);
  // std::vector<int> indset2;
  // planeCloud = regionGrowSeg(planeCloud, indset2);
  // indsetROI = indexMapping(indsetROI, indset2);
  // std::cout<<"plane removed outlier"<<std::endl;
  //custom_pcshow(planeCloud);

//  // Step 2: Find lane marks
  std::vector<int> indsetROIinCloud = indexMapping(indsetCloud,indsetROI);
  
// Use Ostu Thresholding to select threshold adaptively
// First remove outliers on the plane
  // double ostu_th = Ostu_thresholding(planeCloud, 100);
  // std::cout<<"ostu threshold:"<<ostu_th<<std::endl;
  //drawHist(planeCloud, 100);
  //pcl::PointCloud<pcl::PointXYZI>::Ptr ostu_cloud = filterByIntensity(planeCloud, ostu_th);
  //custom_pcshow(ostu_cloud);
  // 
  


  //std::vector<int> indsetPlane = findLanes(planeCloud, par);
  if (fieldname == "y") {
      Eigen::Matrix4d T;
      T.row(0) = Eigen::Vector4d(0., 1., 0., 0.);
      T.row(1) = Eigen::Vector4d(1., 0., 0., 0.);
      T.row(2) = Eigen::Vector4d(0., 0., 1., 0.);
      T.row(3) = Eigen::Vector4d(0., 0., 0., 1.);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*planeCloud, *transformed_cloud, T);
      planeCloud = transformed_cloud;

  }

  std::vector<int> indsetPlane = findLanes_adp(planeCloud, plane_model, 0.075, par);

#ifdef DEBUG
  // custom_pcshow(select(planeCloud, indsetPlane));
#endif

  std::vector<int> indset = indexMapping(indsetROIinCloud,indsetPlane);

  return indset;
}

void
findLanesInPointcloud(string pcdfile, LanePar& par){
  std::cout<<"Running lane detection for "+pcdfile<<" ......"<<std::endl;
  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (pcdfile, *cloud);
  // pcl::io::loadPCDFile ("point_cloud_00007.pcd", *cloud);

  int numPts = cloud->points.size();
  std::vector<int> lanepts(numPts, 0);

  float subregion_width = par.subregion_width;
  string fieldname = "x";

  vector<float> range;
  vector<float> xlimits = getXLimits(cloud);
  vector<float> ylimits = getYLimits(cloud);
  vector<float> ilimits = getILimits(cloud);
  for (int i = 0 ; i < cloud->points.size(); i++) {
    cloud->points[i].intensity /= ilimits[1];
  }
  ilimits = getILimits(cloud);
  cout<<"ilimits "<<ilimits[1]<<endl;
  std::cout<<"X limit:"<<xlimits[0]<<" "<<xlimits[1]<<std::endl;
  std::cout<<"Y limit:"<<ylimits[0]<<" "<<ylimits[1]<<std::endl;
  if(xlimits[1]-xlimits[0]<ylimits[1]-ylimits[0]){
      fieldname = "y";
      range=ylimits;
  }else{
      fieldname = "x";
      range=xlimits;
  }

  int N = (range[1]-range[0]+1)/subregion_width;

  vector<float> roi(2,0.0);
  assert(par.start <= N);
  
  for(int i=(N+par.start)%N; i<=N; i++){
      roi[0] = range[0]+subregion_width*i-10;
      roi[1] = range[0]+subregion_width*(i+1);
      if(i==N) roi[1] = range[1];
      std::vector<int> indset = findLanesByROI(cloud, roi, fieldname, par);
      pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud, indset);
      pcl::PCDWriter subwriter;
      // #ifdef REC_DETECT
      //   subwriter.write<pcl::PointXYZI> (pcdfile.substr(0, pcdfile.find(".pcd"))+"_rect_output"+std::to_string(i)+".pcd", *lane_ptc, false);
      // #else
      //   subwriter.write<pcl::PointXYZI> (pcdfile.substr(0, pcdfile.find(".pcd"))+"_output"+std::to_string(i)+".pcd", *lane_ptc, false);
      // #endif
      // custom_pcshow(lane_ptc);
      for(const auto & p: indset)
        lanepts[p]=1;
      
  }
  std::vector<int> lane_indset;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) lane_indset.push_back(i); // if(lanepts[i]>0) indset.push_back(i);
  }

  std::cout<<"Lane detection completed!"<<std::endl;

  // Write the extracted plane to disk
  pcl::PCDWriter writer;
#ifdef REC_DETECT
  writer.write<pcl::PointXYZI> (pcdfile.substr(0, pcdfile.find(".pcd"))+"_rect_output.pcd", *select(cloud, lane_indset), false);
#else
  vector<string> path_name = SplitFilename(pcdfile);
  if (!exists(path_name[0] + "/" + par.save_to))
  {
      create_directory(path_name[0] + "/" + par.save_to);
  }
  string output_file_path = path_name[0] + "/" + par.save_to + "/" + path_name[1].substr(0, path_name[1].length() - 4) + "_output.pcd";
  //if (!exists(output_file_path))
    writer.write<pcl::PointXYZI> (output_file_path, *select(cloud, lane_indset), false);
#endif
#ifdef DEBUG
  custom_pcshow(select(cloud, lane_indset));
#endif

}
int VERBOSE = 0;
void
findLanesInPointcloud(string pcdfile, string parfile){
  LanePar par(parfile);
  if (par.verbose) {
      VERBOSE = 1;
  }
  else {
      VERBOSE = 0;
  }
  findLanesInPointcloud(pcdfile, par);
}

void
findLanesInPointcloud(string pcdfile){
  LanePar par;
  findLanesInPointcloud(pcdfile, par);
}
