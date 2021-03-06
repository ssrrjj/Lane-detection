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
#include <set>
#include "lanemark.h"
#include <iostream>
#include <fstream>
#include "lasStream.h"
#include "shapefil.h"
#include <pcl/common/pca.h>

using namespace std;
using namespace std::filesystem;
using namespace cv;
void custom_pcshow(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<bool>& visited, vector<int>& cloud2pixel, vector<pcl::PointXYZ>& points) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visited_c(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unvisited_c(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < cloud->points.size(); i++) {
        auto& p = cloud->points[i];
        if (visited[cloud2pixel[i]]) {

            visited_c->points.push_back(pcl::PointXYZRGB(p.x, p.y, p.z, 0, 255, 0));
        }
        else {
            unvisited_c->points.push_back(pcl::PointXYZRGB(p.x, p.y, p.z, 255, 255, 255));
        }
    }
    visited_c->width = visited_c->points.size();
    visited_c->height = 1;
    unvisited_c->width = unvisited_c->points.size();
    unvisited_c->height = 1;
    viewer->addPointCloud<pcl::PointXYZRGB>(visited_c, "cloud1");
    viewer->addPointCloud<pcl::PointXYZRGB>(unvisited_c, "cloud2");
    int line_idx = 0;
    for (int point_idx = 0; point_idx < points.size() - 1; point_idx++) {
        viewer->addLine(points[point_idx], points[point_idx + 1], 255, 0, 0, "line" + to_string(point_idx));
    }
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void showpolyline(CloudPtr cloud, vector<pcl::PointXYZ> points) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto & p : cloud->points)
        tmp->points.push_back(pcl::PointXYZ(p.x, p.y, p.z));
    viewer->addPointCloud<pcl::PointXYZ>(tmp, "cloud");
    for (int point_idx = 0; point_idx < points.size() - 1; point_idx++) {
        viewer->addLine(points[point_idx], points[point_idx + 1], 255, 0, 0, "line" + to_string(point_idx));
    }
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
vector<LaneMark*>all_marks;
vector<PolyLine> all_polylines;
pcl::PointCloud<pcl::PointXYZI>::Ptr all_lane_ptc(new pcl::PointCloud<pcl::PointXYZI>);
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
findLanes_adp(vector<LaneMark*> & marks, pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud, Eigen::Vector4d plane_model, float grid_size, LanePar par) {
    

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
    vector<int> mark_idx = findLaneByImage(marks, inCloud, plane_model, grid_size, par);

    
    CloudPtr mark_cloud = select(inCloud, mark_idx);
    if (VERBOSE == 1) {
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
int LaneMark::global_idx = 0;
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
  //writer.write<pcl::PointXYZI>("ptcROI_debug.pcd", *ptcROI, false);
  // Step 1: Extract road plane
  float distThreshold = par.plane_dist_threshold; // float distThreshold = 0.05; // float distThreshold = 0.15;
  vector<int> indsetROI;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = pcfitplane(ptcROI, indsetROI, distThreshold);

  Eigen::Vector4d plane_model;
  if (ptcROI->points.size() < 100) {
      return indsetROI;
  }
  pcfitplaneByROI(ptcROI, indsetROI, plane_model, distThreshold, fieldname);
  

  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud = select(ptcROI, indsetROI);
  //toImage(planeCloud, plane_model, 0.1);
   //std::cout<<"plane cloud"<<std::endl;
   //custom_pcshow(planeCloud);
//  // Step 2: Find lane marks
  std::vector<int> indsetROIinCloud = indexMapping(indsetCloud,indsetROI);


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
  
  vector<LaneMark*> marks;

  set<int> added;
  std::vector<int> indsetPlane = findLanes_adp(marks, planeCloud, plane_model, par.grid_size, par);

  

  // cout << "match with previous marks" << endl;
  //vector<LaneMark*> all_marks_tmp(all_marks);
  //all_marks.clear();


  //for (auto new_mark : marks) {
  //    LaneMark* best_match = nullptr;
  //    float score;
  //    float best_score = 100000;
  //    for (auto mark : all_marks_tmp) {
  //        if (is_coline(mark, new_mark, 15, 1, &score)) {
  //            if (score < best_score) {
  //                best_score = score;
  //                best_match = mark;
  //            }
  //        }
  //    }
  //    
  //    if (best_match != nullptr)
  //        join(new_mark, best_match);
  //}

  //for (auto new_mark : marks) {
  //    all_marks_tmp.push_back(new_mark);
  //    //if (added.find(new_mark->idx) == added.end()) {
  //    //    all_marks.push_back(new_mark);
  //    //}
  //    //else
  //    //    delete new_mark;
  //}
  //vector<LaneMark*>all_marks_tmp2 = group2(all_marks_tmp);
  //
  //for (auto mark : all_marks_tmp2) {
  //    if (mark->points->points.size() > 30)
  //        all_marks.push_back(mark);
  //}
  //

  //if (VERBOSE == 2) 
  //  custom_pcshow(all_marks);
  
#ifdef DEBUG
  // custom_pcshow(select(planeCloud, indsetPlane));
#endif

  std::vector<int> indset = indexMapping(indsetROIinCloud,indsetPlane);
  
  return indset;
}

void
findLanesInPointcloud(string pcdfile, LanePar& par, bool cloud_output, string shape_file_path, function<void(double)> processor = NULL){
  std::cout<<"Running lane detection for "+pcdfile<<" ......"<<std::endl;
  //check if it is done before
  vector<string> path_name = SplitFilename(pcdfile);
  if (!exists(path_name[0] + "/" + par.save_to))
  {
      create_directory(path_name[0] + "/" + par.save_to);
  }
  string output_file_path = path_name[0] + "/" + par.save_to + "/" + path_name[1].substr(0, path_name[1].length() - 4) + "_output";
  if (exists(output_file_path + ".pcd")) {
      cout << "Done before." << endl;
      return;
  }

  // Read in point cloud to apply lane detection
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::io::loadPCDFile(pcdfile, *cloud);
  if (pcdfile[pcdfile.length()-1] == 'd')
      pcl::io::loadPCDFile(pcdfile, *cloud);
  // pcl::io::loadPCDFile ("point_cloud_00007.pcd", *cloud);
  else
      readlas(pcdfile, cloud);
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
  int N = (range[1]-range[0])/subregion_width;
  vector<float> roi(2,0.0);
  assert(par.start <= N);
  pcl::PCDWriter writer;
  
  for(int i=(N+par.start)%N; i<=N; i++){
      roi[0] = range[0]+subregion_width*i;
      roi[1] = range[0]+subregion_width*(i+1);
      if(i==N) roi[1] = range[1];
      std::vector<int> indset = findLanesByROI(cloud, roi, fieldname, par);
      pcl::PointCloud<pcl::PointXYZI>::Ptr lane_ptc = select(cloud, indset);
      
      pcl::PCDWriter subwriter;
      for(const auto & p: indset)
        lanepts[p]=1;
      double rate = ((roi[1]-range[0]) / (range[1]-range[0]));
      //cout << rate << endl;
      if (processor) {
          processor(rate);
      }
  }
  std::vector<int> lane_indset;
  for(int i=0; i<numPts; i++){
      if(lanepts[i]>0) lane_indset.push_back(i); // if(lanepts[i]>0) indset.push_back(i);
  }
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_result(new pcl::PointCloud<pcl::PointXYZRGB>);
  //vector<Vec3b> colors;
  //for (int i = 0; i < all_marks.size(); i++)
  //{
  //    int b = theRNG().uniform(0, 256);
  //    int g = theRNG().uniform(0, 256);
  //    int r = theRNG().uniform(0, 256);
  //    colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
  //}

  //int final_idx = 0;
  //for (auto mark : all_marks) {
  //    //custom_pcshow(mark->points3D);
  //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  //    toRGB(mark->points3D, rgb, colors[final_idx]);
  //    final_idx += 1;
  //    *final_result += *rgb;
  //}
  if (fieldname == "y") {
      Eigen::Matrix4d T;
      T.row(0) = Eigen::Vector4d(0., 1., 0., 0.);
      T.row(1) = Eigen::Vector4d(1., 0., 0., 0.);
      T.row(2) = Eigen::Vector4d(0., 0., 1., 0.);
      T.row(3) = Eigen::Vector4d(0., 0., 0., 1.);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_final_result(new pcl::PointCloud<pcl::PointXYZRGB>);
      //pcl::transformPointCloud(*final_result, *transformed_final_result, T);
      //final_result = transformed_final_result;
  }
  std::cout<<"Lane detection completed!"<<std::endl;

  // Write the extracted plane to disk
  
  custom_pcshow(all_lane_ptc, all_polylines);

  
  //if (!exists(output_file_path))
  if (cloud_output) {
      writer.write<pcl::PointXYZI>(output_file_path + ".pcd", *select(cloud, lane_indset), false);
      //writer.write<pcl::PointXYZRGB>(output_file_path + "_rgb.pcd", *final_result, false);
  }
    //write polyline
  if (shape_file_path.length() == 0)
      shape_file_path = output_file_path + ".shp";
  
    SHPHandle shp = SHPCreate(shape_file_path.c_str(), SHPT_ARCZ);
    SHPClose(shp);
    shp = SHPOpen(shape_file_path.c_str(), "r+b");
    int n_vertices = 0;
    vector<double>shp_x, shp_y, shp_z;
    vector<int>panstart;
    for (int i = 0; i < all_polylines.size(); i++) {
        //writer.write<pcl::PointXYZI>("lanemark_" + to_string(i) + ".pcd", *all_marks[i]->points3D, false);
        PolyLine& polyline = all_polylines[i];
        // polyline.smooth();
        if (polyline.points.size() == 0)
            continue;
        for (int cutidx = 0; cutidx < polyline.cuts.size(); cutidx ++) {
            
            panstart.push_back(shp_x.size());
            for (int i = (cutidx==0)?0:polyline.cuts[cutidx-1]; i < polyline.cuts[cutidx]; i++) {
                auto& point = polyline.points[i];
                if (fieldname == "x") {
                    shp_x.push_back(point.x);
                    shp_y.push_back(point.y);
                }
                else {
                    shp_x.push_back(point.y);
                    shp_y.push_back(point.x);
                }
                shp_z.push_back(point.z);
            }
        }
    }
    cout << shp_x.size() << endl;
    for (auto panstarti : panstart)
        cout << panstarti << " ";
    cout << endl;
    double* x_array = new double[shp_x.size()];
    double* y_array = new double[shp_x.size()];
    double* z_array = new double[shp_x.size()];
    int* panstart_array = new int[panstart.size()];
    memcpy(x_array, shp_x.data(), shp_x.size()*sizeof(double));
    memcpy(y_array, shp_y.data(), shp_x.size() * sizeof(double));
    memcpy(z_array, shp_z.data(), shp_x.size() * sizeof(double));
    memcpy(panstart_array, panstart.data(), panstart.size() * sizeof(int));
    SHPObject* obj = SHPCreateObject(SHPT_ARCZ, 2, panstart.size(), panstart_array, NULL, shp_x.size(), x_array, y_array, z_array, NULL);
    SHPWriteObject(shp, -1, obj);
    SHPDestroyObject(obj);
    SHPClose(shp);
    delete[]x_array;
    delete[]y_array;
    delete[]z_array;
    delete[]panstart_array;
  //  ofstream myfile(output_file_path + ".txt");
  //  for (int i = 0; i < all_marks.size(); i++) {

  //      writer.write<pcl::PointXYZI>("lanemark_" + to_string(i) + ".pcd", *all_marks[i]->points3D, false);
  //      PolyLine polyline(all_marks[i]->points3D);
  //      if (polyline.points.size() == 0)
  //          continue;
  //      
  //      //showpolyline(all_marks[i]->points3D, polyline.points);
  //      string line = to_string(i) + " ";
  //      for (auto & point : polyline.points) {
  //          if (fieldname == "x") {
  //              line += to_string(point.x) + ",";
  //              line += to_string(point.y) + ",";
  //          }
  //          else {
  //              line += to_string(point.y) + ",";
  //              line += to_string(point.x) + ",";
  //          }
  //          line += to_string(point.z) + " ";
  //          
  //      }
  //      line += "\n";
  //      myfile << line;
  //}
  //  myfile.close();

    
#ifdef DEBUG
  custom_pcshow(select(cloud, lane_indset));
#endif

}
int VERBOSE = 0;
void foo(double r) {
    cout << r << endl;
}
void
LaneDetection(string pcdfile, string save_path, function<void(double)> processor) {
    LanePar par;
    findLanesInPointcloud(pcdfile, par, false, save_path, processor);
}

void
findLanesInPointcloud(string pcdfile, string parfile){
  LanePar par(parfile);
  VERBOSE = par.verbose;
  findLanesInPointcloud(pcdfile, par, true, "", foo);
}

void
findLanesInPointcloud(string pcdfile){
  LanePar par;
  findLanesInPointcloud(pcdfile, par, true, "", NULL);
}


void extractLine(CloudPtr cloud, LanePar par) {
    DBSCAN dbscan;
    dbscan.setInputCloud(cloud);
    vector<int>dbclustering = dbscan.segment(0.6, 4);
    vector<vector<int>> clusters = dbscan.clusters;
    int i = 0;
    vector<LaneMark3D* > lanemarks;

    vector<Vec3b> colors;
    for (i = 0; i < clusters.size(); i++)
    {
        int b = theRNG().uniform(0, 256);
        int g = theRNG().uniform(0, 256);
        int r = theRNG().uniform(0, 256);
        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dbresult(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (i = 0; i < clusters.size(); i++) {

        for (auto k : clusters[i]) {
            pcl::PointXYZI tmp = cloud->points[k];
            pcl::PointXYZRGB rgb_tmp(tmp.x, tmp.y, tmp.z);
            rgb_tmp.r = colors[i][0]; 
            rgb_tmp.g = colors[i][1]; 
            rgb_tmp.b = colors[i][2]; 
            dbresult->points.push_back(rgb_tmp);
        }
    }
    dbresult->height = 1;
    dbresult->width = dbresult->points.size();

    custom_pcshow(dbresult);
    i = 0;
    for (auto& cluster : clusters) {
        if (cluster.size() > 30) {
            CloudPtr markcloud = select(cloud, cluster);
            //custom_pcshow(markcloud);
            LaneMark3D* mark_profile = new LaneMark3D(markcloud, i);
            lanemarks.push_back(mark_profile);
            i += 1;
        }
    }

    cout << "create " << lanemarks.size() << " lanemark profiles" << endl;

    for (i = 0; i < lanemarks.size(); i++) {
        for (int j = i + 1; j < lanemarks.size(); j++) {
            if (findp(lanemarks[i]) == findp(lanemarks[j]))
                continue;
            if (is_coline(lanemarks[i], lanemarks[j], cloud, 0.8)) {

                join(lanemarks[i], lanemarks[j]);

                cout << "join" << i << " " << j << " " << findp(lanemarks[i])->idx << " " << findp(lanemarks[j])->idx << endl;
            }
        }
    }
    cout << "group coline marks" << endl;




    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

    cout << "set color" << endl;
    for (auto& mark : lanemarks) {
        int line_idx = findp(mark)->idx;
        cout << line_idx << endl;
        for (auto& point : mark->points->points) {
            pcl::PointXYZRGB rgb_tmp(point.x, point.y, point.z);
            rgb_tmp.r = colors[line_idx][0]; 
            rgb_tmp.g = colors[line_idx][1]; 
            rgb_tmp.b = colors[line_idx][2]; 
            
            result->points.push_back(rgb_tmp);
        }

    }
    result->width = result->points.size();
    result->height = 1;
    cout << "visualize" << endl;
    custom_pcshow(result);
    for (i = 0; i < lanemarks.size(); i++) {
        delete lanemarks[i];
    }
}

void trackLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, LanePar par) {
    //store cloud in 2D grid
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PCA<pcl::PointXYZI> pca_model;

    vector<float> xrange = getXLimits(cloud);
    vector<float> yrange = getYLimits(cloud);
    int h, w;
    float grid_size = par.grid_size;
    h = ceil((yrange[1] - yrange[0]) / grid_size);
    w = ceil((xrange[1] - xrange[0]) / grid_size);

    vector<vector<int>> pixel2cloud(h*w);
    vector<int> cloud2pixel(cloud->points.size());
    vector<bool> visited(h * w, false);
    for (int pi = 0; pi < cloud->points.size(); pi++) {
        auto& point = cloud->points[pi];
        int i = floor((point.y - yrange[0]) / grid_size);
        int j = floor((point.x - xrange[0]) / grid_size);
        pixel2cloud[i * w + j].push_back(pi);
        cloud2pixel[pi] = i * w + j;
    }
    
    // loop
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (pixel2cloud[i * w + j].size() > 0 && visited[i*w+j] == false) {
                cout << i << " " << j << endl;
                // get points within pca_radius. Do pca to check linearity of point (i,j) 
                float pca_radius = 3.0/2;
                int search_d = ceil(pca_radius / grid_size);
                vector<pcl::PointXYZ> neighbor;
                for (int ni = max(0, i - search_d); ni < min(h, i + search_d + 1); ni++) {
                    for (int nj = max(0, j - search_d); nj < min(w, j + search_d + 1); nj++) {

                        if (pixel2cloud[ni * w + nj].size() > 0 && visited[ni*w+nj] == false) {
                            for (int k : pixel2cloud[ni * w + nj]) {
                                neighbor.push_back(pcl::PointXYZ(cloud->points[k].x, cloud->points[k].y, cloud->points[k].z));
                            }
                        }
                    }
                }
                cout << neighbor.size() << endl;
                if (neighbor.size() < 20)
                    continue;
                Mat data_pts = Mat(neighbor.size(), 2, CV_64F);
                for (int i = 0; i < data_pts.rows; i++)
                {
                    data_pts.at<double>(i, 0) = (double)neighbor[i].x;
                    data_pts.at<double>(i, 1) = (double)neighbor[i].y;
                }
                cout << "perform pca" << endl;
                //Perform PCA analysis
                PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);
                //Store the eigenvalues and eigenvectors
                vector<Eigen::Vector2d> eigen_vecs(2);
                vector<double> eigen_val(2);
                for (int i = 0; i < 2; i++)
                {
                    eigen_vecs[i] = Eigen::Vector2d(pca_analysis.eigenvectors.at<double>(i, 0),
                        pca_analysis.eigenvectors.at<double>(i, 1));
                    eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
                }
                
                cout << eigen_val[0] / eigen_val[1] << endl;
                
                // visualize the result
                //pcl::PointCloud<pcl::PointXYZ>::Ptr fake_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                //for (int i = 0; i < neighbor.size(); i++) {
                //    fake_cloud->points.push_back(pcl::PointXYZ(neighbor[i].x, neighbor[i].y, neighbor[i].z));
                //}
                //fake_cloud->width = fake_cloud->points.size();
                //fake_cloud->height = 1;
                //vector<pcl::PointXYZ> line;
                //eigen_vecs[0] /= eigen_vecs[0].norm();
                ///*line.push_back(pcl::PointXYZ(j, i, 0));
                //line.push_back(pcl::PointXYZ(j + 3 * eigen_vecs[0][0], i + 3 * eigen_vecs[0][1], 0));*/
                //pcl::PointXYZ c = cloud->points[pixel2cloud[i*w+j][0]];
                //line.push_back(pcl::PointXYZ(c.x, c.y, c.z));
                //line.push_back(pcl::PointXYZ(c.x + 3 * eigen_vecs[0][0], c.y + 3 * eigen_vecs[0][1], c.z));

                //custom_pcshow(fake_cloud, line);
                //custom_pcshow(cloud, line);
                // start a line from the center of neighbor
                if (eigen_val[0] / eigen_val[1] < 10)
                    continue;
                pcl::PointXYZ center(0, 0, 0);
                for (auto& point : neighbor) {
                    center.x += point.x / neighbor.size();
                    center.y += point.y / neighbor.size();
                    center.z += point.z / neighbor.size();
                }
                
                PolyLine polyline= PolyLine();
                polyline.add(center);
                float expand_d = 1.5; // length to expand
                for (int direction = 1; direction > -2; direction -= 2) 
                {
                    Eigen::Vector2f last_dir;
                    last_dir[0] = (float)eigen_vecs[0][0];
                    last_dir[1] = (float)eigen_vecs[0][1];
                    last_dir.normalize();
                    last_dir *= direction;
                    if (direction == -1) {
                        reverse(polyline.points.begin(), polyline.points.end());
                    }
                    cout << "start " << direction << endl;
                    while (1) {
                        
                        Eigen::Vector2f last_p;
                        last_p[0] = polyline.points.back().x;
                        last_p[1] = polyline.points.back().y;
                        vector<float>pointRadiusSquaredDistance;
                        vector<int>pointIdxRadiusSearch;
                        int nn1 = kdtree.radiusSearch(polyline.points.back(), (double)expand_d, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                        cout << "n neighbor: " << pointIdxRadiusSearch.size() << endl;

                        vector<int> best_inliers;
                        vector<int> cur_inliers;
                        int most_inlier = 0;
                        Eigen::Vector2f best_dir;
                        pcl::PointXYZ next_p;
                        float next_d;

                        for (int theta = -10; theta < 11; theta++) {
                            //cout << theta << endl;
                            float rad = M_PI / 180 * theta;
                            float cos_theta = cos(rad), sin_theta = sin(rad);
                            pcl::PointXYZ farthest_p;
                            float farthest_d = 0;
                            Eigen::Vector2f new_dir(cos_theta * last_dir[0] - sin_theta * last_dir[1], sin_theta * last_dir[0] + cos_theta * last_dir[1]);

                            float a = new_dir[1], b = -new_dir[0], c = new_dir[0] * last_p[1] - new_dir[1] * last_p[0];
                            cur_inliers.clear();
                            for (auto& pi : pointIdxRadiusSearch) {
                                //cout << pi << " " << cloud2pixel.size() << endl;
                                //cout << cloud2pixel[pi] << visited.size() << endl;
                                if (visited[cloud2pixel[pi]])
                                    continue;
                                auto& cur_p = cloud->points[pi];
                                float cur_d = (cur_p.x - last_p[0]) * new_dir[0] + (cur_p.y - last_p[1]) * new_dir[1];
                                //cout << "cur_d: " << cur_d << endl;
                                if (cur_d < 0)
                                    continue;

                                float dis = abs(a * cur_p.x + b * cur_p.y + c) / sqrt(a * a + b * b);
                                //cout << "dis: " << dis << endl;
                                if (dis < 0.1) {
                                    cur_inliers.push_back(i);
                                    if (cur_d > farthest_d) {
                                        farthest_d = cur_d;
                                        farthest_p = cur_p;
                                    }
                                }

                            }
                            if (cur_inliers.size() > most_inlier) {
                                best_inliers = cur_inliers;
                                most_inlier = cur_inliers.size();
                                best_dir = new_dir;
                                next_p = farthest_p;
                                next_d = farthest_d;
                                // best_theta = theta;
                            }
                        }
                        cout << best_inliers.size() << endl;
                        if (best_inliers.size() == 0) {
                            cout << "end" << endl;
                            //custom_pcshow(cloud, polyline.points);
                            
                            if (direction == -1)
                            {
                                // custom_pcshow(cloud, visited, cloud2pixel, polyline.points);
                                polyline.cuts.push_back(polyline.points.size());
                                if (polyline.points.size() > 1 && polyline.length > 1) {
                                    polyline.smooth();
                                    all_polylines.push_back(polyline);

                                }
                                
                            }
                            break;

                        }
                        for (auto& pi : pointIdxRadiusSearch) {
                            auto& cur_p = cloud->points[pi];
                            float a = best_dir[1], b = -best_dir[0], c = best_dir[0] * last_p[1] - best_dir[1] * last_p[0];
                            float dis = abs(a * cur_p.x + b * cur_p.y + c) / sqrt(a * a + b * b);
                            float cur_d = (cur_p.x - last_p[0]) * best_dir[0] + (cur_p.y - last_p[1]) * best_dir[1];
                            if ((cur_d > 0||polyline.points.size()>1) && dis < 0.8) {
                                //cout << "visited " << cloud2pixel[pi] / w << " " << cloud2pixel[pi] % w << endl;
                                visited[cloud2pixel[pi]] = true;
                            }
                        }
                        last_dir = best_dir;

                        polyline.add(next_p);
                        // 
                       
                    }
                }
            }
        }
    }
}

void extractLine(vector<LaneMark*>& grouped, Mat lane_mark, vector<vector<int>>& pixel2cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr whole, LanePar par) {
    //cout << "extractline" << endl;
    // create a point cloud
    int h = lane_mark.rows;
    int w = lane_mark.cols;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fake_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    fake_cloud->height = 1;
    vector<int> cloud2pixel;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            if (lane_mark.at<uchar>(i, j) > 0) {
                fake_cloud->points.push_back(pcl::PointXYZI((float)j, (float)i, 0.f, 1.));
                cloud2pixel.push_back(i * w + j);
            }
        }
    }
    fake_cloud->width = fake_cloud->points.size();
    //cout << "show fake cloud" << endl;
    //custom_pcshow(fake_cloud);
    vector<LaneMark*> lanemarks;
    DBSCAN dbscan;
    dbscan.setInputCloud(fake_cloud);
    std::vector<int> dbclustering = dbscan.segment(int(0.4/par.grid_size), 1);
    vector<vector<int>> clusters = dbscan.clusters;
    int i = 0;
    for (auto& cluster : clusters) {
        ofstream fout;
        
        if (cluster.size() > 20) {
            fout.open("cluster_" + to_string(i) + ".txt");
            
            CloudPtr markcloud = select(fake_cloud, cluster);
            //custom_pcshow(markcloud);
            for (auto& p : markcloud->points) {
                fout << p.x << " " << p.y << endl;
            }
            fout.close();
            LaneMark* mark_profile = new LaneMark(markcloud, pixel2cloud, whole, w);
            lanemarks.push_back(mark_profile);
            i += 1;
        }
    }

    //cout << "create " << lanemarks.size() << " lanemark profiles" << endl;
    grouped = group(lanemarks);
    //for (i = 0; i < lanemarks.size(); i++) {
    //    for (int j = i + 1; j < lanemarks.size(); j++) {
    //        if (findp(lanemarks[i]) == findp(lanemarks[j]))
    //            continue;
    //        if (is_coline(lanemarks[i], lanemarks[j])) {
    //            
    //            join(lanemarks[i], lanemarks[j]);
    //            cout << "join" << i << " " << j << " " << findp(lanemarks[i])->idx << " " << findp(lanemarks[j])->idx << endl;
    //        }
    //    }
    //}
    //cout << "group coline marks" << endl;
    vector<Vec3b> colors;

    cv::Mat line_img = cv::Mat::zeros(h, w, CV_8UC3);
    for (i = 0; i < lanemarks.size(); i++)
    {
        int b = theRNG().uniform(0, 256);
        int g = theRNG().uniform(0, 256);
        int r = theRNG().uniform(0, 256);
        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

    //cout << "set color" << endl;
    for (auto& mark : grouped) {
        int line_idx = findp(mark)->idx % (colors.size());
        //cout << line_idx << endl;
        for (auto& point : mark->points->points) {
            int pj = (int)point.x;
            int pi = (int)point.y;
            line_img.at<cv::Vec3b>(pi, pj) = colors[line_idx];
        }

    }

    if (VERBOSE == 2){
        cout << "visualize" << endl;
        cv::imshow("lines", line_img);
        cv::waitKey(0);
    }

    //for (i = 0; i < lanemarks.size(); i++) {
    //    delete lanemarks[i];
    //}
}

vector<int> findLaneByImage(vector<LaneMark*>& marks, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4d plane_model, float grid_size, LanePar par) {


    //return lane_mark_idx;
    vector<vector<int>> pixel2cloud;
    vector<int> lane_mark_idx;
    float img_x, img_y;
    img_x = getXLimits(cloud)[0];
    img_y = getYLimits(cloud)[0];
    Mat uimage = toImage(cloud, plane_model, grid_size, pixel2cloud);

    Mat lanemark = findLaneInImage(uimage);
    int h = uimage.rows;
    int w = uimage.cols;
    //for (int i = 0; i < h; i++) {
    //    for (int j = 0; j < w; j++) {
    //        // find the points corresponding to pixel at [i,j]
    //        if (lanemark.at<uchar>(i, j) > 0) {
    //            for (int corres_idx : pixel2cloud[i * w + j]) {
    //                lane_mark_idx.push_back(corres_idx);
    //            }
    //        }
    //    }
    //}
    //CloudPtr lanemark_cloud = select(cloud, lane_mark_idx);
    // pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZI> ("lane_mark.pcd", *lanemark_cloud, false);
    lanemark = removeFalsePostive(lanemark, par);

    lane_mark_idx.clear();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            // find the points corresponding to pixel at [i,j]
            if (lanemark.at<uchar>(i, j) > 0) {
                for (int corres_idx : pixel2cloud[i * w + j]) {
                    lane_mark_idx.push_back(corres_idx);
                }
            }
        }
    }
    CloudPtr lane_mark_cloud = select(cloud, lane_mark_idx);
    *all_lane_ptc += *lane_mark_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mark_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& point : lane_mark_cloud->points) {
        mark_cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    mark_cloud->width = mark_cloud->points.size();
    mark_cloud->height = 1;
    cout << "track line" << endl;
    trackLine(mark_cloud, par);
    //extractLine(marks, lanemark, pixel2cloud, cloud, par);
    

    //back project points and lines in each mark from image to 3D:
    
    //for (auto mark : marks) {
    //    mark->min_line[2] += img_x/grid_size;
    //    mark->min_line[3] += img_y/grid_size;
    //    mark->max_line[2] += img_x / grid_size;
    //    mark->max_line[3] += img_y / grid_size;
    //    mark->maxx += img_x / grid_size;
    //    mark->maxy += img_y / grid_size;
    //    mark->minx += img_x / grid_size;
    //    mark->miny += img_y / grid_size;
    //    for (auto& point : mark->points->points) {
    //        point.x += img_x / grid_size;
    //        point.y += img_y / grid_size;
    //    }
    //}


    if (VERBOSE == 1) {
        imshow("remove fp", lanemark);
        waitKey(0);
    }
    


    
    


    return lane_mark_idx;

}


Mat removeFalsePostive(Mat lane_mark, LanePar par) {
    vector<int> cloud2pixel;
    vector<vector<int>> clusters = ImageDbscan(lane_mark, cloud2pixel, par.dbscan_dis, 2);
    int h = lane_mark.rows, w = lane_mark.cols;
    Mat ret = Mat::zeros(h, w, CV_8UC1);
    for (auto& cluster : clusters) {
        if (cluster.size() > par.lanemark_minpts) {
            for (int point : cluster) {
                int pixel_idx = cloud2pixel[point];
                int i = pixel_idx / w;
                int j = pixel_idx % w;
                ret.at<uchar>(i, j) = 255;
            }
        }
    }
    return ret;
}



