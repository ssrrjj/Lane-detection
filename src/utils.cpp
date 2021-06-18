/*
 * utils.cpp
 *
 *  Created on: May 20, 2021
 *      Author: aina
 */

#include "utils.h"

using namespace std;
using namespace cv;

vector<float>
getXLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  vector<float> xlimits(2,0.0);
  if(cloud->points.size()==0) return xlimits;
  xlimits[0]=cloud->points[0].x;
  xlimits[1]=xlimits[0];
  for(const auto & pt: cloud->points){
      if(pt.x>xlimits[1]) xlimits[1] = pt.x;
      if(pt.x<xlimits[0]) xlimits[0] = pt.x;
  }
  return xlimits;
}

vector<float>
getYLimits(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  vector<float> ylimits(2,0.0);
  if(cloud->points.size()==0) return ylimits;
  ylimits[0]=cloud->points[0].y;
  ylimits[1]=ylimits[0];
  for(const auto & pt: cloud->points){
      if(pt.y>ylimits[1]) ylimits[1] = pt.y;
      if(pt.y<ylimits[0]) ylimits[0] = pt.y;
  }
  return ylimits;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointIndices::Ptr &inliers)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);

  // Extract the inliers
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*output);

  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr select(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::vector<int> &inliers)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = 0; i<inliers.size(); i++){
      output->points.push_back(cloud->points[inliers[i]]);
  }
  output->width = output->points.size();
  output->height = 1; // 1 means unorganized point cloud
  return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string fieldname, float lb, float ub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass(false); // Initializing with true will allow us to extract the removed indices
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldname);
  pass.setFilterLimits (lb, ub);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByField(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<int> &inlierIndset, string fieldname, float lb, float ub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass(true); // Initializing with true will allow us to extract the removed indices
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldname);
  pass.setFilterLimits (lb, ub);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered); // filtered point cloud
  pass.filter (inlierIndset); // TODO filtered index array, not efficient
  return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float lb, float ub)
{
  return filterByField(cloud, "intensity", lb, ub);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, vector<int> &inlierIndset, float lb, float ub)
{
  return filterByField(cloud, inlierIndset, "intensity", lb, ub);
}

vector<int> indexMapping(vector<int>& X, vector<int>&Y)
{
  // returns X[Y]

  std::vector<int> Z;
  for(const auto & p: Y){
      Z.push_back(X[p]);
  }
  return Z;
}

float mean(vector<float>& v)
{
  if(v.empty()) return 0.0;
  float total = 0.0;
  for(auto &p: v) total+=p;
  return total/v.size();
}

float maxvalue(vector<float>& v)
{
  float r=0.0;
  for(auto &p: v){
      if(p>r) r=p;
  }
  return r;
}

float stdv(vector<float>& v)
{
  float mu = mean(v);
  float total = 0.0;
  for(const auto & p: v ){
      total = total+(p-mu)*(p-mu);
  }
  float sigma = sqrt(total/(v.size()-1.0));
  return sigma;
}

vector<int> getHist(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int n_bins, double & bin_width, double & min_intensity) {
  double max_intensity = -1, intensity = 0;
  min_intensity = 2;
  vector<int> bins(n_bins, 0);
  for (int i = 0; i < cloud->points.size(); ++ i) {
    intensity = cloud->points[i].intensity;
    if (intensity > max_intensity)
      max_intensity = intensity;
    if (intensity< min_intensity)
      min_intensity = intensity;
  }
  bin_width = (max_intensity - min_intensity) / n_bins;
  for (int i = 0; i < cloud->points.size(); ++ i) {
    intensity = cloud->points[i].intensity;
    int which_bin = floor((intensity - min_intensity)/bin_width);
    if (which_bin >= n_bins)
      which_bin = n_bins - 1;
    bins[which_bin] += 1;
  }
  cout<<"maxi:"<<max_intensity<<endl;
  cout<<"mini:"<<min_intensity<<endl;
  return bins;
}

void drawHist(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int n_bins, int highlight) {
  
  int rows = 0, cols = n_bins * 3;
  double bin_width, min_intensity;
  vector<int> bins = getHist(cloud, n_bins, bin_width, min_intensity);

  rows = *max_element(bins.begin(), bins.end()) + 10;
  for (int i = 0 ; i < n_bins; ++ i)
    cout<<bins[i]<<endl;
  double factor = 300.0/rows;
  cout<< rows <<" "<< cols<<endl;
  Mat3b hist = Mat3b(300, cols, Vec3b(0, 0, 0));
  for (int i = 0; i < n_bins; ++ i) {
    if (i != highlight)
      rectangle(hist, Point(i*3, (rows - bins[i])*factor), Point((i+1)*3, 300), (i%2) ? Scalar(0, 100, 255) : Scalar(0, 0, 255), FILLED);
    else
      rectangle(hist, Point(i*3, (rows - bins[i])*factor), Point((i+1)*3, 300), Scalar(255, 100, 255), FILLED);
  }
  imshow("HIstogram", hist);
  waitKey();
  return;
}

double Ostu_thresholding(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int n_bins) {
  double min_intensity = 0, bin_width = 0;
  vector<int> hist = getHist(cloud, n_bins, bin_width, min_intensity);
  double q1=0, q2=0, mu1=0, mu2=0, sum = 0,sumB = 0, threshold = 0, var_max = 0;
  double sigma = 0;
  for (int i = 0; i < n_bins; ++i) {
    sum += i*hist[i];
  }
  for (int i = 0; i < n_bins; ++i) {
    q1 += hist[i];
    if (q1 == 0)
      continue;
    q2 = cloud->points.size() - q1;

    sumB += i*hist[i];
    mu1 = sumB/q1;
    mu2 = (sum-sumB)/q2;
    sigma = q1*q2*pow((mu1-mu2), 2);
    if (sigma > var_max) {
      threshold = i;
      var_max = sigma;
    }
  }
  // threshold += 3;
  // if (threshold>= n_bins)
  //   threshold = n_bins - 1;
  drawHist(cloud, 100, int(threshold));
  std::cout<<threshold<<std::endl;
  threshold = (threshold+0.5)*bin_width;
  std::cout<<threshold<<std::endl;
  return threshold;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
OstuFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, string fieldname) {
  //remove outlier
  // DBSCAN dbscan;
  // dbscan.setInputCloud (cloud);
  // std::vector<int> dbclustering = dbscan.segment (0.2, 10);
  // std::vector<std::vector<int>> & clusters = dbscan.clusters;
  // int max_cluster, max_size = 0;
  // for (int i = 0 ; i < clusters.size(); ++i) {
  //   if (clusters[i].size()>max_size) {
  //     std::cout<<clusters[i].size()<<std::endl;
  //     max_size = clusters[i].size();
  //     max_cluster = i;
  //   }
  // }
  // cloud = select(cloud, clusters[max_cluster]);
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZI> ("subplane.pcd", *cloud, false);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  // sor.setInputCloud (cloud);
  // sor.setMeanK (30);
  // sor.setStddevMulThresh (2.5);
  // sor.filter (*cloud);
  cloud = regionGrowSeg(cloud);

  std::cout<<"Remove outliers"<<endl;
  custom_pcshow(cloud);
  vector<float> xlimits = getXLimits(cloud);
  vector<float> ylimits = getYLimits(cloud);
  double lb, ub;
  string otherfield;

  if (fieldname == "x") {
    lb = ylimits[0];
    ub = ylimits[1];
    otherfield = "y";
  }
  else {
    lb = xlimits[0];
    ub = xlimits[1];
    otherfield = "x";
  }
  vector<int> indsetsubregion;
  vector<int> indsethighintensity;
  vector<int> extracted;
  std::cout<<"xbound:"<<xlimits[0]<<" "<<xlimits[1]<<std::endl;
  std::cout<<"ybound:"<<ylimits[0]<<" "<<ylimits[1]<<std::endl;
  std::cout<<"bound:"<<lb<<" "<<ub<<std::endl;
  double subregion_width = 10;
  double subregion_ub = lb + subregion_width;
  double subregion_lb = lb;

  while (subregion_lb < ub) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud = filterByField(cloud, indsetsubregion,\
     otherfield, subregion_lb, subregion_ub);
    std::cout<<"Show sub cloud "<<std::endl;
    custom_pcshow(sub_cloud);
    double ostu_th = Ostu_thresholding(sub_cloud, 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_highintensity = filterByIntensity(sub_cloud, indsethighintensity, ostu_th);
    std::cout<<"Show sub cloud filtered by ostu_th" <<std::endl;
    custom_pcshow(sub_highintensity);
    vector<int> subextracted = indexMapping(indsetsubregion, indsethighintensity);
    extracted.insert(extracted.end(), subextracted.begin(), subextracted.end());
    subregion_lb = subregion_ub;
    subregion_ub = subregion_lb + subregion_width;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud = select(cloud, extracted);
  custom_pcshow(extracted_cloud);
  return extracted_cloud;
}

pcl::PointCloud <pcl::PointXYZI>::Ptr
regionGrowSeg(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
  reg.setMinClusterSize (10000);
  //reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  int counter = 0;
  int max_size = 0;
  int max_cluster = 0;
  for (int i = 0; i < clusters.size(); ++i) {
    std::cout<<clusters[i].indices.size()<<std::endl;
    if (clusters[i].indices.size() > max_size) {

      max_size = clusters[i].indices.size();
      max_cluster = i;
    }
  }

  pcl::PointCloud <pcl::PointXYZI>::Ptr biggest_cluster = select(cloud, clusters[max_cluster].indices);
  return biggest_cluster;
}