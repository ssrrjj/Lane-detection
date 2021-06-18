/*
 * test_utils.cpp
 *
 *  Created on: May 26, 2021
 *      Author: aina
 */

#include "utils.h"

void test_filterByIntensity(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile ("extracted_plane_bypieces.pcd", *planeCloud);

#ifdef DEBUG
  std::cout<<"In test_filterByIntensity: planeCloud->points.size() = "<<planeCloud->points.size()<<std::endl;
#endif

  float INTENSITY_THRESHOLD = 0.1;
  vector<int> indsetFiltered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptc = filterByIntensity(planeCloud, indsetFiltered, INTENSITY_THRESHOLD);

#ifdef DEBUG
  std::cout<<"In test_filterByIntensity: indsetFiltered.size() = "<<indsetFiltered.size()<<std::endl;
#endif
}

int
main ()
{
  test_filterByIntensity();
}


