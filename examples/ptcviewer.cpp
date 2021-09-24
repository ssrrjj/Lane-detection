/*
 * test_pcshow.cpp
 *
 *  Created on: May 21, 2021
 *      Author: aina
 */
#include "utils.h"
#include "pcshow.h"


int
main (int argc, char* argv[])
{
  string pcdfile;
  if(argc>=2){
      pcdfile = argv[1];
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI (
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (pcdfile, *cloudXYZI);
  std::cout<<"Please press 'r' to view the point cloud ..."<<std::endl;
  custom_pcshow(cloudXYZI);

  return 0;
}

