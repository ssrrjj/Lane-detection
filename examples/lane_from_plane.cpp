#include "lanedetection.h"
int main(int argc, char* argv[])
{
  string pcdfile;
  string parfile;
  if(argc>=2){
      pcdfile = argv[1];
      if(argc>=3) parfile = argv[2];
  }else{
    std::cout<<"Usage: lanedet [pcdfile] [parfile] \n"<<std::endl;
    std::cout<<"Please provide a point cloud input file in pcd format."<<std::endl;
    std::cout<<"Exit without lane detection. \n"<<std::endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile (pcdfile, *cloud);
  // double th = Otsu_thresholding(cloud, 200);
  // std::cout<<th<<endl;
  // cloud = filterByIntensity(cloud, th);
  // custom_pcshow(cloud);
  //OtsuFilter(cloud, "x");
  toImage(cloud);
  return (0);
}