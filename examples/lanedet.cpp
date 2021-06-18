/*
 * lanedet.cpp
 *
 *  Created on: May 30, 2021
 *      Author: aina
 */

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
  findLanesInPointcloud(pcdfile, parfile); // findLanesInPointcloud(pcdfile, par);
  return (0);
}


