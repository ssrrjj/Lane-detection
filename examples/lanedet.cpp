/*
 * lanedet.cpp
 *
 *  Created on: May 30, 2021
 *      Author: aina
 */

#include "lanedetection.h"
//#include "Python.h"
#include <stdio.h>
#include <lanemark.h>

int main(int argc, char* argv[])
{
    //FILE* fp;
    //fp = fopen("../../../python/dbscan.py", "r");
    //wchar_t* program = Py_DecodeLocale(argv[0], NULL);
    //if (program == NULL) {
    //    fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
    //    exit(1);
    //}
    //Py_SetProgramName(program);  /* optional but recommended */
    //Py_Initialize();
    //PyRun_SimpleFile(fp, "dbscan.py");
    //if (Py_FinalizeEx() < 0) {
    //    exit(120);
    //}
    //PyMem_RawFree(program);
    
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


