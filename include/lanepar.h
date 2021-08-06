/*
 * lanepar.h
 *
 *  Created on: Jun 1, 2021
 *      Author: aina
 */

#ifndef INCLUDE_LANEPAR_H_
#define INCLUDE_LANEPAR_H_

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace std;

class LanePar{
public:
  LanePar(){
    subregion_width = 50.0;
    plane_dist_threshold = 0.05;
    lane_width = 0.2;
    dbscan_minpts = 10;
    lanemark_minpts = 100;
    verbose = 0;
    save_to = "result";
    dbscan_dis = 10.0;
    start = 0;
  }
  LanePar(string parfile):LanePar(){
    if(!parfile.empty()) parseParfile(parfile);
  }
  double subregion_width;
  double plane_dist_threshold;
  double lane_width;
  int dbscan_minpts;
  int lanemark_minpts;
  int verbose;
  string save_to;
  float dbscan_dis;
  int start;
  void parseLine(string& line);
  void parseParfile(string parfile);
  
};



#endif /* INCLUDE_LANEPAR_H_ */
