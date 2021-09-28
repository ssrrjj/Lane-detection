/*
 * lanepar.cpp
 *
 *  Created on: Jun 1, 2021
 *      Author: aina
 */

#include "lanepar.h"

void
LanePar::parseLine(string& line){
  stringstream ss(line);
  string key;
  string value;
  getline(ss, key, ' ');
  while(getline(ss, value, ' ')){
      if(!value.empty()) break;
  } 
  cout << key << " " << value << endl;
  if(key=="SUBREGION_WIDTH")
      subregion_width = stof(value);
  if(key=="PLANE_DIST_THRESHOLD")
      plane_dist_threshold = stof(value);
  if(key=="LANE_WIDTH")
      lane_width = stof(value);
  if(key=="DBSCAN_MINPTS")
      dbscan_minpts = stoi(value);
  if(key=="LANEMARK_MINPTS")
      lanemark_minpts = stoi(value);
  if (key == "DOWNSAMPLE") {
      if (stoi(value) == 0)
          downsample = false;
      else
          downsample = true;
  }
  if (key == "GRID_SIZE") {
      cout << "get grid_size" << endl;
      grid_size = stof(value);
  }
  if (key == "VERBOSE") {
      verbose = stoi(value);
  }
  if (key == "SAVE_TO")
      save_to = value;
  if (key == "DBSCAN_DIS")
      dbscan_dis = stof(value);
  if (key == "START")
      start = stoi(value);
  
      
}

void
LanePar::parseParfile(string parfile){
  std::ifstream cFile (parfile);
  if (cFile.is_open())
  {
      std::string line;
      while(getline(cFile, line))
     {
          if( line.empty() || line[0] == '#' )
          {
              continue;
          }
          parseLine(line);
      }
  }
  else
  {
      std::cerr << "Couldn't open config file for reading.\n";
  }
}


