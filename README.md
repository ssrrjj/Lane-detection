Lanedet - Lane detection for 3D point cloud
===========================================

Overview
--------

Lanedet is a tool for 3D point cloud lane detection written in C++.   

System Requirements 
-------------------
Point Cloud Library (PCL) https://pointclouds.org/  
CMake


Compiling 
---------
The compilation has been tested on Ubuntu 18.04 with CMake 3.10.2 and PCL 1.8.1. Compiling on Windows should be similar.
 
Step 1: Download the source code or clone it from git repository.  
Step 2: In command line, type and run the following  
cd laneDetection  
make build  
cmake ../  
make


Usage
-----
lanedet pcdfile

pcdfile is the point cloud input file in pcd format.  
The output of detected lane points will be written to pcdfile.output. 

