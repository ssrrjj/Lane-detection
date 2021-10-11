Lanedet - Lane detection for 3D point cloud
===========================================

Overview
--------

Lanedet is a tool for 3D point cloud lane detection written in C++.   

System Requirements 
-------------------
Windows or Ubuntu (20.04)
Point Cloud Library (PCL) https://pointclouds.org/ 
 (1.11) (Under Ubuntu 20.04, can use 1.10 with modified pcl/impl/point_types.hpp)  
CMake (3.21.2)  
Open3D  


Compiling 
---------
The compilation has been tested on Ubuntu 18.04 with CMake 3.10.2 and PCL 1.8.1. Compiling on Windows should be similar.
 
Step 1: Download the source code or clone it from git repository.  
Step 2: In command line, type and run the following  
cd laneDetection  
mkdir build; cd build     
cmake -DOpen3D_ROOT=${HOME}/open3d_install/open3d_install ..   
make


Usage
-----
lanedet pcdfile
.
pcdfile is the point cloud input file in pcd format.  
The output of detected lane points will be written to pcdfile.output. 


Install dependencies under Ubuntu 20.04
---------------------------------------

PCL Installation

sudo apt install libpcl-dev 
modify pcl/impl/point_types.hpp 

Open3D Installation

git clone --recursive https://github.com/intel-isl/Open3D   
git submodule update --init --recursive # You can also update the submodule manually   
mkdir build && cd build   
cmake -DBUILD_EIGEN3=ON -DBUILD_GLEW=ON -DBUILD_GLFW=ON -DBUILD_JSONCPP=ON -DBUILD_PNG=ON \   
      -DGLIBCXX_USE_CXX11_ABI=ON -DPYTHON_EXECUTABLE=/usr/bin/python -DBUILD_UNIT_TESTS=ON ..   
make -j4   
sudo make install  

Building without -DGLIBCXX_USE_CXX11_ABI=ON will have link error when linking Open3D with PCL under Ubuntu 20.04.   


